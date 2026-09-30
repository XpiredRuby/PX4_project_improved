#!/usr/bin/env python3
import math
import threading
import time

from pymavlink import mavutil

from magless_bootstrap import (
    capture_launch_reference,
    recover_local_home,
    run_magless_yaw_bootstrap,
    send_attitude_target,
    wait_for_bootstrap_ready,
)
from mission_state import FailureAction, MissionOutcome
from PID_position_new import PositionController
from px4_policy import audit_px4_configuration
from run_record import RunRecord


PRESTREAM_SECONDS = 2.0
MODE_TIMEOUT = 8.0
ARM_TIMEOUT = 8.0

PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_AUTO_SUB_MODE_LAND = 6

POST_DISARM_CONFIRM_SECONDS = 1.0
ACTUATOR_MAX_AGE_SECONDS = 1.5
PROPULSION_STOP_THRESHOLD = 0.01


def snapshot(controller):
    with controller.state_lock:
        return controller.state.mode, controller.state.armed, controller.state.yaw


def heartbeat_snapshot(controller):
    with controller.state_lock:
        if not controller.state.heartbeat_received:
            return None, None, None
        return (
            controller.state.heartbeat_main_mode,
            controller.state.heartbeat_sub_mode,
            controller.state.armed,
        )


def landing_snapshot(controller):
    with controller.state_lock:
        state = controller.state
        age = (
            float("inf")
            if state.extended_state_received_at is None
            else max(0.0, time.monotonic() - state.extended_state_received_at)
        )
        return (
            state.landed_state,
            age,
            state.message_counts.get("EXTENDED_SYS_STATE", 0),
        )


def propulsion_snapshot(controller):
    with controller.state_lock:
        state = controller.state
        age = (
            float("inf")
            if state.actuator_received_at is None
            else max(0.0, time.monotonic() - state.actuator_received_at)
        )
        return (
            tuple(float(value) for value in state.actuator_outputs[:4]),
            age,
            state.message_counts.get("ACTUATOR_OUTPUT_STATUS", 0),
        )


def command_ack_snapshot(controller, command=None):
    with controller.state_lock:
        state = controller.state
        if command is not None:
            return state.command_ack_by_command.get(command, (0, -1, -1))
        return (
            state.message_counts.get("COMMAND_ACK", 0),
            state.command_ack_command,
            state.command_ack_result,
            state.command_ack_progress,
        )


def send_neutral(controller):
    _, _, yaw = snapshot(controller)
    controller.send_velocity(0.0, 0.0, 0.0, yaw)


def stream_for(controller, seconds, send_setpoint=None):
    send_setpoint = send_setpoint or (lambda: send_neutral(controller))
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        send_setpoint()
        time.sleep(controller.control_dt)


def request_mode(controller, mode_name):
    master = controller.master
    mode_name = mode_name.upper()

    with controller.mav_send_lock:
        if mode_name == "OFFBOARD":
            custom_mode = PX4_MAIN_MODE_OFFBOARD << 16
        elif mode_name == "LAND":
            custom_mode = (
                (PX4_AUTO_SUB_MODE_LAND << 24)
                | (PX4_MAIN_MODE_AUTO << 16)
            )
        else:
            master.set_mode(mode_name)
            return

        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode,
        )


def ensure_mode(
    controller,
    mode_name,
    timeout,
    keep_streaming,
    send_setpoint=None,
):
    mode_name = mode_name.upper()

    def selected(main_mode, sub_mode):
        if mode_name == "OFFBOARD":
            return main_mode == PX4_MAIN_MODE_OFFBOARD
        if mode_name == "LAND":
            return (
                main_mode == PX4_MAIN_MODE_AUTO
                and sub_mode == PX4_AUTO_SUB_MODE_LAND
            )
        return False

    deadline = time.monotonic() + timeout
    next_request = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            request_mode(controller, mode_name)
            next_request = now + 1.0
        if keep_streaming:
            if send_setpoint is None:
                send_neutral(controller)
            else:
                send_setpoint()
        main_mode, sub_mode, armed = heartbeat_snapshot(controller)
        if selected(main_mode, sub_mode):
            return
        time.sleep(controller.control_dt)

    main_mode, sub_mode, armed = heartbeat_snapshot(controller)
    raise RuntimeError(
        f"PX4 did not enter {mode_name} "
        f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
    )


def request_arm(controller):
    command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    before_sequence, _, _ = command_ack_snapshot(controller, command)
    with controller.mav_send_lock:
        controller.master.mav.command_long_send(
            controller.master.target_system,
            controller.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    return before_sequence


def wait_for_command_ack(controller, command, after_sequence, timeout):
    accepted = {
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    }
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sequence, result, progress = command_ack_snapshot(controller, command)
        if sequence > after_sequence:
            if result not in accepted:
                raise RuntimeError(
                    f"PX4 rejected command {command}: result={result}"
                )
            print(
                f"[runner] Command {command} acknowledged "
                f"(result={result}, progress={progress})"
            )
            return
        time.sleep(0.02)
    raise TimeoutError(f"No COMMAND_ACK received for command {command}")


def wait_for_armed(controller, timeout, send_setpoint=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if send_setpoint is None:
            send_neutral(controller)
        else:
            send_setpoint()
        _, _, armed = heartbeat_snapshot(controller)
        if armed is True:
            return
        time.sleep(controller.control_dt)
    main_mode, sub_mode, armed = heartbeat_snapshot(controller)
    raise RuntimeError(
        "PX4 did not arm "
        f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
    )


def wait_for_initial_ground_state(controller, timeout=5.0):
    deadline = time.monotonic() + timeout
    on_ground = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    while time.monotonic() < deadline:
        landed_state, age, _ = landing_snapshot(controller)
        if landed_state == on_ground and age <= 1.5:
            return
        time.sleep(0.05)
    landed_state, age, _ = landing_snapshot(controller)
    raise RuntimeError(
        "PX4 did not report a fresh ON_GROUND state before arming "
        f"(landed_state={landed_state}, age={age:.2f}s)"
    )


def wait_for_native_landing(
    controller,
    after_sequence,
    timeout,
    post_disarm_confirm_s=POST_DISARM_CONFIRM_SECONDS,
    phase="PX4_LAND",
):
    deadline = time.monotonic() + timeout
    on_ground = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    safe_disarm_since = None

    while time.monotonic() < deadline:
        controller.log_native_landing_sample(phase=phase)
        main_mode, sub_mode, armed = heartbeat_snapshot(controller)
        landed_state, age, sequence = landing_snapshot(controller)
        fresh_on_ground = (
            sequence > after_sequence
            and age <= 1.5
            and landed_state == on_ground
        )
        if armed is False:
            if not fresh_on_ground:
                raise RuntimeError(
                    "Vehicle disarmed without a fresh PX4 ON_GROUND report"
                )
            outputs, actuator_age, _ = propulsion_snapshot(controller)
            propulsion_stopped = (
                actuator_age <= ACTUATOR_MAX_AGE_SECONDS
                and all(math.isfinite(value) for value in outputs)
                and all(
                    abs(value) <= PROPULSION_STOP_THRESHOLD
                    for value in outputs
                )
            )
            if propulsion_stopped:
                if safe_disarm_since is None:
                    safe_disarm_since = time.monotonic()
                if (
                    time.monotonic() - safe_disarm_since
                    >= post_disarm_confirm_s
                ):
                    return
            else:
                safe_disarm_since = None
            time.sleep(0.05)
            continue
        safe_disarm_since = None
        if (
            main_mode != PX4_MAIN_MODE_AUTO
            or sub_mode != PX4_AUTO_SUB_MODE_LAND
        ):
            controller.set_failure_action(FailureAction.PX4_FAILSAFE)
            raise RuntimeError(
                "PX4 left LAND before auto-disarm "
                f"(main_mode={main_mode}, sub_mode={sub_mode})"
            )
        time.sleep(0.05)

    main_mode, sub_mode, armed = heartbeat_snapshot(controller)
    landed_state, age, _ = landing_snapshot(controller)
    outputs, actuator_age, _ = propulsion_snapshot(controller)
    raise TimeoutError(
        f"PX4 LAND did not reach confirmed safe shutdown within {timeout:.0f}s "
        f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed}, "
        f"landed_state={landed_state}, landed_age={age:.2f}s, "
        f"actuator_age={actuator_age:.2f}s, propulsion={outputs})"
    )


def wait_for_failsafe_landing(
    controller,
    after_sequence,
    landing_timeout,
    takeover_timeout=MODE_TIMEOUT,
    post_disarm_confirm_s=POST_DISARM_CONFIRM_SECONDS,
):
    controller.begin_failsafe_handoff()
    deadline = time.monotonic() + takeover_timeout
    unexpected_mode = None
    while time.monotonic() < deadline:
        controller.log_native_landing_sample(phase="PX4_FAILSAFE")
        main_mode, sub_mode, armed = heartbeat_snapshot(controller)
        failsafe_land_active = (
            main_mode == PX4_MAIN_MODE_AUTO
            and sub_mode == PX4_AUTO_SUB_MODE_LAND
        )
        if armed is False or failsafe_land_active:
            wait_for_native_landing(
                controller,
                after_sequence=after_sequence,
                timeout=landing_timeout,
                post_disarm_confirm_s=post_disarm_confirm_s,
                phase="PX4_FAILSAFE",
            )
            return
        if main_mode != PX4_MAIN_MODE_OFFBOARD:
            unexpected_mode = (main_mode, sub_mode)
        time.sleep(0.05)
    if unexpected_mode is not None:
        main_mode, sub_mode = unexpected_mode
        raise RuntimeError(
            "PX4 failsafe did not settle into LAND after leaving OFFBOARD "
            f"(main_mode={main_mode}, sub_mode={sub_mode})"
        )
    raise TimeoutError("PX4 did not take control after Offboard commands stopped")


def final_state_snapshot(controller):
    main_mode, sub_mode, armed = heartbeat_snapshot(controller)
    landed_state, _, _ = landing_snapshot(controller)
    return {
        "heartbeat_main_mode": main_mode,
        "heartbeat_sub_mode": sub_mode,
        "armed": armed,
        "landed_state": landed_state,
        "mission_phase": str(controller.phase_snapshot()),
        "failure_action": str(controller.failure_action_snapshot()),
    }


def classify_failure(controller, vehicle_was_armed):
    if controller.failure_action_snapshot() == FailureAction.PX4_FAILSAFE:
        return MissionOutcome.PX4_FAILSAFE
    if vehicle_was_armed:
        return MissionOutcome.ABORTED_TO_LAND
    return MissionOutcome.PREARM_REJECTED


def main():
    run_record = RunRecord()
    c = PositionController()
    run_record.attach_context(c, {})
    worker = None
    controller_stopped = False
    vehicle_was_armed = False
    outcome = MissionOutcome.PREARM_REJECTED
    outcome_reason = "Mission did not reach arming"
    cleanup_status = "not_required"
    cleanup_error = None

    try:
        print("[runner] Connecting to PX4...")
        c.connect()
        px4_parameters = audit_px4_configuration(c)
        run_record.attach_context(c, px4_parameters)

        c.start_receiver()
        c.wait_for_fresh_telemetry()
        wait_for_bootstrap_ready(c)
        wait_for_initial_ground_state(c)
        launch_reference = capture_launch_reference(c)

        def send_level_disarmed():
            send_attitude_target(c, 0.0, 0.0)

        print(
            f"[runner] Pre-streaming level attitude setpoints for "
            f"{PRESTREAM_SECONDS:.1f}s..."
        )
        stream_for(c, PRESTREAM_SECONDS, send_level_disarmed)

        print("[runner] Requesting attitude OFFBOARD...")
        ensure_mode(
            c,
            "OFFBOARD",
            MODE_TIMEOUT,
            keep_streaming=True,
            send_setpoint=send_level_disarmed,
        )
        print("[runner] Attitude OFFBOARD confirmed")

        print("[runner] Requesting arm...")
        ack_sequence = request_arm(c)
        wait_for_command_ack(
            c,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            ack_sequence,
            ARM_TIMEOUT,
        )
        wait_for_armed(c, ARM_TIMEOUT, send_level_disarmed)
        vehicle_was_armed = True
        print("[runner] Armed confirmed")

        run_magless_yaw_bootstrap(
            c,
            hover_thrust=float(px4_parameters["MPC_THR_HOVER"]),
            px4_max_thrust=float(px4_parameters["MPC_THR_MAX"]),
        )
        c.wait_for_preflight_ready(timeout=5.0)
        home_snapshot = recover_local_home(c, launch_reference)
        c.configure_cruise_height(home_snapshot)
        c.validate_mission_plan()
        c.setup_logger(run_id=run_record.run_id)
        c.initialize_target(start_from_current=True)
        print("[runner] Starting position controller...")
        worker = threading.Thread(
            target=c.run,
            name="position-controller",
            daemon=True,
        )
        worker.start()

        deadline = time.monotonic() + c.config.mission_timeout_s
        while time.monotonic() < deadline:
            if c.worker_error is not None:
                raise RuntimeError(f"Controller failed: {c.worker_error}")
            if c.handoff_ready_event.wait(timeout=0.05):
                break
            if not worker.is_alive():
                phase = c.phase_snapshot()
                raise RuntimeError(
                    f"Controller stopped before handoff; phase={phase}"
                )
        else:
            if not c.request_return_home("mission timeout"):
                phase = c.phase_snapshot()
                raise TimeoutError(
                    f"Mission did not reach handoff within "
                    f"{c.config.mission_timeout_s:.0f}s; phase={phase}"
                )
            recovery_deadline = (
                time.monotonic() + c.config.return_recovery_timeout_s
            )
            while time.monotonic() < recovery_deadline:
                if c.worker_error is not None:
                    raise RuntimeError(f"Controller failed: {c.worker_error}")
                if c.handoff_ready_event.wait(timeout=0.05):
                    break
                if not worker.is_alive():
                    phase = c.phase_snapshot()
                    raise RuntimeError(
                        "Controller stopped during timeout recovery; "
                        f"phase={phase}"
                    )
            else:
                raise TimeoutError(
                    "Safe return did not reach handoff within "
                    f"{c.config.return_recovery_timeout_s:.0f}s"
                )

        print("[runner] Stable over home; handing descent to PX4 LAND...")
        _, _, land_sequence = landing_snapshot(c)
        c.prepare_native_land_handoff()
        ensure_mode(c, "LAND", MODE_TIMEOUT, keep_streaming=False)
        c.begin_native_land_handoff()
        worker.join(timeout=2.0)
        if worker.is_alive():
            raise RuntimeError("Offboard controller did not stop for LAND handoff")
        print("[runner] PX4 LAND confirmed; waiting for touchdown + auto-disarm...")
        wait_for_native_landing(
            c,
            after_sequence=land_sequence,
            timeout=c.config.land_timeout_s,
        )
        print("[runner] PX4 ON_GROUND and automatic disarm confirmed")
        outcome = MissionOutcome.SUCCESS
        outcome_reason = "PX4 ON_GROUND and automatic disarm confirmed"

        c.stop()
        controller_stopped = True
        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        landed_state, _, _ = landing_snapshot(c)
        phase = c.phase_snapshot()
        print(
            f"[runner] Final state main_mode={main_mode} sub_mode={sub_mode} "
            f"armed={armed} landed_state={landed_state} phase={phase}"
        )
        print("[runner] SUCCESS")

    except BaseException as exc:
        outcome_reason = f"{type(exc).__name__}: {exc}"
        outcome = classify_failure(c, vehicle_was_armed)
        raise

    finally:
        # Never force-disarm a vehicle that PX4 may still consider airborne.
        try:
            main_mode, sub_mode, armed = heartbeat_snapshot(c)
            if armed:
                phase = c.phase_snapshot()
                if c.failure_action_snapshot() == FailureAction.PX4_FAILSAFE:
                    print(
                        "[runner] Failure cleanup: stopping Offboard "
                        "commands and yielding to PX4 failsafe "
                        f"(phase={phase}, main_mode={main_mode}, "
                        f"sub_mode={sub_mode})"
                    )
                    _, _, land_sequence = landing_snapshot(c)
                    wait_for_failsafe_landing(
                        c,
                        after_sequence=land_sequence,
                        landing_timeout=c.config.land_timeout_s,
                    )
                    cleanup_status = (
                        "px4_failsafe_land_and_disarm_confirmed"
                    )
                else:
                    print(
                        "[runner] Failure cleanup: handing control to PX4 LAND "
                        f"(phase={phase}, main_mode={main_mode}, "
                        f"sub_mode={sub_mode})"
                    )
                    _, _, land_sequence = landing_snapshot(c)
                    c.prepare_native_land_handoff()
                    ensure_mode(c, "LAND", MODE_TIMEOUT, keep_streaming=False)
                    c.begin_native_land_handoff()
                    wait_for_native_landing(
                        c,
                        after_sequence=land_sequence,
                        timeout=c.config.land_timeout_s,
                    )
                    cleanup_status = "px4_land_and_disarm_confirmed"
        except Exception as exc:
            cleanup_status = "failed"
            cleanup_error = repr(exc)
            print(f"[runner] Failure LAND cleanup error: {exc!r}")
        finally:
            try:
                if c.master is not None and not controller_stopped:
                    c.stop()
            except Exception as exc:
                if cleanup_error is None:
                    cleanup_status = "failed"
                    cleanup_error = repr(exc)
                print(f"[runner] Controller cleanup error: {exc!r}")
            try:
                run_record.finalize(
                    outcome=outcome,
                    reason=outcome_reason,
                    final_state=final_state_snapshot(c),
                    cleanup_status=cleanup_status,
                    cleanup_error=cleanup_error,
                )
                print(f"[runner] Run manifest: {run_record.path}")
            except Exception as exc:
                print(f"[runner] Run manifest error: {exc!r}")


if __name__ == "__main__":
    main()
