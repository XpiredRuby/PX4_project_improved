#!/usr/bin/env python3
import math
import threading
import time

from pymavlink import mavutil

from mission_state import FailureAction
from PID_position_new import PositionController


PRESTREAM_SECONDS = 2.0
MODE_TIMEOUT = 8.0
ARM_TIMEOUT = 8.0

PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_AUTO_SUB_MODE_LAND = 6

# PX4 v1.17 EKF2 fusion controls. Read these before the receiver thread owns
# recv_match; never alter persistent PX4 parameters from a mission run.
EKF2_SOURCE_PARAMETERS = (
    "EKF2_GPS_CTRL",
    "EKF2_HGT_REF",
    "EKF2_BARO_CTRL",
    "EKF2_MAG_TYPE",
    "EKF2_OF_CTRL",
    "EKF2_EV_CTRL",
    "EKF2_RNG_CTRL",
    "EKF2_AGP_CTRL",
)
PX4_SAFETY_PARAMETERS = (
    "COM_OF_LOSS_T",
    "COM_OBL_RC_ACT",
    "COM_DISARM_LAND",
)


def check_gps_imu_estimator_config(parameters):
    """Reject known EKF2 configurations that use other navigation sensors."""
    missing = sorted(set(EKF2_SOURCE_PARAMETERS) - parameters.keys())
    if missing:
        raise RuntimeError(
            "Cannot verify PX4 GPS/IMU fusion configuration: "
            + ", ".join(missing)
        )

    values = {}
    for name in EKF2_SOURCE_PARAMETERS:
        raw = float(parameters[name])
        if not math.isfinite(raw) or not raw.is_integer():
            raise RuntimeError(f"Invalid PX4 parameter {name}={raw}")
        values[name] = int(raw)

    required = {
        "EKF2_HGT_REF": 1,  # GPS height reference
        "EKF2_BARO_CTRL": 0,
        "EKF2_MAG_TYPE": 5,  # no magnetic initialization or fusion
        "EKF2_OF_CTRL": 0,
        "EKF2_EV_CTRL": 0,
        "EKF2_RNG_CTRL": 0,
        "EKF2_AGP_CTRL": 0,
    }
    mismatch = [
        f"{name}={values[name]} (expected {expected})"
        for name, expected in required.items()
        if values[name] != expected
    ]
    # Bits 0-2 enable GNSS horizontal position, altitude, and 3D velocity;
    # bit 3 optionally enables dual-antenna GNSS heading.
    if values["EKF2_GPS_CTRL"] not in (7, 15):
        mismatch.append(
            f"EKF2_GPS_CTRL={values['EKF2_GPS_CTRL']} (expected 7 or 15)"
        )
    if mismatch:
        raise RuntimeError(
            "PX4 estimator does not match GPS/IMU mission: "
            + "; ".join(mismatch)
            + ". Configure PX4 while disarmed, reboot, then retry."
        )


def check_px4_safety_config(
    parameters,
    max_offboard_loss_s=1.0,
    landing_timeout_s=120.0,
):
    """Verify that command loss lands and confirmed landing auto-disarms."""
    missing = sorted(set(PX4_SAFETY_PARAMETERS) - parameters.keys())
    if missing:
        raise RuntimeError(
            "Cannot verify PX4 failsafe configuration: " + ", ".join(missing)
        )
    offboard_loss_s = float(parameters["COM_OF_LOSS_T"])
    auto_disarm_s = float(parameters["COM_DISARM_LAND"])
    action_raw = float(parameters["COM_OBL_RC_ACT"])
    if not all(
        math.isfinite(value)
        for value in (offboard_loss_s, auto_disarm_s, action_raw)
    ):
        raise RuntimeError("PX4 failsafe parameters must be finite")
    if not action_raw.is_integer():
        raise RuntimeError(f"Invalid PX4 parameter COM_OBL_RC_ACT={action_raw}")

    mismatch = []
    if not 0.0 <= offboard_loss_s <= max_offboard_loss_s:
        mismatch.append(
            f"COM_OF_LOSS_T={offboard_loss_s:g}s "
            f"(expected 0..{max_offboard_loss_s:g}s)"
        )
    if int(action_raw) != 4:
        mismatch.append(
            f"COM_OBL_RC_ACT={int(action_raw)} (expected 4/Land)"
        )
    if not 0.0 < auto_disarm_s < landing_timeout_s:
        mismatch.append(
            f"COM_DISARM_LAND={auto_disarm_s:g}s "
            f"(expected >0 and <{landing_timeout_s:g}s)"
        )
    if mismatch:
        raise RuntimeError(
            "PX4 failsafe does not match mission assumptions: "
            + "; ".join(mismatch)
            + ". Configure PX4 while disarmed, then retry."
        )


def read_px4_parameters(controller, names, attempts=3, timeout_s=0.7):
    """Read named parameters before the telemetry receiver owns recv_match."""
    master = controller.master
    parameters = {}
    for name in names:
        for _ in range(attempts):
            with controller.mav_send_lock:
                master.mav.param_request_read_send(
                    master.target_system,
                    master.target_component,
                    name.encode("ascii"),
                    -1,
                )
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                msg = master.recv_match(
                    type="PARAM_VALUE",
                    blocking=True,
                    timeout=min(0.2, max(0.0, deadline - time.monotonic())),
                )
                if (
                    msg is None
                    or msg.get_srcSystem() != master.target_system
                    or msg.get_srcComponent() != master.target_component
                ):
                    continue
                param_id = msg.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode("ascii", errors="replace")
                if param_id.rstrip("\x00") == name:
                    parameters[name] = msg.param_value
                    break
            if name in parameters:
                break

    return parameters


def audit_px4_configuration(controller, attempts=3, timeout_s=0.7):
    """Read and verify sensor-fusion and failure-response configuration."""
    names = EKF2_SOURCE_PARAMETERS + PX4_SAFETY_PARAMETERS
    parameters = read_px4_parameters(
        controller,
        names,
        attempts=attempts,
        timeout_s=timeout_s,
    )

    check_gps_imu_estimator_config(parameters)
    check_px4_safety_config(
        parameters,
        max_offboard_loss_s=controller.config.max_offboard_loss_timeout_s,
        landing_timeout_s=controller.config.land_timeout_s,
    )
    print("[runner] PX4 GPS/IMU and failure-response settings verified")


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


def stream_for(controller, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        send_neutral(controller)
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


def ensure_mode(controller, mode_name, timeout, keep_streaming):
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
            send_neutral(controller)
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


def wait_for_armed(controller, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        send_neutral(controller)
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


def wait_for_native_landing(controller, after_sequence, timeout):
    deadline = time.monotonic() + timeout
    on_ground = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

    while time.monotonic() < deadline:
        controller.log_native_landing_sample()
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
            return
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
    raise TimeoutError(
        f"PX4 LAND did not finish within {timeout:.0f}s "
        f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed}, "
        f"landed_state={landed_state}, landed_age={age:.2f}s)"
    )


def wait_for_failsafe_takeover(controller, timeout=MODE_TIMEOUT):
    controller.begin_failsafe_handoff()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        controller.log_native_landing_sample(phase="PX4_FAILSAFE")
        main_mode, _, armed = heartbeat_snapshot(controller)
        if armed is False or main_mode != PX4_MAIN_MODE_OFFBOARD:
            return
        time.sleep(0.05)
    raise TimeoutError("PX4 did not take control after Offboard commands stopped")


def main():
    c = PositionController()
    worker = None
    controller_stopped = False

    try:
        print("[runner] Connecting to PX4...")
        c.connect()
        audit_px4_configuration(c)

        c.start_receiver()
        c.wait_for_fresh_telemetry()
        c.wait_for_preflight_ready()
        wait_for_initial_ground_state(c)
        home_snapshot = c.capture_home_reference()
        c.configure_cruise_height(home_snapshot)
        c.validate_mission_plan()
        c.setup_logger()

        print(
            f"[runner] Pre-streaming neutral setpoints for "
            f"{PRESTREAM_SECONDS:.1f}s..."
        )
        stream_for(c, PRESTREAM_SECONDS)

        print("[runner] Requesting OFFBOARD...")
        ensure_mode(c, "OFFBOARD", MODE_TIMEOUT, keep_streaming=True)
        print("[runner] OFFBOARD confirmed")

        print("[runner] Requesting arm...")
        ack_sequence = request_arm(c)
        wait_for_command_ack(
            c,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            ack_sequence,
            ARM_TIMEOUT,
        )
        wait_for_armed(c, ARM_TIMEOUT)
        print("[runner] Armed confirmed")

        c.initialize_target()
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
                    wait_for_failsafe_takeover(c)
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
        except Exception as exc:
            print(f"[runner] Failure LAND cleanup error: {exc!r}")
        finally:
            try:
                if c.master is not None and not controller_stopped:
                    c.stop()
            except Exception as exc:
                print(f"[runner] Controller cleanup error: {exc!r}")


if __name__ == "__main__":
    main()
