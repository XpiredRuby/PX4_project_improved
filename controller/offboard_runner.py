#!/usr/bin/env python3
import threading
import time

from pymavlink import mavutil

from PID_position_new import PositionController


PRESTREAM_SECONDS = 2.0
MODE_TIMEOUT = 8.0
ARM_TIMEOUT = 8.0

PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_AUTO_SUB_MODE_LAND = 6


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


def set_int_param_before_receiver(master, name, value, timeout=5.0):
    encoded_name = name.encode("ascii")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            encoded_name,
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        window_end = min(deadline, time.monotonic() + 1.0)
        while time.monotonic() < window_end:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if msg is None:
                continue
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", errors="ignore")
            param_id = str(param_id).rstrip("\x00")
            if param_id == name:
                actual = int(round(float(msg.param_value)))
                if actual != int(value):
                    raise RuntimeError(
                        f"PX4 parameter {name} acknowledged as {actual}, "
                        f"expected {value}"
                    )
                print(f"[runner] PX4 parameter {name}={actual} confirmed")
                return
    raise RuntimeError(f"Timed out setting PX4 parameter {name}={value}")


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
    ground_confirmed = False

    while time.monotonic() < deadline:
        main_mode, sub_mode, armed = heartbeat_snapshot(controller)
        landed_state, age, sequence = landing_snapshot(controller)
        if (
            sequence > after_sequence
            and age <= 1.5
            and landed_state == on_ground
        ):
            ground_confirmed = True
        if armed is False:
            if not ground_confirmed:
                raise RuntimeError(
                    "Vehicle disarmed without a fresh PX4 ON_GROUND report"
                )
            return
        if (
            main_mode != PX4_MAIN_MODE_AUTO
            or sub_mode != PX4_AUTO_SUB_MODE_LAND
        ):
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


def main():
    c = PositionController()
    worker = None
    controller_stopped = False

    try:
        print("[runner] Connecting to PX4...")
        c.connect()

        # This dedicated SITL workflow does not depend on QGC. PX4's separate
        # Offboard-loss and navigation failsafes remain enabled.
        set_int_param_before_receiver(c.master, "NAV_DLL_ACT", 0)
        set_int_param_before_receiver(c.master, "SDLOG_MODE", 0)

        c.start_receiver()
        c.wait_for_fresh_telemetry()
        c.wait_for_preflight_ready()
        wait_for_initial_ground_state(c)
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
            if c.phase == "HANDOFF":
                break
            if not worker.is_alive():
                raise RuntimeError(
                    f"Controller stopped before handoff; phase={c.phase}"
                )
            time.sleep(0.05)
        else:
            raise TimeoutError(
                f"Mission did not reach handoff within "
                f"{c.config.mission_timeout_s:.0f}s; phase={c.phase}"
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
        print(
            f"[runner] Final state main_mode={main_mode} sub_mode={sub_mode} "
            f"armed={armed} landed_state={landed_state} phase={c.phase}"
        )
        print("[runner] SUCCESS")

    finally:
        # Never force-disarm a vehicle that PX4 may still consider airborne.
        try:
            main_mode, sub_mode, armed = heartbeat_snapshot(c)
            if armed:
                print(
                    "[runner] Failure cleanup: handing control to PX4 LAND "
                    f"(phase={c.phase}, main_mode={main_mode}, "
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
