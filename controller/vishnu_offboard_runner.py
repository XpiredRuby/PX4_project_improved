#!/usr/bin/env python3
import threading
import time
from pymavlink import mavutil
from PID_position_new import PositionController

PRESTREAM_SECONDS = 2.0
MODE_TIMEOUT = 8.0
ARM_TIMEOUT = 8.0
LAND_TIMEOUT = 35.0
MISSION_TIMEOUT = 180.0

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
                        f"PX4 parameter {name} acknowledged as {actual}, expected {value}"
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


def wait_heartbeat_state(controller, timeout, predicate, keep_streaming=True):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if keep_streaming:
            send_neutral(controller)
        main_mode, sub_mode, armed = heartbeat_snapshot(controller)
        if predicate(main_mode, sub_mode, armed):
            return True
        time.sleep(controller.control_dt)
    main_mode, sub_mode, armed = heartbeat_snapshot(controller)
    return bool(predicate(main_mode, sub_mode, armed))


def request_mode(controller, mode_name):
    master = controller.master
    mode_name = mode_name.upper()

    with controller.mav_send_lock:
        if mode_name == "OFFBOARD":
            custom_mode = PX4_MAIN_MODE_OFFBOARD << 16
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )
            return

        if mode_name == "LAND":
            custom_mode = (
                (PX4_AUTO_SUB_MODE_LAND << 24)
                | (PX4_MAIN_MODE_AUTO << 16)
            )
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )
            return

        master.set_mode(mode_name)


def request_arm(controller, arm=True):
    with controller.mav_send_lock:
        controller.master.mav.command_long_send(
            controller.master.target_system,
            controller.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0, 0, 0, 0, 0, 0,
        )


def main():
    c = PositionController()
    worker = None
    completed = False
    controller_stopped = False

    try:
        print("[runner] Connecting to PX4...")
        c.connect()

        # Make this dedicated SITL workflow independent of whether QGC is open.
        # Disable only the GCS data-link-loss action; PX4's distinct Offboard-
        # loss failsafe remains enabled.
        set_int_param_before_receiver(c.master, "NAV_DLL_ACT", 0)
        set_int_param_before_receiver(c.master, "SDLOG_MODE", 0)

        c.start_receiver()
        time.sleep(1.0)
        c.wait_for_fresh_telemetry()
        c.setup_logger()

        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        print(
            f"[runner] Initial heartbeat main_mode={main_mode} "
            f"sub_mode={sub_mode} armed={armed}"
        )

        print(f"[runner] Pre-streaming neutral setpoints for {PRESTREAM_SECONDS:.1f}s...")
        stream_for(c, PRESTREAM_SECONDS)

        print("[runner] Requesting OFFBOARD...")
        request_mode(c, "OFFBOARD")
        offboard = wait_heartbeat_state(
            c,
            MODE_TIMEOUT,
            predicate=lambda main, sub, armed: main == PX4_MAIN_MODE_OFFBOARD,
        )
        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        if not offboard:
            raise RuntimeError(
                f"PX4 did not enter OFFBOARD "
                f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
            )
        print(
            f"[runner] OFFBOARD confirmed "
            f"(main_mode={main_mode}, sub_mode={sub_mode})"
        )

        print("[runner] Requesting arm...")
        request_arm(c, True)
        armed_ok = wait_heartbeat_state(
            c,
            ARM_TIMEOUT,
            predicate=lambda main, sub, armed: armed is True,
        )
        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        if not armed_ok:
            raise RuntimeError(
                f"PX4 did not arm "
                f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
            )
        print("[runner] Armed confirmed")

        # Capture position/yaw after prestreaming, mode change, and arming so
        # takeoff and the trajectory share one current origin.
        c.initialize_target()

        print("[runner] Starting Vishnu position controller...")
        worker = threading.Thread(target=c.run, daemon=True)
        worker.start()

        deadline = time.monotonic() + MISSION_TIMEOUT
        while time.monotonic() < deadline and worker.is_alive():
            if c.phase == "DONE":
                completed = True
                print("[runner] Mission reached DONE")
                break
            time.sleep(0.1)

        if not completed:
            main_mode, sub_mode, armed = heartbeat_snapshot(c)
            print(
                f"[runner] Mission did not complete "
                f"(phase={c.phase}, main_mode={main_mode}, "
                f"sub_mode={sub_mode}, armed={armed})"
            )
            try:
                request_mode(c, "LAND")
                time.sleep(1.0)
            except Exception as exc:
                print(f"[runner] LAND request failed: {exc}")
            raise RuntimeError(f"Mission did not complete; final phase={c.phase}")

        # Vishnu's controller marks DONE just above the point where PX4's own
        # land detector may declare touchdown. Hand the final touchdown to PX4
        # AUTO/LAND and wait for PX4 to auto-disarm natively.
        print("[runner] Vishnu mission DONE; handing touchdown to PX4 LAND...")
        request_mode(c, "LAND")
        land_mode = wait_heartbeat_state(
            c,
            MODE_TIMEOUT,
            predicate=lambda main, sub, armed: (
                main == PX4_MAIN_MODE_AUTO and sub == PX4_AUTO_SUB_MODE_LAND
            ),
            keep_streaming=False,
        )
        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        if not land_mode:
            raise RuntimeError(
                f"PX4 did not enter LAND "
                f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
            )
        print("[runner] PX4 LAND confirmed; waiting for native landing + auto-disarm...")

        disarmed = wait_heartbeat_state(
            c,
            LAND_TIMEOUT,
            predicate=lambda main, sub, armed: armed is False,
            keep_streaming=False,
        )
        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        if not disarmed:
            raise RuntimeError(
                f"PX4 LAND did not auto-disarm within {LAND_TIMEOUT:.0f}s "
                f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
            )
        print("[runner] PX4 native landing/disarm confirmed")

        c.stop()
        controller_stopped = True
        if worker is not None:
            worker.join(timeout=3.0)

        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        print(
            f"[runner] Final state main_mode={main_mode} sub_mode={sub_mode} "
            f"armed={armed} phase={c.phase}"
        )
        print("[runner] SUCCESS")

    finally:
        # Any armed failure is handed to PX4 LAND. Never force-disarm a
        # vehicle that PX4 may still consider airborne.
        try:
            main_mode, sub_mode, armed = heartbeat_snapshot(c)
            if armed:
                print(
                    "[runner] Failure cleanup: requesting PX4 LAND "
                    f"(phase={c.phase}, main_mode={main_mode}, "
                    f"sub_mode={sub_mode})"
                )
                request_mode(c, "LAND")
                wait_heartbeat_state(
                    c,
                    LAND_TIMEOUT,
                    predicate=lambda main, sub, armed: armed is False,
                    keep_streaming=False,
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
