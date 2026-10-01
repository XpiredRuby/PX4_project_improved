#!/usr/bin/env python3
# ruff: noqa: E402
"""Independent observer for an explicitly confirmed local SITL process-loss test."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from sitl_fault_injector import read_parameter, wait_until_airborne


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner-pid-file", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--confirm-sitl", action="store_true")
    args = parser.parse_args()
    if not args.confirm_sitl:
        parser.error("--confirm-sitl is required")

    master = mavutil.mavlink_connection(
        "udpin:0.0.0.0:14550", source_system=253, source_component=191,
    )
    heartbeat = master.wait_heartbeat(timeout=15)
    if heartbeat is None:
        raise TimeoutError("No PX4 heartbeat")
    if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
        raise RuntimeError("Target heartbeat is not from the PX4 autopilot")
    master.target_component = heartbeat.get_srcComponent()
    settings = {}
    for name in ("SIM_GPS_USED", "COM_OF_LOSS_T", "COM_OBL_RC_ACT"):
        settings[name], _ = read_parameter(master, name)
    if not settings["SIM_GPS_USED"] >= 4:
        raise RuntimeError("SITL GPS is not healthy")
    if not 0 < settings["COM_OF_LOSS_T"] <= 2:
        raise RuntimeError("Unexpected Offboard-loss delay")
    if settings["COM_OBL_RC_ACT"] != 4:
        raise RuntimeError("Offboard-loss action must be LAND")

    for message_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    ):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, 50000, 0, 0, 0, 0, 0,
        )
    wait_until_airborne(master, 8.0, 150.0)
    pid = int(args.runner_pid_file.read_text().strip())
    expected_script = (ROOT / "controller/offboard_runner.py").resolve()
    command = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    if str(expected_script).encode() not in command:
        raise RuntimeError("PID does not belong to the specified SITL runner")
    os.kill(pid, signal.SIGKILL)
    killed_at = time.monotonic()
    print(f"Stopped task-owned runner PID {pid} after the 8m climb", flush=True)

    state = {}
    events = []
    first_land_s = None
    first_ground_s = None
    first_disarm_s = None
    stable_since = None
    bounced = False
    while time.monotonic() - killed_at < 150:
        message = master.recv_match(blocking=True, timeout=0.2)
        now = time.monotonic()
        elapsed = now - killed_at
        if message is None:
            continue
        if (
            message.get_srcSystem() != master.target_system
            or message.get_srcComponent() != master.target_component
        ):
            continue
        kind = message.get_type()
        if kind == "HEARTBEAT":
            state.update(
                armed=bool(message.base_mode & 128), heartbeat_at=now,
                main_mode=(message.custom_mode >> 16) & 255,
                sub_mode=(message.custom_mode >> 24) & 255,
            )
            if state["main_mode"] == 4 and state["sub_mode"] == 6:
                if first_land_s is None:
                    first_land_s = elapsed
            if not state["armed"] and first_disarm_s is None:
                first_disarm_s = elapsed
        elif kind == "EXTENDED_SYS_STATE":
            state.update(landed=message.landed_state, landed_at=now)
            if message.landed_state == 1:
                if first_ground_s is None:
                    first_ground_s = elapsed
                    events.append({"touchdown": elapsed, **state})
            elif first_ground_s is not None:
                bounced = True
        elif kind == "ACTUATOR_OUTPUT_STATUS":
            state.update(
                motors=[float(x) for x in message.actuator[:4]],
                active=int(message.active), actuator_at=now,
            )
        elif kind == "LOCAL_POSITION_NED":
            state.update(
                x=message.x, y=message.y, z=message.z,
                vx=message.vx, vy=message.vy, vz=message.vz,
            )
        elif kind == "ATTITUDE":
            state.update(roll=message.roll, pitch=message.pitch)
        else:
            continue
        events.append({"elapsed_s": elapsed, "message": kind, **state})
        motors = state.get("motors", [])
        terminal = (
            first_land_s is not None and first_ground_s is not None
            and state.get("armed") is False and state.get("landed") == 1
            and now - state.get("heartbeat_at", -math.inf) < 2
            and now - state.get("landed_at", -math.inf) < 1
            and now - state.get("actuator_at", -math.inf) < 1
            # Installed PX4 v1.17 sends act.noutputs in this field, not a mask.
            and 4 <= state.get("active", 0) <= 32
            and len(motors) == 4
            and all(math.isfinite(x) and abs(x) <= 0.01 for x in motors)
        )
        stable_since = (now if stable_since is None else stable_since) if terminal else None
        if stable_since is not None and now - stable_since >= 1:
            break
    touchdown = next((item for item in events if "touchdown" in item), {})
    touchdown_values = [touchdown.get(key, math.inf) for key in ("vx", "vy", "vz", "roll", "pitch")]
    touchdown_bounded = (
        all(math.isfinite(value) for value in touchdown_values)
        and math.hypot(touchdown_values[0], touchdown_values[1]) <= 0.5
        and abs(touchdown_values[2]) <= 0.5
        and max(abs(touchdown_values[3]), abs(touchdown_values[4])) <= math.radians(10)
    )
    passed = (
        stable_since is not None and time.monotonic() - stable_since >= 1
        and first_land_s is not None and first_land_s <= 5
        and first_disarm_s is not None and first_ground_s is not None
        and 0 <= first_disarm_s - first_ground_s <= 5 and not bounced
        and touchdown_bounded
    )
    result = {
        "overall_passed": passed, "px4_parameters": settings,
        "land_mode_delay_s": first_land_s,
        "ground_delay_s": first_ground_s,
        "disarm_delay_s": first_disarm_s,
        "post_touchdown_bounce": bounced,
        "touchdown_dynamics_bounded": touchdown_bounded,
        "final_state": state, "events": events,
    }
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "events"}, indent=2))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
