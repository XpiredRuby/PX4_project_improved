#!/usr/bin/env python3
"""Inject a bounded GPS fault into PX4 SITL and always restore safe settings."""

import argparse
import math
import struct
import time

from pymavlink import mavutil


FAILURE_TYPES = ("off",)
SIM_GPS_USED = "SIM_GPS_USED"

INTEGER_FORMATS = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8: ">xxxB",
    mavutil.mavlink.MAV_PARAM_TYPE_INT8: ">xxxb",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16: ">xxH",
    mavutil.mavlink.MAV_PARAM_TYPE_INT16: ">xxh",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32: ">I",
    mavutil.mavlink.MAV_PARAM_TYPE_INT32: ">i",
}


def validate_local_connection(value):
    """Limit this destructive test helper to a local UDP receive endpoint."""
    parts = value.split(":")
    if len(parts) != 3 or parts[0] != "udpin":
        raise argparse.ArgumentTypeError(
            "connection must use udpin:<local-host>:<port>"
        )
    host = parts[1].lower()
    if host not in {"0.0.0.0", "127.0.0.1", "localhost"}:
        raise argparse.ArgumentTypeError(
            "failure injection is restricted to a local endpoint"
        )
    try:
        port = int(parts[2])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("UDP port must be an integer") from exc
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("UDP port is outside 1..65535")
    return value


def clean_parameter_id(value):
    if isinstance(value, bytes):
        return value.decode("ascii", errors="ignore").rstrip("\x00")
    return str(value).rstrip("\x00")


def encode_parameter_value(value, parameter_type):
    if parameter_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)
    packed = struct.pack(INTEGER_FORMATS[parameter_type], int(value))
    return struct.unpack(">f", packed)[0]


def decode_parameter_value(raw_value, parameter_type):
    if parameter_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(raw_value)
    packed = struct.pack(">f", float(raw_value))
    return float(struct.unpack(INTEGER_FORMATS[parameter_type], packed)[0])


def wait_for_parameter(master, name, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = master.recv_match(
            type="PARAM_VALUE",
            blocking=True,
            timeout=min(0.5, max(0.0, deadline - time.monotonic())),
        )
        if message is None:
            continue
        if (
            message.get_srcSystem() != master.target_system
            or clean_parameter_id(message.param_id) != name
        ):
            continue
        return (
            decode_parameter_value(
                message.param_value,
                message.param_type,
            ),
            message.param_type,
        )
    raise TimeoutError(f"No PARAM_VALUE received for {name}")


def read_parameter(master, name, timeout_s=3.0):
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        -1,
    )
    return wait_for_parameter(master, name, timeout_s)


def set_parameter(master, name, desired, parameter_type, attempts=4):
    encoded = encode_parameter_value(desired, parameter_type)
    confirmed = None
    for _ in range(attempts):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode("ascii"),
            encoded,
            parameter_type,
        )
        try:
            confirmed, _ = wait_for_parameter(master, name)
        except TimeoutError:
            confirmed, _ = read_parameter(master, name)
        if math.isclose(
            confirmed,
            float(desired),
            rel_tol=0.0,
            abs_tol=1e-3,
        ):
            return confirmed
        time.sleep(0.2)
    raise RuntimeError(
        f"{name}: requested {desired}, PX4 reports {confirmed}"
    )


class AirborneGate:
    """Require arming plus a measured climb from the disarmed baseline."""

    def __init__(self, minimum_height_m):
        if minimum_height_m <= 0.0:
            raise ValueError("minimum_height_m must be positive")
        self.minimum_height_m = float(minimum_height_m)
        self.baseline_z = None
        self.armed = False

    def observe_heartbeat(self, base_mode):
        self.armed = bool(
            int(base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

    def observe_position(self, z):
        z = float(z)
        if not math.isfinite(z):
            return False
        if not self.armed:
            self.baseline_z = (
                z
                if self.baseline_z is None
                else 0.95 * self.baseline_z + 0.05 * z
            )
            return False
        return (
            self.baseline_z is not None
            and z <= self.baseline_z - self.minimum_height_m
        )


def wait_until_airborne(master, minimum_height_m, timeout_s):
    gate = AirborneGate(minimum_height_m)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = master.recv_match(
            type=["HEARTBEAT", "LOCAL_POSITION_NED"],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == "HEARTBEAT":
            gate.observe_heartbeat(message.base_mode)
        elif (
            message_type == "LOCAL_POSITION_NED"
            and gate.observe_position(message.z)
        ):
            print(
                "[injector] Airborne gate passed "
                f"(baseline_z={gate.baseline_z:.2f}m, z={message.z:.2f}m)",
                flush=True,
            )
            return
    raise TimeoutError(
        "Vehicle did not arm and climb "
        f"{minimum_height_m:.1f}m within {timeout_s:.0f}s"
    )


def run_fault(master, failure_name, duration_s):
    if failure_name != "off":
        raise ValueError(f"Unsupported Gazebo GPS fault: {failure_name}")
    before, parameter_type = read_parameter(master, SIM_GPS_USED)
    if int(before) < 4:
        raise RuntimeError(
            f"{SIM_GPS_USED} was already unhealthy: {before}"
        )
    try:
        set_parameter(master, SIM_GPS_USED, 0, parameter_type)
        print(
            f"[injector] Gazebo GPS outage injected for "
            f"{duration_s:.2f}s",
            flush=True,
        )
        time.sleep(duration_s)
    finally:
        set_parameter(master, SIM_GPS_USED, int(before), parameter_type)
        print(
            f"[injector] {SIM_GPS_USED} restored to {int(before)}",
            flush=True,
        )


def build_parser():
    parser = argparse.ArgumentParser(
        description="Inject one bounded GPS fault into PX4 SITL."
    )
    parser.add_argument(
        "--connection",
        type=validate_local_connection,
        default="udpin:0.0.0.0:14550",
    )
    parser.add_argument(
        "--failure-type",
        choices=tuple(FAILURE_TYPES),
        required=True,
    )
    parser.add_argument("--duration-s", type=float, required=True)
    parser.add_argument("--airborne-height-m", type=float, default=8.0)
    parser.add_argument("--wait-timeout-s", type=float, default=150.0)
    parser.add_argument(
        "--confirm-sitl",
        action="store_true",
        help="Required acknowledgement that the target is PX4 SITL.",
    )
    return parser


def main():
    args = build_parser().parse_args()
    if not args.confirm_sitl:
        raise SystemExit("--confirm-sitl is required")
    if args.duration_s <= 0.0 or args.duration_s > 30.0:
        raise SystemExit("--duration-s must be in (0, 30]")

    master = mavutil.mavlink_connection(
        args.connection,
        source_system=253,
        source_component=190,
    )
    heartbeat = master.wait_heartbeat(timeout=15.0)
    if heartbeat is None:
        raise TimeoutError("PX4 heartbeat was not received")
    print(
        f"[injector] Connected to PX4 system={master.target_system} "
        f"component={master.target_component}",
        flush=True,
    )
    wait_until_airborne(
        master,
        minimum_height_m=args.airborne_height_m,
        timeout_s=args.wait_timeout_s,
    )
    run_fault(master, args.failure_type, args.duration_s)


if __name__ == "__main__":
    main()
