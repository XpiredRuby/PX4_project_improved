"""PX4 parameter contracts required by the mission runner."""

import math
import struct
import time

from pymavlink import mavutil


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
    "SYS_FAILURE_EN",
)
PX4_BOOTSTRAP_PARAMETERS = (
    "MPC_THR_HOVER",
    "MPC_THR_MAX",
)


_INTEGER_PARAM_FORMATS = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8: ">xxxB",
    mavutil.mavlink.MAV_PARAM_TYPE_INT8: ">xxxb",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16: ">xxH",
    mavutil.mavlink.MAV_PARAM_TYPE_INT16: ">xxh",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32: ">I",
    mavutil.mavlink.MAV_PARAM_TYPE_INT32: ">i",
}


def decode_px4_parameter_value(raw_value, param_type):
    """Decode MAVLink's byte-wise integer encoding used by PX4 parameters."""
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(raw_value)
    try:
        integer_format = _INTEGER_PARAM_FORMATS[param_type]
    except KeyError as exc:
        raise RuntimeError(
            f"Unsupported MAVLink parameter type {param_type}"
        ) from exc
    packed = struct.pack(">f", float(raw_value))
    return struct.unpack(integer_format, packed)[0]


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
    failure_injection_raw = float(parameters["SYS_FAILURE_EN"])
    if not all(
        math.isfinite(value)
        for value in (
            offboard_loss_s, auto_disarm_s, action_raw, failure_injection_raw
        )
    ):
        raise RuntimeError("PX4 failsafe parameters must be finite")
    if not action_raw.is_integer() or not failure_injection_raw.is_integer():
        raise RuntimeError("PX4 integer safety parameters are invalid")

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
    if int(failure_injection_raw) != 0:
        mismatch.append(
            f"SYS_FAILURE_EN={int(failure_injection_raw)} (expected 0)"
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
                    parameters[name] = decode_px4_parameter_value(
                        msg.param_value,
                        msg.param_type,
                    )
                    break
            if name in parameters:
                break

    return parameters


def audit_px4_configuration(controller, attempts=3, timeout_s=0.7):
    """Read and verify sensor-fusion and failure-response configuration."""
    names = (
        EKF2_SOURCE_PARAMETERS
        + PX4_SAFETY_PARAMETERS
        + PX4_BOOTSTRAP_PARAMETERS
    )
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
    hover_thrust = float(parameters["MPC_THR_HOVER"])
    max_thrust = float(parameters["MPC_THR_MAX"])
    if not (
        math.isfinite(hover_thrust)
        and math.isfinite(max_thrust)
        and 0.0 < hover_thrust < max_thrust <= 1.0
    ):
        raise RuntimeError(
            "Invalid PX4 multicopter thrust parameters: "
            f"MPC_THR_HOVER={hover_thrust}, MPC_THR_MAX={max_thrust}"
        )
    print("[runner] PX4 GPS/IMU and failure-response settings verified")
    return parameters
