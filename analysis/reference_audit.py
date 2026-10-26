"""Data-only cross-check of exported NED references; no planner imports.

This is a sampled diagnostic, NOT a continuous certificate or flight approval.
The quadrature cross-check uses endpoint velocity/acceleration/jerk to integrate
velocity via quintic Hermite quadrature. It is exact for velocity degree <= 5;
for higher degree it is a resolution-sensitive consistency diagnostic.
"""
from __future__ import annotations

import csv
import hashlib
import io
import math

import numpy as np


def audit_reference(csv_text, vehicle, environment):
    reader = csv.DictReader(io.StringIO(csv_text))
    fields = ("time", "x", "y", "z", "yaw", "vx", "vy", "vz", "yaw_rate",
              "ax", "ay", "az", "yaw_acceleration", "jx", "jy", "jz", "yaw_jerk")
    if not set(fields).issubset(reader.fieldnames or []):
        raise ValueError("Reference CSV is missing state fields")
    rows = np.array([[float(row[k]) for k in fields] for row in reader])
    if rows.ndim != 2 or len(rows) < 2 or not np.isfinite(rows).all():
        raise ValueError("Reference requires at least two finite states")
    dt = np.diff(rows[:, 0])
    if rows[0, 0] != 0 or np.any(dt <= 0) or np.max(dt) > 0.05000001:
        raise ValueError("Reference clock must start at zero and increase by <= 50 ms")
    p, v, a, j = [rows[:, start:start + 4] for start in (1, 5, 9, 13)]
    h = dt[:, None]
    integral = (h * (v[:-1] + v[1:]) / 2
                + h**2 * (a[:-1] - a[1:]) / 10
                + h**3 * (j[:-1] + j[1:]) / 120)
    residual = np.abs(np.diff(p, axis=0) - integral)
    # A separate cubic-Hermite integral of acceleration detects constant
    # acceleration bias, which cancels in the position quadrature above.
    velocity_residual = np.abs(np.diff(v, axis=0) - (
        h * (a[:-1] + a[1:]) / 2 + h**2 * (j[:-1] - j[1:]) / 12))
    speed = np.linalg.norm(v[:, :3], axis=1)
    accel = np.linalg.norm(a[:, :3], axis=1)
    jerk = np.linalg.norm(j[:, :3], axis=1)
    # NED force: mass * (acceleration - [0, 0, g]); upward thrust magnitude.
    force = a[:, :3].copy()
    force[:, 2] -= 9.80665
    thrust = float(vehicle["mass_kg"]) * np.linalg.norm(force, axis=1)
    tilt = np.arctan2(np.linalg.norm(force[:, :2], axis=1), -force[:, 2])
    metrics = dict(speed_m_s=float(max(speed)), acceleration_m_s2=float(max(accel)),
                   jerk_m_s3=float(max(jerk)), thrust_max_n=float(max(thrust)),
                   thrust_min_n=float(min(thrust)), tilt_deg=float(np.degrees(max(tilt))),
                   position_integral_residual_m=float(residual[:, :3].max()),
                   yaw_integral_residual_rad=float(residual[:, 3].max()),
                   velocity_integral_residual_m_s=float(velocity_residual[:, :3].max()),
                   yaw_rate_integral_residual_rad_s=float(velocity_residual[:, 3].max()))
    checks = {}
    for key, data, limit in (
        ("speed", speed, vehicle["maximum_speed_m_s"]),
        ("acceleration", accel, vehicle["maximum_acceleration_m_s2"]),
        ("jerk", jerk, vehicle["maximum_jerk_m_s3"]),
        ("yaw_rate", np.abs(v[:, 3]), math.radians(vehicle["maximum_yaw_rate_deg_s"])),
        ("yaw_acceleration", np.abs(a[:, 3]), math.radians(vehicle["maximum_yaw_acceleration_deg_s2"])),
        ("yaw_jerk", np.abs(j[:, 3]), math.radians(vehicle["maximum_yaw_jerk_deg_s3"])),
        ("thrust_max", thrust, vehicle["maximum_thrust_n"]),
        ("tilt", tilt, math.radians(vehicle["maximum_tilt_deg"])),
        ("radius", np.linalg.norm(p[:, :2], axis=1), environment["max_radius_m"]),
        ("height", -p[:, 2], environment["max_height_above_launch_m"]),
    ):
        checks[key] = bool(np.max(data) <= float(limit) + 1e-8)
    checks["thrust_min"] = bool(min(thrust) >= vehicle["minimum_thrust_n"] - 1e-8)
    checks["above_launch"] = bool(max(p[:, 2]) <= 1e-8)
    # Explicit diagnostic tolerance; not a C3 proof. Keep metric in report.
    checks["position_integral"] = bool(residual[:, :3].max() <= 1e-6)
    checks["yaw_integral"] = bool(residual[:, 3].max() <= 1e-6)
    # Necessary consistency condition for any jerk-bounded acceleration:
    # trapezoid integral error <= J*h^2/4 (integrate the Lipschitz tent).
    # Unlike the cubic diagnostic, this requires no unprovided snap bound.
    trap_error = np.diff(v, axis=0) - h * (a[:-1] + a[1:]) / 2
    checks["velocity_integral"] = bool(np.all(
        np.linalg.norm(trap_error[:, :3], axis=1)
        <= vehicle["maximum_jerk_m_s3"] * dt**2 / 4 + 1e-9))
    checks["yaw_rate_integral"] = bool(np.all(
        np.abs(trap_error[:, 3])
        <= math.radians(vehicle["maximum_yaw_jerk_deg_s3"]) * dt**2 / 4 + 1e-9))
    distance = float(np.linalg.norm(np.diff(p[:, :3], axis=0), axis=1).sum())
    duration = float(rows[-1, 0])
    stationary = (speed[:-1] < 1e-4) & (speed[1:] < 1e-4)
    turning = stationary & ((np.abs(v[:-1, 3]) + np.abs(v[1:, 3])) > 1e-4)
    return dict(
        passed=all(checks.values()), checks=checks, metrics=metrics,
        sha256=hashlib.sha256(csv_text.encode()).hexdigest(), samples=len(rows),
        duration_s=duration, sampled_path_length_m=distance,
        average_speed_m_s=distance / duration,
        sampled_stationary_time_s=float(dt[stationary].sum()),
        sampled_stationary_turn_time_s=float(dt[turning].sum()),
        speed_only_lower_bound_s=distance / vehicle["maximum_speed_m_s"],
        scope="sampled reference diagnostic; no continuous or control-authority certificate",
        model_identified=vehicle.get("identified_for_physical_flight") is True,
        simulator_ran=False,
    )


def main():
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('--vehicle', required=True, type=Path)
    parser.add_argument('--environment', required=True, type=Path)
    args = parser.parse_args()
    try:
        result = audit_reference(args.trajectory.read_text(),
                                 json.loads(args.vehicle.read_text()),
                                 json.loads(args.environment.read_text()))
    except (ValueError, KeyError, TypeError, OSError) as error:
        result = dict(passed=False, error=str(error), simulator_ran=False)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
