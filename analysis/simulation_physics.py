"""Independent Gazebo ground-truth kinematic checks for the TRAJECTORY phase.

These are measured simulator dynamics, not planner derivatives or physical
flight qualification. Corrections by the feedback controller can exceed
reference acceleration/jerk; these are reported, not mislabeled as trajectory
certificate violations. Vehicle speed and tilt bounds are checked explicitly.
"""
from __future__ import annotations

import numpy as np

from trajectory_quality import distribution


def quaternion_tilt_deg(quaternion):
    """Tilt of the simulated body Z axis from vertical, independent of yaw."""
    q = np.asarray(quaternion, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError("ground-truth quaternion must have four components")
    lengths = np.linalg.norm(q, axis=1)
    valid = np.isfinite(q).all(axis=1) & np.isfinite(lengths) & (lengths > 1e-10)
    normal = np.full_like(q, np.nan, dtype=float)
    np.divide(q, lengths[:, None], out=normal, where=valid[:, None])
    vertical = np.clip(1.0 - 2.0 * (normal[:, 1] ** 2 + normal[:, 2] ** 2), -1., 1.)
    return np.degrees(np.arccos(vertical))


def assess_simulation_physics(phase, velocity, acceleration, tilt_deg, *,
                              maximum_speed_m_s, maximum_tilt_deg):
    """Independently audit Gazebo's measured physical motion in the route.

    The simulator's actual speed/tilt limits are gates. Acceleration is
    descriptive: the controller needs extra acceleration for tracking error,
    so planned acceleration and measured acceleration are distinct quantities.
    A measured jerk bound is intentionally not claimed from noisy samples.
    """
    phases = np.asarray(phase)
    velocity = np.asarray(velocity, dtype=float)
    acceleration = np.asarray(acceleration, dtype=float)
    tilt = np.asarray(tilt_deg, dtype=float)
    count = len(phases)
    if (velocity.shape != (count, 3) or acceleration.shape != (count, 3)
            or tilt.shape != (count,)):
        raise ValueError("ground-truth velocity/acceleration/tilt dimensions differ")
    if (not np.isfinite([maximum_speed_m_s, maximum_tilt_deg]).all()
            or maximum_speed_m_s <= 0 or not 0 < maximum_tilt_deg < 90):
        raise ValueError("a finite positive simulator envelope is required")
    route = phases == "TRAJECTORY"
    speed = np.linalg.norm(velocity[route], axis=1)
    accel = np.linalg.norm(acceleration[route], axis=1)
    tilt_route = tilt[route]
    valid = (len(speed) >= 2 and np.isfinite(speed).all()
             and np.isfinite(accel).all() and np.isfinite(tilt_route).all())
    speed_ok = bool(valid and np.max(speed) <= maximum_speed_m_s + 1e-6)
    tilt_ok = bool(valid and np.max(tilt_route) <= maximum_tilt_deg + 1e-6)
    checks = [
        {"name": "Independent Gazebo velocity/attitude/acceleration covers every trajectory sample",
         "passed": bool(valid),
         "detail": {"route_samples": int(route.sum()), "all_finite": bool(valid)}},
        {"name": "Simulated physical trajectory speed stays within vehicle hard speed limit",
         "passed": speed_ok,
         "detail": {"maximum_speed_m_s": maximum_speed_m_s,
                    "measured_speed_m_s": distribution(speed)}},
        {"name": "Simulated physical trajectory tilt stays within vehicle hard tilt limit",
         "passed": tilt_ok,
         "detail": {"maximum_tilt_deg": maximum_tilt_deg,
                    "measured_tilt_deg": distribution(tilt_route)}},
    ]
    return {
        "checks": checks,
        "observations": {
            "simulator_truth_speed_m_s": distribution(speed),
            "simulator_truth_acceleration_m_s2": distribution(accel),
            "simulator_truth_tilt_deg": distribution(tilt_route),
        },
        "scope": "Gazebo ground-truth trajectory-phase samples only; "
                 "no physical flight authority, between-sample guarantee or observed jerk certificate",
    }
