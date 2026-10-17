"""Independent acceptance checks for baseline timed trajectories."""

from __future__ import annotations

from dataclasses import astuple
import math

import numpy as np

from .contracts import (
    EnvironmentSnapshot,
    MotionIntent,
    VehicleModel,
    VerificationReport,
)
from .geometry import GeometryPlan, measure_path_fidelity
from .retiming import CandidateTrajectory
from .timing import PathBounds, certified_dynamics, schedule_defects


def _check(
    requirement_id: str,
    passed: bool,
    detail: str,
    *,
    margin: float | None = None,
    units: str | None = None,
) -> dict:
    return {
        "requirement_id": requirement_id,
        "status": "PASS" if passed else "FAIL",
        "hard": True,
        "margin": margin,
        "units": units,
        "detail": detail,
    }


def _finite_candidate(candidate: CandidateTrajectory) -> bool:
    return bool(candidate.points) and all(
        all(
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
            for value in astuple(point)
        )
        for point in candidate.points
    )


def _trajectory_structure(
    candidate: CandidateTrajectory,
    maximum_sample_period_s: float,
) -> bool:
    if not _finite_candidate(candidate):
        return False
    points = candidate.points
    intervals = len(points) - 1
    return (
        candidate.schema_version == 1
        and candidate.release_class == "SITL_ONLY"
        and points[0].time_s == 0.0
        and points[0].path_parameter == 0.0
        and points[-1].time_s == candidate.summary.selected_duration_s
        and points[-1].path_parameter == 1.0
        and intervals >= 1
        and all(
            right.time_s > left.time_s
            and right.path_parameter >= left.path_parameter
            and right.time_s - left.time_s <= maximum_sample_period_s + 1e-12
            for left, right in zip(points, points[1:], strict=False)
        )
        and all(
            math.isclose(
                point.time_s,
                candidate.summary.selected_duration_s * index / intervals,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
            for index, point in enumerate(points)
        )
    )


def _trajectory_binds_to_curve(
    geometry: GeometryPlan,
    candidate: CandidateTrajectory,
    motion: MotionIntent,
    *,
    tolerance: float = 1e-8,
) -> bool:
    if not _finite_candidate(candidate):
        return False
    duration = candidate.summary.selected_duration_s
    if not math.isfinite(duration) or duration <= 0.0:
        return False
    if motion.yaw_mode != "fixed" or motion.fixed_yaw_deg is None:
        return False
    expected_yaw = math.radians(motion.fixed_yaw_deg)
    schedule = candidate.schedule
    for point in candidate.points:
        u = point.time_s / duration
        if not 0.0 <= u <= 1.0:
            return False
        parameter = min(max(schedule.evaluate(u)[0], 0.0), 1.0)
        q1 = schedule.evaluate(u, 1)[0]
        q2 = schedule.evaluate(u, 2)[0]
        q3 = schedule.evaluate(u, 3)[0]
        position = geometry.curve.evaluate(parameter)
        first = geometry.curve.evaluate(parameter, 1)
        second = geometry.curve.evaluate(parameter, 2)
        third = geometry.curve.evaluate(parameter, 3)
        velocity = tuple(value * q1 / duration for value in first)
        acceleration = tuple(
            (second[axis] * q1**2 + first[axis] * q2) / duration**2
            for axis in range(3)
        )
        jerk = tuple(
            (
                third[axis] * q1**3
                + 3.0 * second[axis] * q1 * q2
                + first[axis] * q3
            )
            / duration**3
            for axis in range(3)
        )
        expected = (
            parameter,
            *position,
            expected_yaw,
            *velocity,
            0.0,
            *acceleration,
            0.0,
            *jerk,
            0.0,
        )
        actual = (
            point.path_parameter,
            point.north_m,
            point.east_m,
            point.down_m,
            point.yaw_rad,
            point.north_velocity_m_s,
            point.east_velocity_m_s,
            point.down_velocity_m_s,
            point.yaw_rate_rad_s,
            point.north_acceleration_m_s2,
            point.east_acceleration_m_s2,
            point.down_acceleration_m_s2,
            point.yaw_acceleration_rad_s2,
            point.north_jerk_m_s3,
            point.east_jerk_m_s3,
            point.down_jerk_m_s3,
            point.yaw_jerk_rad_s3,
        )
        if not np.allclose(expected, actual, atol=tolerance, rtol=tolerance):
            return False
    return True


def _stationary_endpoints(candidate: CandidateTrajectory) -> bool:
    if not candidate.points:
        return False
    for point in (candidate.points[0], candidate.points[-1]):
        derivatives = (
            point.north_velocity_m_s,
            point.east_velocity_m_s,
            point.down_velocity_m_s,
            point.yaw_rate_rad_s,
            point.north_acceleration_m_s2,
            point.east_acceleration_m_s2,
            point.down_acceleration_m_s2,
            point.yaw_acceleration_rad_s2,
            point.north_jerk_m_s3,
            point.east_jerk_m_s3,
            point.down_jerk_m_s3,
            point.yaw_jerk_rad_s3,
        )
        if any(abs(value) > 1e-12 for value in derivatives):
            return False
    return True


def _environment_margin(
    geometry: GeometryPlan,
    environment: EnvironmentSnapshot,
    controller_cruise_height_m: float,
) -> tuple[bool, float]:
    controls = np.asarray(geometry.curve.control_points, dtype=float)
    origin = controls[0]
    relative_xy = controls[:, :2] - origin[:2]
    heights = controller_cruise_height_m - (controls[:, 2] - origin[2])
    radial_margin = environment.max_radius_m - float(
        np.max(np.linalg.norm(relative_xy, axis=1))
    )
    height_margin = environment.max_height_above_launch_m - float(
        np.max(heights)
    )
    drop_margin = environment.max_drop_below_launch_m - float(
        np.max(-heights)
    )
    margin = min(radial_margin, height_margin, drop_margin)
    return margin >= -1e-12, margin


def _dynamic_margins(
    geometry: GeometryPlan,
    candidate: CandidateTrajectory,
    motion: MotionIntent,
    vehicle: VehicleModel,
    *,
    maximum_limit_scale: float,
) -> tuple[bool, float]:
    summary = candidate.summary
    duration = summary.selected_duration_s
    if (
        not math.isfinite(duration)
        or duration <= 0.0
        or not 0.0 < summary.limit_scale <= maximum_limit_scale
    ):
        return False, -math.inf
    preferred_speed = (
        motion.target_speed_m_s
        if motion.target_speed_m_s is not None
        else motion.maximum_speed_m_s
    )
    limits = (
        min(preferred_speed, motion.maximum_speed_m_s, vehicle.maximum_speed_m_s)
        * summary.limit_scale,
        vehicle.maximum_acceleration_m_s2 * summary.limit_scale,
        vehicle.maximum_jerk_m_s3 * summary.limit_scale,
    )
    try:
        bounds = certified_dynamics(
            PathBounds.of(geometry.curve), candidate.schedule, duration
        )
    except ValueError:
        return False, -math.inf
    normalized_margins = [
        (limit - bound) / limit for limit, bound in zip(limits, bounds, strict=True)
    ]
    margin = min(normalized_margins)
    return margin >= -1e-12, margin


def _duration_valid(
    candidate: CandidateTrajectory,
    motion: MotionIntent,
) -> tuple[bool, float | None, str | None]:
    selected = candidate.summary.selected_duration_s
    minimum = candidate.summary.minimum_feasible_duration_s
    if not all(math.isfinite(value) and value > 0.0 for value in (selected, minimum)):
        return False, None, None
    if selected + 1e-12 < minimum:
        return False, selected - minimum, "s"
    requested = motion.requested_duration_s
    if motion.time_mode == "exact":
        passed = requested is not None and math.isclose(
            selected, requested, abs_tol=1e-12, rel_tol=1e-12
        )
        return passed, None if requested is None else selected - requested, "s"
    if motion.time_mode == "maximum":
        passed = requested is not None and selected <= requested + 1e-12
        return passed, None if requested is None else requested - selected, "s"
    return True, selected - minimum, "s"


def verify_candidate_trajectory(
    geometry: GeometryPlan,
    candidate: CandidateTrajectory,
    motion: MotionIntent,
    vehicle: VehicleModel,
    environment: EnvironmentSnapshot,
    *,
    maximum_deviation_m: float,
    verifier_version: str = "baseline-verifier-v1",
    maximum_limit_scale: float = 0.9,
    fidelity_resolution_m: float = 0.01,
    controller_cruise_height_m: float = 15.0,
    maximum_sample_period_s: float = 0.05,
) -> VerificationReport:
    """Build a report without trusting the retimer's pass/fail conclusions."""
    if not 0.0 < maximum_limit_scale <= 1.0:
        raise ValueError("maximum_limit_scale must be in (0, 1]")
    if not math.isfinite(maximum_deviation_m) or maximum_deviation_m < 0.0:
        raise ValueError("maximum_deviation_m must be finite and nonnegative")
    if not math.isfinite(fidelity_resolution_m) or fidelity_resolution_m <= 0.0:
        raise ValueError("fidelity_resolution_m must be finite and positive")
    if (
        not math.isfinite(controller_cruise_height_m)
        or controller_cruise_height_m <= 0.0
    ):
        raise ValueError("controller_cruise_height_m must be finite and positive")
    if (
        not math.isfinite(maximum_sample_period_s)
        or not 0.0 < maximum_sample_period_s <= 0.1
    ):
        raise ValueError("maximum_sample_period_s must be in (0, 0.1]")
    checks = []
    independent_fidelity = measure_path_fidelity(
        geometry.curve,
        geometry.source_points,
        maximum_deviation_m=maximum_deviation_m,
        verification_resolution_m=fidelity_resolution_m,
    )
    if candidate.points:
        maximum_parameter_gap = max(
            (
                right.path_parameter - left.path_parameter
                for left, right in zip(
                    candidate.points, candidate.points[1:], strict=False
                )
            ),
            default=math.inf,
        )
    else:
        maximum_parameter_gap = math.inf
    controller_fidelity_bound = (
        independent_fidelity.symmetric_upper_bound_m
        + geometry.curve.derivative_norm_bound(1) * maximum_parameter_gap
    )
    fidelity_accepted = (
        independent_fidelity.accepted
        and controller_fidelity_bound <= maximum_deviation_m + 1e-12
    )
    checks.append(_check(
        "INT-007",
        fidelity_accepted,
        "Spline and piecewise-linear controller path fit the declared tolerance.",
        margin=maximum_deviation_m - controller_fidelity_bound,
        units="m",
    ))
    structure = _trajectory_structure(candidate, maximum_sample_period_s)
    defects = schedule_defects(candidate.schedule)
    checks.append(_check(
        "PLN-001",
        structure and _stationary_endpoints(candidate) and not defects,
        "Trajectory is finite, ordered, endpoint-inclusive, and stationary "
        "through jerk, and its time law is a monotone clamped quintic"
        + ("." if not defects else f"; defects: {'; '.join(defects)}."),
    ))
    bound = _trajectory_binds_to_curve(geometry, candidate, motion)
    checks.append(_check(
        "VER-003",
        bound,
        "Every emitted position binds to the independently supplied geometry.",
    ))
    environment_ok, environment_margin = _environment_margin(
        geometry, environment, controller_cruise_height_m
    )
    checks.append(_check(
        "ENV-001",
        environment_ok,
        "B-spline control hull stays inside launch-relative environment bounds.",
        margin=environment_margin,
        units="m",
    ))
    dynamics_ok, dynamics_margin = _dynamic_margins(
        geometry,
        candidate,
        motion,
        vehicle,
        maximum_limit_scale=maximum_limit_scale,
    )
    checks.append(_check(
        "PLN-004",
        dynamics_ok,
        "Independent continuous derivative bounds fit the reserved vehicle limits.",
        margin=dynamics_margin if math.isfinite(dynamics_margin) else None,
        units="fraction",
    ))
    duration_ok, duration_margin, duration_units = _duration_valid(candidate, motion)
    checks.append(_check(
        "PLN-007",
        duration_ok,
        "Selected duration satisfies minimum-feasible and requested-time semantics.",
        margin=duration_margin,
        units=duration_units,
    ))
    accepted = all(check["status"] == "PASS" for check in checks)
    return VerificationReport.from_dict({
        "schema_version": 1,
        "verifier_version": verifier_version,
        "candidate_trajectory_sha256": candidate.sha256,
        "accepted": accepted,
        "checks": checks,
    })
