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
from .flatness import (
    body_rate_bound_rad_s,
    maximum_tilt_rad,
    thrust_interval_n,
)
from .geometry import GeometryPlan, measure_path_fidelity
from .retiming import CandidateTrajectory
from .timing import PathBounds, certified_dynamics, repeated_bounds, schedule_defects
from .yaw import (
    MAXIMUM_HEADING_ERROR_RAD,
    curve_parameter,
    heading_error_bound,
)


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
        and points[-1].path_parameter == candidate.summary.loop_count
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
    loops = candidate.summary.loop_count
    schedule = candidate.schedule
    profile = candidate.yaw_profile
    if motion.yaw_mode == "tangent":
        if profile is None:
            return False
        try:
            error = heading_error_bound(
                geometry.curve, profile, loops=loops, count=8_000
            )
        except ValueError:
            return False
        if error > MAXIMUM_HEADING_ERROR_RAD + 1e-9:
            return False
    elif motion.yaw_mode != "fixed" or profile is not None or motion.fixed_yaw_deg is None:
        return False
    fixed_yaw = (
        0.0 if motion.fixed_yaw_deg is None else math.radians(motion.fixed_yaw_deg)
    )
    for point in candidate.points:
        u = point.time_s / duration
        if not 0.0 <= u <= 1.0:
            return False
        normalized = schedule.evaluate(u)[0]
        end = schedule.control_points[-1][0]
        normalized = min(max(normalized, 0.0), end)
        unwrapped = normalized * loops
        parameter = curve_parameter(geometry.curve, unwrapped, loops)
        q1 = schedule.evaluate(u, 1)[0]
        q2 = schedule.evaluate(u, 2)[0]
        q3 = schedule.evaluate(u, 3)[0]
        rate = q1 * loops
        acceleration_rate = q2 * loops
        jerk_rate = q3 * loops
        if motion.yaw_mode == "tangent":
            yaw_first = profile.evaluate(normalized, 1)[0]
            yaw_second = profile.evaluate(normalized, 2)[0]
            yaw_third = profile.evaluate(normalized, 3)[0]
            expected_yaw = profile.evaluate(normalized)[0]
            expected_yaw_rate = yaw_first * q1 / duration
            expected_yaw_acceleration = (
                yaw_second * q1**2 + yaw_first * q2
            ) / duration**2
            expected_yaw_jerk = (
                yaw_third * q1**3
                + 3.0 * yaw_second * q1 * q2
                + yaw_first * q3
            ) / duration**3
        else:
            expected_yaw = fixed_yaw
            expected_yaw_rate = 0.0
            expected_yaw_acceleration = 0.0
            expected_yaw_jerk = 0.0
        position = geometry.curve.evaluate(parameter)
        first = geometry.curve.evaluate(parameter, 1)
        second = geometry.curve.evaluate(parameter, 2)
        third = geometry.curve.evaluate(parameter, 3)
        velocity = tuple(value * rate / duration for value in first)
        acceleration = tuple(
            (
                second[axis] * rate**2 + first[axis] * acceleration_rate
            )
            / duration**2
            for axis in range(3)
        )
        jerk = tuple(
            (
                third[axis] * rate**3
                + 3.0 * second[axis] * rate * acceleration_rate
                + first[axis] * jerk_rate
            )
            / duration**3
            for axis in range(3)
        )
        expected = (
            unwrapped,
            *position,
            expected_yaw,
            *velocity,
            expected_yaw_rate,
            *acceleration,
            expected_yaw_acceleration,
            *jerk,
            expected_yaw_jerk,
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
    scale = summary.limit_scale
    limits = [
        min(preferred_speed, motion.maximum_speed_m_s, vehicle.maximum_speed_m_s)
        * scale,
        vehicle.maximum_acceleration_m_s2 * scale,
        vehicle.maximum_jerk_m_s3 * scale,
    ]
    try:
        bounds = list(certified_dynamics(
            repeated_bounds(PathBounds.of(geometry.curve), summary.loop_count),
            candidate.schedule,
            duration,
        ))
        if motion.yaw_mode == "tangent":
            if candidate.yaw_profile is None:
                return False, -math.inf
            yaw_bounds = certified_dynamics(
                PathBounds.of(candidate.yaw_profile),
                candidate.schedule,
                duration,
            )
            limits.extend((
                math.radians(vehicle.maximum_yaw_rate_deg_s) * scale,
                math.radians(vehicle.maximum_yaw_acceleration_deg_s2) * scale,
                math.radians(vehicle.maximum_yaw_jerk_deg_s3) * scale,
            ))
            bounds.extend(yaw_bounds)
        elif candidate.yaw_profile is not None:
            return False, -math.inf
        acceleration = bounds[1]
        jerk = bounds[2]
        yaw_rate = bounds[3] if motion.yaw_mode == "tangent" else 0.0
        tilt = maximum_tilt_rad(acceleration)
        thrust_low, thrust_high = thrust_interval_n(acceleration, vehicle.mass_kg)
        body_rate = body_rate_bound_rad_s(acceleration, jerk, yaw_rate)
    except ValueError:
        return False, -math.inf
    normalized_margins = [
        (limit - bound) / limit for limit, bound in zip(limits, bounds, strict=True)
    ]
    normalized_margins.append(
        (math.radians(vehicle.maximum_tilt_deg) - tilt)
        / math.radians(vehicle.maximum_tilt_deg)
    )
    thrust_span = vehicle.maximum_thrust_n - vehicle.minimum_thrust_n
    normalized_margins.append((vehicle.maximum_thrust_n - thrust_high) / thrust_span)
    normalized_margins.append((thrust_low - vehicle.minimum_thrust_n) / thrust_span)
    body_limit = math.radians(vehicle.maximum_body_rate_deg_s) * scale
    if not math.isfinite(body_rate):
        return False, -math.inf
    normalized_margins.append((body_limit - body_rate) / body_limit)
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


def merge_verification_reports(
    reports: tuple[VerificationReport, ...],
    candidate_sha256: str,
) -> VerificationReport:
    """Combine per-segment reports, keeping the worst margin of each requirement."""
    if not reports:
        raise ValueError("merging verification requires at least one report")
    identifiers = [check.requirement_id for check in reports[0].checks]
    merged = []
    for requirement in identifiers:
        checks = []
        for report in reports:
            match = [
                check for check in report.checks
                if check.requirement_id == requirement
            ]
            if len(match) != 1:
                raise ValueError(
                    f"segment reports disagree on requirement {requirement}"
                )
            checks.append(match[0])
        failed = [check for check in checks if check.status != "PASS"]
        margins = [check.margin for check in checks if check.margin is not None]
        chosen = failed[0] if failed else checks[0]
        detail = chosen.detail
        if failed:
            detail = f"segment {checks.index(failed[0]) + 1}: {failed[0].detail}"
        merged.append({
            "requirement_id": requirement,
            "status": "FAIL" if failed else "PASS",
            "hard": True,
            "margin": None if not margins else min(margins),
            "units": chosen.units,
            "detail": detail[:1024],
        })
    accepted = all(check["status"] == "PASS" for check in merged)
    return VerificationReport.from_dict({
        "schema_version": 1,
        "verifier_version": reports[0].verifier_version,
        "candidate_trajectory_sha256": candidate_sha256,
        "accepted": accepted,
        "checks": merged,
    })


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
