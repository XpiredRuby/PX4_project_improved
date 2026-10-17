"""Conservative analytic retiming for smooth stop-to-stop paths."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math

from .contracts import MotionIntent, VehicleModel, canonical_sha256
from .explain import PlanningError
from .flatness import (
    attitude_duration_s,
    body_rate_bound_rad_s,
    maximum_tilt_rad,
    thrust_interval_n,
)
from .geometry import BSplinePath, GeometryPlan
from .timing import (
    PathBounds,
    TimeLawReport,
    certified_dynamics,
    plan_time_law,
    repeated_bounds,
)
from .yaw import YawTrack, curve_parameter, plan_tangent_yaw


MAX_TRAJECTORY_POINTS = 200_000


def _positive(value: float, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be finite and positive")
    return result


@dataclass(frozen=True)
class TimedTrajectoryPoint:
    time_s: float
    path_parameter: float
    north_m: float
    east_m: float
    down_m: float
    yaw_rad: float
    north_velocity_m_s: float
    east_velocity_m_s: float
    down_velocity_m_s: float
    yaw_rate_rad_s: float
    north_acceleration_m_s2: float
    east_acceleration_m_s2: float
    down_acceleration_m_s2: float
    yaw_acceleration_rad_s2: float
    north_jerk_m_s3: float
    east_jerk_m_s3: float
    down_jerk_m_s3: float
    yaw_jerk_rad_s3: float


@dataclass(frozen=True)
class RetimeSummary:
    minimum_feasible_duration_s: float
    selected_duration_s: float
    limiting_constraint: str
    limit_scale: float
    speed_limit_m_s: float
    acceleration_limit_m_s2: float
    jerk_limit_m_s3: float
    conservative_speed_bound_m_s: float
    conservative_acceleration_bound_m_s2: float
    conservative_jerk_bound_m_s3: float
    time_law: TimeLawReport
    loop_count: int
    yaw_mode: str
    heading_error_bound_rad: float | None
    tilt_bound_deg: float
    minimum_thrust_bound_n: float
    maximum_thrust_bound_n: float
    body_rate_bound_deg_s: float
    segment_count: int


@dataclass(frozen=True)
class CandidateTrajectory:
    schema_version: int
    release_class: str
    points: tuple[TimedTrajectoryPoint, ...]
    schedule: BSplinePath
    summary: RetimeSummary
    yaw_profile: BSplinePath | None = None
    segments: tuple["CandidateTrajectory", ...] = ()

    def to_dict(self):
        return asdict(self)

    @property
    def sha256(self) -> str:
        return canonical_sha256(self)


def _select_duration(
    motion: MotionIntent,
    minimum_feasible_duration_s: float,
) -> float:
    requested = motion.requested_duration_s
    if motion.time_mode == "exact":
        if requested is None:
            raise RuntimeError("validated exact timing has no duration")
        if requested + 1e-12 < minimum_feasible_duration_s:
            raise PlanningError(
                "requested exact duration is infeasible; minimum feasible "
                f"duration is {minimum_feasible_duration_s:.6f} s",
                limiting_constraint="duration",
                minimum_feasible_duration_s=minimum_feasible_duration_s,
            )
        return requested
    if motion.time_mode == "maximum":
        if requested is None:
            raise RuntimeError("validated maximum timing has no duration")
        if minimum_feasible_duration_s > requested + 1e-12:
            raise PlanningError(
                "requested maximum duration is infeasible; minimum feasible "
                f"duration is {minimum_feasible_duration_s:.6f} s",
                limiting_constraint="duration",
                minimum_feasible_duration_s=minimum_feasible_duration_s,
            )
    return minimum_feasible_duration_s


def retime_stop_path(
    geometry: GeometryPlan,
    motion: MotionIntent,
    vehicle: VehicleModel,
    *,
    sample_period_s: float = 0.05,
    limit_scale: float = 0.9,
    minimum_duration_s: float = 1.0,
    loop_count: int = 1,
) -> CandidateTrajectory:
    """Retime a path from rest to rest, including repeated laps of a closed curve.

    A periodic curve starts and stops at its seam. The entry and the exit are
    the stationary ramps of the time law, flown on the loop itself, and any
    laps between them keep the pace the certified bounds allow. Fixed yaw holds
    one heading. Tangent yaw follows a certified approximation of the horizontal
    tangent.
    """
    period = _positive(sample_period_s, "sample_period_s")
    minimum_duration = _positive(minimum_duration_s, "minimum_duration_s")
    if not math.isfinite(limit_scale) or not 0.0 < limit_scale <= 1.0:
        raise ValueError("limit_scale must be finite and in (0, 1]")
    if motion.yaw_mode == "unconstrained":
        raise PlanningError(
            "unconstrained yaw does not produce a flyable heading command; "
            "choose fixed or tangent",
            limiting_constraint="yaw",
        )
    if motion.yaw_mode == "fixed" and motion.fixed_yaw_deg is None:
        raise PlanningError(
            "fixed yaw is missing its heading",
            limiting_constraint="yaw",
        )
    if motion.yaw_mode not in {"fixed", "tangent"}:
        raise PlanningError(
            f"yaw mode {motion.yaw_mode} is not supported",
            limiting_constraint="yaw",
        )

    preferred_speed = (
        motion.target_speed_m_s
        if motion.target_speed_m_s is not None
        else motion.maximum_speed_m_s
    )
    speed_limit = (
        min(preferred_speed, motion.maximum_speed_m_s, vehicle.maximum_speed_m_s)
        * limit_scale
    )
    acceleration_limit = vehicle.maximum_acceleration_m_s2 * limit_scale
    jerk_limit = vehicle.maximum_jerk_m_s3 * limit_scale
    yaw_rate_limit = math.radians(vehicle.maximum_yaw_rate_deg_s) * limit_scale
    yaw_acceleration_limit = (
        math.radians(vehicle.maximum_yaw_acceleration_deg_s2) * limit_scale
    )
    yaw_jerk_limit = math.radians(vehicle.maximum_yaw_jerk_deg_s3) * limit_scale
    yaw_track: YawTrack | None = None
    if motion.yaw_mode == "tangent":
        yaw_track = plan_tangent_yaw(geometry.curve, loops=loop_count)
    law = plan_time_law(
        geometry,
        speed_limit_m_s=speed_limit,
        acceleration_limit_m_s2=acceleration_limit,
        jerk_limit_m_s3=jerk_limit,
        loop_count=loop_count,
        yaw_bounds=None if yaw_track is None else yaw_track.bounds,
        yaw_rate_limit_rad_s=None if yaw_track is None else yaw_rate_limit,
        yaw_acceleration_limit_rad_s2=(
            None if yaw_track is None else yaw_acceleration_limit
        ),
        yaw_jerk_limit_rad_s3=None if yaw_track is None else yaw_jerk_limit,
    )
    path_bounds = repeated_bounds(PathBounds.of(geometry.curve), loop_count)
    _, acceleration_numerator, jerk_numerator = certified_dynamics(
        path_bounds, law.schedule, 1.0
    )
    if yaw_track is None:
        yaw_rate_numerator = 0.0
    else:
        yaw_rate_numerator, _, _ = certified_dynamics(
            yaw_track.bounds, law.schedule, 1.0
        )
    attitude_duration, attitude_constraint = attitude_duration_s(
        acceleration_numerator=acceleration_numerator,
        jerk_numerator=jerk_numerator,
        yaw_rate_numerator=yaw_rate_numerator,
        mass_kg=vehicle.mass_kg,
        tilt_limit_rad=math.radians(vehicle.maximum_tilt_deg),
        minimum_thrust_n=vehicle.minimum_thrust_n,
        maximum_thrust_n=vehicle.maximum_thrust_n,
        maximum_body_rate_rad_s=math.radians(vehicle.maximum_body_rate_deg_s)
        * limit_scale,
    )
    constrained_duration = max(
        law.report.minimum_requirement_s, attitude_duration
    )
    limiting_constraint = law.report.limiting_constraint
    if attitude_duration > law.report.minimum_requirement_s + 1e-9:
        limiting_constraint = attitude_constraint
    minimum_feasible = max(minimum_duration, constrained_duration)
    if minimum_duration >= constrained_duration:
        limiting_constraint = "minimum_duration"
    duration = _select_duration(motion, minimum_feasible)

    intervals = max(1, math.ceil(duration / period))
    if intervals + 1 > MAX_TRAJECTORY_POINTS:
        raise ValueError(
            f"trajectory exceeds {MAX_TRAJECTORY_POINTS} output points"
        )
    points: list[TimedTrajectoryPoint] = []
    fixed_yaw = (
        0.0 if motion.fixed_yaw_deg is None else math.radians(motion.fixed_yaw_deg)
    )
    max_speed = 0.0
    max_acceleration = 0.0
    max_jerk = 0.0
    max_yaw_rate = 0.0
    max_yaw_acceleration = 0.0
    max_yaw_jerk = 0.0
    for index in range(intervals + 1):
        time_s = duration if index == intervals else duration * index / intervals
        normalized, q1, q2, q3 = law.sample(index / intervals)
        unwrapped = normalized * loop_count
        parameter = curve_parameter(geometry.curve, unwrapped, loop_count)
        first = geometry.curve.evaluate(parameter, 1)
        second = geometry.curve.evaluate(parameter, 2)
        third = geometry.curve.evaluate(parameter, 3)
        position = geometry.curve.evaluate(parameter)
        rate = q1 * loop_count
        acceleration_rate = q2 * loop_count
        jerk_rate = q3 * loop_count
        velocity = tuple(value * rate / duration for value in first)
        acceleration = tuple(
            (second[axis] * rate**2 + first[axis] * acceleration_rate)
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
        if yaw_track is None:
            yaw = fixed_yaw
            yaw_rate = yaw_acceleration = yaw_jerk = 0.0
        else:
            yaw_first = yaw_track.profile.evaluate(normalized, 1)[0]
            yaw_second = yaw_track.profile.evaluate(normalized, 2)[0]
            yaw_third = yaw_track.profile.evaluate(normalized, 3)[0]
            yaw = yaw_track.profile.evaluate(normalized)[0]
            yaw_rate = yaw_first * q1 / duration
            yaw_acceleration = (
                yaw_second * q1**2 + yaw_first * q2
            ) / duration**2
            yaw_jerk = (
                yaw_third * q1**3
                + 3.0 * yaw_second * q1 * q2
                + yaw_first * q3
            ) / duration**3
        max_speed = max(max_speed, math.sqrt(sum(value**2 for value in velocity)))
        max_acceleration = max(
            max_acceleration,
            math.sqrt(sum(value**2 for value in acceleration)),
        )
        max_jerk = max(max_jerk, math.sqrt(sum(value**2 for value in jerk)))
        max_yaw_rate = max(max_yaw_rate, abs(yaw_rate))
        max_yaw_acceleration = max(max_yaw_acceleration, abs(yaw_acceleration))
        max_yaw_jerk = max(max_yaw_jerk, abs(yaw_jerk))
        points.append(TimedTrajectoryPoint(
            time_s=time_s,
            path_parameter=unwrapped,
            north_m=position[0],
            east_m=position[1],
            down_m=position[2],
            yaw_rad=yaw,
            north_velocity_m_s=velocity[0],
            east_velocity_m_s=velocity[1],
            down_velocity_m_s=velocity[2],
            yaw_rate_rad_s=yaw_rate,
            north_acceleration_m_s2=acceleration[0],
            east_acceleration_m_s2=acceleration[1],
            down_acceleration_m_s2=acceleration[2],
            yaw_acceleration_rad_s2=yaw_acceleration,
            north_jerk_m_s3=jerk[0],
            east_jerk_m_s3=jerk[1],
            down_jerk_m_s3=jerk[2],
            yaw_jerk_rad_s3=yaw_jerk,
        ))

    (
        conservative_speed,
        conservative_acceleration,
        conservative_jerk,
    ) = certified_dynamics(path_bounds, law.schedule, duration)
    tolerance = 1.0 + 1e-9
    if (
        max_speed > speed_limit * tolerance
        or max_acceleration > acceleration_limit * tolerance
        or max_jerk > jerk_limit * tolerance
        or max_yaw_rate > yaw_rate_limit * tolerance
        or max_yaw_acceleration > yaw_acceleration_limit * tolerance
        or max_yaw_jerk > yaw_jerk_limit * tolerance
    ):
        raise RuntimeError("sampled trajectory exceeds its retiming limits")
    tilt_bound = math.degrees(maximum_tilt_rad(conservative_acceleration))
    thrust_low, thrust_high = thrust_interval_n(
        conservative_acceleration, vehicle.mass_kg
    )
    yaw_rate_bound = 0.0 if yaw_track is None else certified_dynamics(
        yaw_track.bounds, law.schedule, duration
    )[0]
    body_rate_bound = math.degrees(body_rate_bound_rad_s(
        conservative_acceleration,
        conservative_jerk,
        yaw_rate_bound,
    ))
    return CandidateTrajectory(
        schema_version=1,
        release_class="SITL_ONLY",
        points=tuple(points),
        schedule=law.schedule,
        yaw_profile=None if yaw_track is None else yaw_track.profile,
        summary=RetimeSummary(
            minimum_feasible_duration_s=minimum_feasible,
            selected_duration_s=duration,
            limiting_constraint=limiting_constraint,
            limit_scale=limit_scale,
            speed_limit_m_s=speed_limit,
            acceleration_limit_m_s2=acceleration_limit,
            jerk_limit_m_s3=jerk_limit,
            conservative_speed_bound_m_s=conservative_speed,
            conservative_acceleration_bound_m_s2=conservative_acceleration,
            conservative_jerk_bound_m_s3=conservative_jerk,
            time_law=law.report,
            loop_count=loop_count,
            yaw_mode=motion.yaw_mode,
            heading_error_bound_rad=(
                None if yaw_track is None else yaw_track.error_bound_rad
            ),
            tilt_bound_deg=tilt_bound,
            minimum_thrust_bound_n=thrust_low,
            maximum_thrust_bound_n=thrust_high,
            body_rate_bound_deg_s=body_rate_bound,
            segment_count=1,
        ),
    )


def concatenate_candidates(
    parts: tuple[CandidateTrajectory, ...],
) -> CandidateTrajectory:
    """Join rest-to-rest segments, dropping the duplicated joint sample."""
    if not parts:
        raise ValueError("concatenation requires at least one segment")
    if len(parts) == 1:
        return parts[0]
    points: list[TimedTrajectoryPoint] = []
    offset = 0.0
    for index, part in enumerate(parts):
        for point_index, point in enumerate(part.points):
            if index and point_index == 0:
                continue
            points.append(replace(point, time_s=point.time_s + offset))
        offset += part.summary.selected_duration_s
    longest = max(parts, key=lambda part: part.summary.selected_duration_s)
    return CandidateTrajectory(
        schema_version=1,
        release_class="SITL_ONLY",
        points=tuple(points),
        schedule=longest.schedule,
        yaw_profile=longest.yaw_profile,
        segments=parts,
        summary=replace(
            longest.summary,
            minimum_feasible_duration_s=sum(
                part.summary.minimum_feasible_duration_s for part in parts
            ),
            selected_duration_s=offset,
            segment_count=len(parts),
            tilt_bound_deg=max(
                part.summary.tilt_bound_deg for part in parts
            ),
            minimum_thrust_bound_n=min(
                part.summary.minimum_thrust_bound_n for part in parts
            ),
            maximum_thrust_bound_n=max(
                part.summary.maximum_thrust_bound_n for part in parts
            ),
            body_rate_bound_deg_s=max(
                part.summary.body_rate_bound_deg_s for part in parts
            ),
        ),
    )
