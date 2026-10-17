"""Conservative analytic retiming for smooth stop-to-stop paths."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math

from .contracts import MotionIntent, VehicleModel, canonical_sha256
from .geometry import GeometryPlan


# Exact normalized derivative maxima for
# q(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7, u in [0, 1].
PEAK_Q1 = 2.1875
PEAK_Q2 = 7.513188404399301
PEAK_Q3 = 52.5
MAX_TRAJECTORY_POINTS = 200_000


def _positive(value: float, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be finite and positive")
    return result


def _smootherstep7(value: float) -> tuple[float, float, float, float]:
    if value <= 0.0:
        return 0.0, 0.0, 0.0, 0.0
    if value >= 1.0:
        return 1.0, 0.0, 0.0, 0.0
    u = value
    position = 35.0 * u**4 - 84.0 * u**5 + 70.0 * u**6 - 20.0 * u**7
    velocity = 140.0 * u**3 - 420.0 * u**4 + 420.0 * u**5 - 140.0 * u**6
    acceleration = (
        420.0 * u**2 - 1680.0 * u**3 + 2100.0 * u**4 - 840.0 * u**5
    )
    jerk = 840.0 * u - 5040.0 * u**2 + 8400.0 * u**3 - 4200.0 * u**4
    return position, velocity, acceleration, jerk


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


@dataclass(frozen=True)
class CandidateTrajectory:
    schema_version: int
    release_class: str
    points: tuple[TimedTrajectoryPoint, ...]
    summary: RetimeSummary

    def to_dict(self):
        return asdict(self)

    @property
    def sha256(self) -> str:
        return canonical_sha256(self)


def _duration_requirements(
    geometry: GeometryPlan,
    *,
    speed_limit_m_s: float,
    acceleration_limit_m_s2: float,
    jerk_limit_m_s3: float,
) -> dict[str, float]:
    first = geometry.curve.derivative_norm_bound(1)
    second = geometry.curve.derivative_norm_bound(2)
    third = geometry.curve.derivative_norm_bound(3)
    normalized_speed = first * PEAK_Q1
    normalized_acceleration = second * PEAK_Q1**2 + first * PEAK_Q2
    normalized_jerk = (
        third * PEAK_Q1**3
        + 3.0 * second * PEAK_Q1 * PEAK_Q2
        + first * PEAK_Q3
    )
    return {
        "speed": normalized_speed / speed_limit_m_s,
        "acceleration": math.sqrt(
            normalized_acceleration / acceleration_limit_m_s2
        ),
        "jerk": (normalized_jerk / jerk_limit_m_s3) ** (1.0 / 3.0),
    }


def _select_duration(
    motion: MotionIntent,
    minimum_feasible_duration_s: float,
) -> float:
    requested = motion.requested_duration_s
    if motion.time_mode == "exact":
        if requested is None:
            raise RuntimeError("validated exact timing has no duration")
        if requested + 1e-12 < minimum_feasible_duration_s:
            raise ValueError(
                "requested exact duration is infeasible; minimum feasible "
                f"duration is {minimum_feasible_duration_s:.6f} s"
            )
        return requested
    if motion.time_mode == "maximum":
        if requested is None:
            raise RuntimeError("validated maximum timing has no duration")
        if minimum_feasible_duration_s > requested + 1e-12:
            raise ValueError(
                "requested maximum duration is infeasible; minimum feasible "
                f"duration is {minimum_feasible_duration_s:.6f} s"
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
) -> CandidateTrajectory:
    """Retime an open, fixed-yaw path from rest to rest with analytic bounds."""
    period = _positive(sample_period_s, "sample_period_s")
    minimum_duration = _positive(minimum_duration_s, "minimum_duration_s")
    if not math.isfinite(limit_scale) or not 0.0 < limit_scale <= 1.0:
        raise ValueError("limit_scale must be finite and in (0, 1]")
    if geometry.curve.periodic:
        raise ValueError(
            "periodic execution requires a separately planned entry and exit"
        )
    if motion.yaw_mode != "fixed" or motion.fixed_yaw_deg is None:
        raise ValueError("baseline retiming currently requires fixed yaw")

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
    requirements = _duration_requirements(
        geometry,
        speed_limit_m_s=speed_limit,
        acceleration_limit_m_s2=acceleration_limit,
        jerk_limit_m_s3=jerk_limit,
    )
    limiting_constraint, constrained_duration = max(
        requirements.items(), key=lambda item: (item[1], item[0])
    )
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
    yaw = math.radians(motion.fixed_yaw_deg)
    max_speed = 0.0
    max_acceleration = 0.0
    max_jerk = 0.0
    for index in range(intervals + 1):
        time_s = duration if index == intervals else duration * index / intervals
        u = index / intervals
        parameter, q1, q2, q3 = _smootherstep7(u)
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
        max_speed = max(max_speed, math.sqrt(sum(value**2 for value in velocity)))
        max_acceleration = max(
            max_acceleration,
            math.sqrt(sum(value**2 for value in acceleration)),
        )
        max_jerk = max(max_jerk, math.sqrt(sum(value**2 for value in jerk)))
        points.append(TimedTrajectoryPoint(
            time_s=time_s,
            path_parameter=parameter,
            north_m=position[0],
            east_m=position[1],
            down_m=position[2],
            yaw_rad=yaw,
            north_velocity_m_s=velocity[0],
            east_velocity_m_s=velocity[1],
            down_velocity_m_s=velocity[2],
            yaw_rate_rad_s=0.0,
            north_acceleration_m_s2=acceleration[0],
            east_acceleration_m_s2=acceleration[1],
            down_acceleration_m_s2=acceleration[2],
            yaw_acceleration_rad_s2=0.0,
            north_jerk_m_s3=jerk[0],
            east_jerk_m_s3=jerk[1],
            down_jerk_m_s3=jerk[2],
            yaw_jerk_rad_s3=0.0,
        ))

    conservative_speed = (
        geometry.curve.derivative_norm_bound(1) * PEAK_Q1 / duration
    )
    conservative_acceleration = (
        geometry.curve.derivative_norm_bound(2) * PEAK_Q1**2
        + geometry.curve.derivative_norm_bound(1) * PEAK_Q2
    ) / duration**2
    conservative_jerk = (
        geometry.curve.derivative_norm_bound(3) * PEAK_Q1**3
        + 3.0
        * geometry.curve.derivative_norm_bound(2)
        * PEAK_Q1
        * PEAK_Q2
        + geometry.curve.derivative_norm_bound(1) * PEAK_Q3
    ) / duration**3
    tolerance = 1.0 + 1e-9
    if (
        max_speed > speed_limit * tolerance
        or max_acceleration > acceleration_limit * tolerance
        or max_jerk > jerk_limit * tolerance
    ):
        raise RuntimeError("sampled trajectory exceeds its retiming limits")
    return CandidateTrajectory(
        schema_version=1,
        release_class="SITL_ONLY",
        points=tuple(points),
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
        ),
    )
