"""Strict local-NED waypoint missions with stop-to-stop C3 trajectories."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from pose import TrajectoryPoint


SCHEMA_VERSION = 1
SUPPORTED_FRAME = "LOCAL_NED"
MAX_STOPS = 64

# Exact normalized derivative maxima for
# q(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7, u in [0, 1].
PEAK_D1 = 2.1875
PEAK_D2 = 7.513188404399301
PEAK_D3 = 52.5


def _mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be an object")
    return value


def _only_fields(value: dict[str, Any], allowed: set[str], field: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError(f"{field} has unknown fields: {', '.join(unknown)}")


def _number(value: Any, field: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    if minimum is not None and result < minimum:
        raise ValueError(f"{field} must be at least {minimum}")
    return result


def _positive(value: Any, field: str) -> float:
    result = _number(value, field)
    if result <= 0.0:
        raise ValueError(f"{field} must be positive")
    return result


@dataclass(frozen=True)
class MissionLimits:
    max_horizontal_speed_m_s: float
    max_vertical_speed_m_s: float
    max_acceleration_m_s2: float
    max_jerk_m_s3: float
    max_yaw_rate_deg_s: float
    max_yaw_acceleration_deg_s2: float
    max_yaw_jerk_deg_s3: float
    minimum_segment_duration_s: float

    @classmethod
    def from_dict(cls, raw: Any) -> "MissionLimits":
        values = _mapping(raw, "limits")
        fields = set(cls.__dataclass_fields__)
        _only_fields(values, fields, "limits")
        missing = sorted(fields - set(values))
        if missing:
            raise ValueError("limits missing fields: " + ", ".join(missing))
        return cls(**{
            name: _positive(values[name], f"limits.{name}")
            for name in fields
        })


@dataclass(frozen=True)
class MissionSafety:
    max_radius_from_start_m: float
    max_vertical_excursion_from_start_m: float
    max_total_distance_m: float
    max_duration_s: float

    @classmethod
    def from_dict(cls, raw: Any) -> "MissionSafety":
        values = _mapping(raw, "safety")
        fields = set(cls.__dataclass_fields__)
        _only_fields(values, fields, "safety")
        missing = sorted(fields - set(values))
        if missing:
            raise ValueError("safety missing fields: " + ", ".join(missing))
        return cls(**{
            name: _positive(values[name], f"safety.{name}")
            for name in fields
        })


@dataclass(frozen=True)
class MissionStop:
    name: str
    north_m: float
    east_m: float
    down_m: float
    yaw_deg: float
    hold_s: float

    @property
    def position(self) -> tuple[float, float, float]:
        return self.north_m, self.east_m, self.down_m

    @classmethod
    def from_dict(cls, raw: Any, index: int) -> "MissionStop":
        values = _mapping(raw, f"stops[{index}]")
        fields = set(cls.__dataclass_fields__)
        _only_fields(values, fields, f"stops[{index}]")
        required = fields - {"hold_s"}
        missing = sorted(required - set(values))
        if missing:
            raise ValueError(
                f"stops[{index}] missing fields: " + ", ".join(missing)
            )
        name = values["name"]
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"stops[{index}].name must be a non-empty string")
        yaw_deg = _number(values["yaw_deg"], f"stops[{index}].yaw_deg")
        if not -180.0 <= yaw_deg <= 180.0:
            raise ValueError(f"stops[{index}].yaw_deg must be in [-180, 180]")
        return cls(
            name=name.strip(),
            north_m=_number(values["north_m"], f"stops[{index}].north_m"),
            east_m=_number(values["east_m"], f"stops[{index}].east_m"),
            down_m=_number(values["down_m"], f"stops[{index}].down_m"),
            yaw_deg=yaw_deg,
            hold_s=_number(
                values.get("hold_s", 0.0),
                f"stops[{index}].hold_s",
                minimum=0.0,
            ),
        )


@dataclass(frozen=True)
class WaypointMission:
    schema_version: int
    frame: str
    sample_period_s: float
    limits: MissionLimits
    safety: MissionSafety
    stops: tuple[MissionStop, ...]

    @classmethod
    def from_dict(cls, raw: Any) -> "WaypointMission":
        values = _mapping(raw, "mission")
        fields = set(cls.__dataclass_fields__)
        _only_fields(values, fields, "mission")
        missing = sorted(fields - set(values))
        if missing:
            raise ValueError("mission missing fields: " + ", ".join(missing))
        version = values["schema_version"]
        if (
            not isinstance(version, int)
            or isinstance(version, bool)
            or version != SCHEMA_VERSION
        ):
            raise ValueError(f"schema_version must be {SCHEMA_VERSION}")
        if values["frame"] != SUPPORTED_FRAME:
            raise ValueError(f"frame must be {SUPPORTED_FRAME}")
        sample_period = _positive(values["sample_period_s"], "sample_period_s")
        if not 0.01 <= sample_period <= 0.1:
            raise ValueError("sample_period_s must be in [0.01, 0.1]")
        stop_values = values["stops"]
        if not isinstance(stop_values, list):
            raise ValueError("stops must be an array")
        if not 2 <= len(stop_values) <= MAX_STOPS:
            raise ValueError(f"stops must contain 2..{MAX_STOPS} entries")
        stops = tuple(
            MissionStop.from_dict(stop, index)
            for index, stop in enumerate(stop_values)
        )
        names = [stop.name for stop in stops]
        if len(names) != len(set(names)):
            raise ValueError("stop names must be unique")
        mission = cls(
            schema_version=SCHEMA_VERSION,
            frame=SUPPORTED_FRAME,
            sample_period_s=sample_period,
            limits=MissionLimits.from_dict(values["limits"]),
            safety=MissionSafety.from_dict(values["safety"]),
            stops=stops,
        )
        mission.validate_geometry()
        return mission

    def validate_geometry(self) -> None:
        start = self.stops[0]
        distance = 0.0
        for index, stop in enumerate(self.stops):
            radius = math.hypot(
                stop.north_m - start.north_m,
                stop.east_m - start.east_m,
            )
            vertical = abs(stop.down_m - start.down_m)
            if radius > self.safety.max_radius_from_start_m + 1e-9:
                raise ValueError(
                    f"stops[{index}] exceeds max_radius_from_start_m"
                )
            if vertical > self.safety.max_vertical_excursion_from_start_m + 1e-9:
                raise ValueError(
                    f"stops[{index}] exceeds max_vertical_excursion_from_start_m"
                )
            if index == 0:
                continue
            previous = self.stops[index - 1]
            delta = tuple(
                right - left
                for left, right in zip(
                    previous.position, stop.position, strict=True
                )
            )
            yaw_delta = _wrapped_delta_radians(
                math.radians(previous.yaw_deg),
                math.radians(stop.yaw_deg),
            )
            segment_distance = math.sqrt(sum(value**2 for value in delta))
            if segment_distance <= 1e-9 and abs(yaw_delta) <= 1e-9:
                raise ValueError(
                    f"stops[{index - 1}] and stops[{index}] are coincident"
                )
            distance += segment_distance
        if distance > self.safety.max_total_distance_m + 1e-9:
            raise ValueError("mission exceeds max_total_distance_m")


@dataclass(frozen=True)
class SegmentSummary:
    start_stop: str
    end_stop: str
    duration_s: float
    distance_m: float
    yaw_change_deg: float
    peak_horizontal_speed_m_s: float
    peak_vertical_speed_m_s: float
    peak_acceleration_m_s2: float
    peak_jerk_m_s3: float
    peak_yaw_rate_deg_s: float
    peak_yaw_acceleration_deg_s2: float
    peak_yaw_jerk_deg_s3: float


@dataclass(frozen=True)
class PlannedMission:
    points: tuple[TrajectoryPoint, ...]
    stop_arrival_s: dict[str, float]
    stop_departure_s: dict[str, float]
    segments: tuple[SegmentSummary, ...]
    duration_s: float


def load_mission(path: str | Path) -> WaypointMission:
    source = Path(path)

    def reject_constant(value: str) -> None:
        raise ValueError(f"mission contains non-finite JSON constant: {value}")

    try:
        raw = json.loads(
            source.read_text(encoding="utf-8"),
            parse_constant=reject_constant,
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load mission plan {source}: {exc}") from exc
    return WaypointMission.from_dict(raw)


def _wrapped_delta_radians(start: float, finish: float) -> float:
    return math.atan2(math.sin(finish - start), math.cos(finish - start))


def _smootherstep7(value: float) -> tuple[float, float, float, float]:
    if value <= 0.0:
        return 0.0, 0.0, 0.0, 0.0
    if value >= 1.0:
        return 1.0, 0.0, 0.0, 0.0
    u = value
    position = 35.0 * u**4 - 84.0 * u**5 + 70.0 * u**6 - 20.0 * u**7
    velocity = 140.0 * u**3 - 420.0 * u**4 + 420.0 * u**5 - 140.0 * u**6
    acceleration = 420.0 * u**2 - 1680.0 * u**3 + 2100.0 * u**4 - 840.0 * u**5
    jerk = 840.0 * u - 5040.0 * u**2 + 8400.0 * u**3 - 4200.0 * u**4
    return position, velocity, acceleration, jerk


def _segment_duration(
    start: MissionStop,
    finish: MissionStop,
    limits: MissionLimits,
) -> tuple[float, tuple[float, float, float], float]:
    delta = tuple(
        right - left
        for left, right in zip(start.position, finish.position, strict=True)
    )
    horizontal = math.hypot(delta[0], delta[1])
    vertical = abs(delta[2])
    distance = math.sqrt(sum(value**2 for value in delta))
    yaw_delta = _wrapped_delta_radians(
        math.radians(start.yaw_deg), math.radians(finish.yaw_deg)
    )
    yaw_distance = abs(yaw_delta)
    duration = max(
        limits.minimum_segment_duration_s,
        PEAK_D1 * horizontal / limits.max_horizontal_speed_m_s,
        PEAK_D1 * vertical / limits.max_vertical_speed_m_s,
        math.sqrt(PEAK_D2 * distance / limits.max_acceleration_m_s2),
        (PEAK_D3 * distance / limits.max_jerk_m_s3) ** (1.0 / 3.0),
        PEAK_D1 * yaw_distance / math.radians(limits.max_yaw_rate_deg_s),
        math.sqrt(
            PEAK_D2
            * yaw_distance
            / math.radians(limits.max_yaw_acceleration_deg_s2)
        ),
        (
            PEAK_D3
            * yaw_distance
            / math.radians(limits.max_yaw_jerk_deg_s3)
        ) ** (1.0 / 3.0),
    )
    return duration, delta, yaw_delta


def _sample_times(duration_s: float, period_s: float) -> list[float]:
    intervals = max(1, math.ceil(duration_s / period_s))
    return [
        duration_s * index / intervals
        for index in range(intervals + 1)
    ]


def _stationary_point(time_s: float, stop: MissionStop, yaw_rad: float) -> TrajectoryPoint:
    return TrajectoryPoint(
        time=time_s,
        x=stop.north_m,
        y=stop.east_m,
        z=stop.down_m,
        yaw=yaw_rad,
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw_rate=0.0,
        ax=0.0,
        ay=0.0,
        az=0.0,
        yaw_acceleration=0.0,
        jx=0.0,
        jy=0.0,
        jz=0.0,
        yaw_jerk=0.0,
    )


def _summary(
    start: MissionStop,
    finish: MissionStop,
    duration: float,
    delta: tuple[float, float, float],
    yaw_delta: float,
) -> SegmentSummary:
    horizontal = math.hypot(delta[0], delta[1])
    vertical = abs(delta[2])
    distance = math.sqrt(sum(value**2 for value in delta))
    yaw_scale = abs(yaw_delta)
    return SegmentSummary(
        start_stop=start.name,
        end_stop=finish.name,
        duration_s=duration,
        distance_m=distance,
        yaw_change_deg=math.degrees(yaw_delta),
        peak_horizontal_speed_m_s=horizontal * PEAK_D1 / duration,
        peak_vertical_speed_m_s=vertical * PEAK_D1 / duration,
        peak_acceleration_m_s2=distance * PEAK_D2 / duration**2,
        peak_jerk_m_s3=distance * PEAK_D3 / duration**3,
        peak_yaw_rate_deg_s=math.degrees(yaw_scale * PEAK_D1 / duration),
        peak_yaw_acceleration_deg_s2=math.degrees(
            yaw_scale * PEAK_D2 / duration**2
        ),
        peak_yaw_jerk_deg_s3=math.degrees(
            yaw_scale * PEAK_D3 / duration**3
        ),
    )


def _validate_points(points: list[TrajectoryPoint], limits: MissionLimits) -> None:
    tolerance = 1.000001
    for index, point in enumerate(points):
        if not all(math.isfinite(value) for value in vars(point).values()):
            raise RuntimeError(f"planned point {index} is non-finite")
        horizontal_speed = math.hypot(point.vx, point.vy)
        acceleration = math.sqrt(point.ax**2 + point.ay**2 + point.az**2)
        jerk = math.sqrt(point.jx**2 + point.jy**2 + point.jz**2)
        violations = (
            horizontal_speed > limits.max_horizontal_speed_m_s * tolerance,
            abs(point.vz) > limits.max_vertical_speed_m_s * tolerance,
            acceleration > limits.max_acceleration_m_s2 * tolerance,
            jerk > limits.max_jerk_m_s3 * tolerance,
            abs(math.degrees(point.yaw_rate))
            > limits.max_yaw_rate_deg_s * tolerance,
            abs(math.degrees(point.yaw_acceleration))
            > limits.max_yaw_acceleration_deg_s2 * tolerance,
            abs(math.degrees(point.yaw_jerk))
            > limits.max_yaw_jerk_deg_s3 * tolerance,
        )
        if any(violations):
            raise RuntimeError(f"planned point {index} exceeds motion limits")


def plan_mission(mission: WaypointMission) -> PlannedMission:
    """Compile a validated mission into endpoint-inclusive trajectory points."""
    mission.validate_geometry()
    points: list[TrajectoryPoint] = []
    summaries: list[SegmentSummary] = []
    arrivals = {mission.stops[0].name: 0.0}
    departures: dict[str, float] = {}
    elapsed = 0.0
    yaw = math.radians(mission.stops[0].yaw_deg)
    points.append(_stationary_point(elapsed, mission.stops[0], yaw))

    def add_hold(stop: MissionStop) -> None:
        nonlocal elapsed
        if stop.hold_s <= 0.0:
            return
        start_time = elapsed
        for local_time in _sample_times(stop.hold_s, mission.sample_period_s)[1:]:
            elapsed = start_time + local_time
            points.append(_stationary_point(elapsed, stop, yaw))

    add_hold(mission.stops[0])
    departures[mission.stops[0].name] = elapsed
    for start, finish in zip(
        mission.stops[:-1], mission.stops[1:], strict=True
    ):
        duration, delta, yaw_delta = _segment_duration(start, finish, mission.limits)
        summaries.append(_summary(start, finish, duration, delta, yaw_delta))
        segment_start = elapsed
        start_yaw = yaw
        for local_time in _sample_times(duration, mission.sample_period_s)[1:]:
            u = local_time / duration
            q, q1, q2, q3 = _smootherstep7(u)
            point_time = segment_start + local_time
            points.append(TrajectoryPoint(
                time=point_time,
                x=start.north_m + delta[0] * q,
                y=start.east_m + delta[1] * q,
                z=start.down_m + delta[2] * q,
                yaw=start_yaw + yaw_delta * q,
                vx=delta[0] * q1 / duration,
                vy=delta[1] * q1 / duration,
                vz=delta[2] * q1 / duration,
                yaw_rate=yaw_delta * q1 / duration,
                ax=delta[0] * q2 / duration**2,
                ay=delta[1] * q2 / duration**2,
                az=delta[2] * q2 / duration**2,
                yaw_acceleration=yaw_delta * q2 / duration**2,
                jx=delta[0] * q3 / duration**3,
                jy=delta[1] * q3 / duration**3,
                jz=delta[2] * q3 / duration**3,
                yaw_jerk=yaw_delta * q3 / duration**3,
            ))
        elapsed = segment_start + duration
        yaw = start_yaw + yaw_delta
        # Replace accumulated floating-point endpoint values with the exact stop.
        points[-1] = _stationary_point(elapsed, finish, yaw)
        arrivals[finish.name] = elapsed
        add_hold(finish)
        departures[finish.name] = elapsed

    if elapsed > mission.safety.max_duration_s + 1e-9:
        raise ValueError(
            f"planned duration {elapsed:.3f}s exceeds max_duration_s "
            f"{mission.safety.max_duration_s:.3f}s"
        )
    if any(
        right.time <= left.time
        for left, right in zip(points, points[1:], strict=False)
    ):
        raise RuntimeError("planner produced non-increasing timestamps")
    _validate_points(points, mission.limits)
    return PlannedMission(
        points=tuple(points),
        stop_arrival_s=arrivals,
        stop_departure_s=departures,
        segments=tuple(summaries),
        duration_s=elapsed,
    )
