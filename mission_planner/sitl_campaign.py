"""Compare a flown track with a plan. This module does not start Gazebo.

A live PX4/Gazebo campaign needs the simulator, an aircraft configuration, and
an operator. Those are not present just because this repository is checked
out. The comparison below is the offline check a recorded track must pass.
``simulator_status`` never reports that a campaign ran.
"""

from __future__ import annotations

from dataclasses import dataclass
from bisect import bisect_right
import csv
import math
from pathlib import Path
import shutil


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class TrackSample:
    time_s: float
    north_m: float
    east_m: float
    down_m: float
    yaw_rad: float


def _load_track(path: Path) -> tuple[TrackSample, ...]:
    with Path(path).open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        if len(fields) != len(reader.fieldnames or ()):
            raise ValueError("track contains duplicate column names")
        required = {"time", "x", "y", "z", "yaw"}
        if not required <= fields:
            raise ValueError(
                "track is missing columns: " + ", ".join(sorted(required - fields))
            )
        samples = []
        for row in reader:
            samples.append(TrackSample(
                time_s=float(row["time"]),
                north_m=float(row["x"]),
                east_m=float(row["y"]),
                down_m=float(row["z"]),
                yaw_rad=float(row["yaw"]),
            ))
    if len(samples) < 2:
        raise ValueError("track must contain at least two samples")
    if any(
        not math.isfinite(value)
        for sample in samples
        for value in (
            sample.time_s,
            sample.north_m,
            sample.east_m,
            sample.down_m,
            sample.yaw_rad,
        )
    ):
        raise ValueError("track samples must be finite")
    if any(
        right.time_s <= left.time_s
        for left, right in zip(samples, samples[1:], strict=False)
    ):
        raise ValueError("track times must increase")
    return tuple(samples)


def _interpolate(
    plan: tuple[TrackSample, ...], time_s: float,
    times: tuple[float, ...] | None = None,
) -> TrackSample:
    if time_s < plan[0].time_s - 1e-9 or time_s > plan[-1].time_s + 1e-9:
        raise ValueError("flown sample lies outside the planned duration")
    time_s = min(max(time_s, plan[0].time_s), plan[-1].time_s)
    if times is None:
        times = tuple(sample.time_s for sample in plan)
    index = min(max(bisect_right(times, time_s) - 1, 0), len(plan) - 2)
    for left, right in ((plan[index], plan[index + 1]),):
        if left.time_s <= time_s <= right.time_s:
            span = right.time_s - left.time_s
            fraction = 0.0 if span == 0.0 else (time_s - left.time_s) / span
            # Planner CSV yaw is unwrapped. Wrapping its increment would erase
            # intentional full rotations on a sparse reference track.
            yaw = left.yaw_rad + fraction * (right.yaw_rad - left.yaw_rad)
            return TrackSample(
                time_s=time_s,
                north_m=left.north_m + fraction * (right.north_m - left.north_m),
                east_m=left.east_m + fraction * (right.east_m - left.east_m),
                down_m=left.down_m + fraction * (right.down_m - left.down_m),
                yaw_rad=yaw,
            )
    raise ValueError("flown sample lies outside the planned duration")


def compare_flown_track(
    plan_path: Path,
    flown_path: Path,
    *,
    maximum_position_error_m: float,
    maximum_yaw_error_rad: float,
    maximum_sample_gap_s: float = 0.25,
    endpoint_tolerance_s: float = 1e-6,
) -> dict:
    """Score a mission-aligned recorded track, including coverage and gaps.

    Times must share the plan's mission clock; trim takeoff/landing separately.
    The 0.25 s gap is an explicit offline default, not an aircraft limit. This
    sampled comparison does not certify between-sample errors, actuator limits,
    touchdown, or completion of a Gazebo campaign.
    """
    for name, value in (
        ("maximum_position_error_m", maximum_position_error_m),
        ("maximum_yaw_error_rad", maximum_yaw_error_rad),
        ("maximum_sample_gap_s", maximum_sample_gap_s),
    ):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0.0):
            raise ValueError(f"{name} must be finite and positive")
    if (isinstance(endpoint_tolerance_s, bool)
            or not isinstance(endpoint_tolerance_s, (int, float))
            or not math.isfinite(endpoint_tolerance_s)
            or endpoint_tolerance_s < 0.0):
        raise ValueError("endpoint_tolerance_s must be finite and nonnegative")
    plan = _load_track(plan_path)
    flown = _load_track(flown_path)
    if endpoint_tolerance_s >= (plan[-1].time_s - plan[0].time_s) / 2.0:
        raise ValueError("endpoint tolerance must be below half the planned duration")
    complete = (
        abs(flown[0].time_s - plan[0].time_s) <= endpoint_tolerance_s
        and abs(flown[-1].time_s - plan[-1].time_s) <= endpoint_tolerance_s
    )
    largest_gap = max(
        right.time_s - left.time_s
        for left, right in zip(flown, flown[1:], strict=False)
    )
    gaps_passed = largest_gap <= maximum_sample_gap_s + 1e-12
    times = tuple(sample.time_s for sample in plan)
    position_error = 0.0
    yaw_error = 0.0
    for sample in flown:
        if (sample.time_s < plan[0].time_s - endpoint_tolerance_s
                or sample.time_s > plan[-1].time_s + endpoint_tolerance_s):
            raise ValueError("flown sample lies outside the planned duration")
        query_time = min(max(sample.time_s, plan[0].time_s), plan[-1].time_s)
        reference = _interpolate(plan, query_time, times)
        position_error = max(
            position_error,
            math.dist(
                (sample.north_m, sample.east_m, sample.down_m),
                (reference.north_m, reference.east_m, reference.down_m),
            ),
        )
        yaw_error = max(yaw_error, abs(_wrap(sample.yaw_rad - reference.yaw_rad)))
    accepted = (
        complete and gaps_passed
        and position_error <= maximum_position_error_m + 1e-12
        and yaw_error <= maximum_yaw_error_rad + 1e-12
    )
    return {
        "schema_version": 1,
        "release_class": "SITL_ONLY",
        "accepted": accepted,
        "position_error_m": position_error,
        "yaw_error_rad": yaw_error,
        "maximum_position_error_m": maximum_position_error_m,
        "maximum_yaw_error_rad": maximum_yaw_error_rad,
        "coverage_passed": complete,
        "sample_gaps_passed": gaps_passed,
        "maximum_sample_gap_s": maximum_sample_gap_s,
        "observed_maximum_sample_gap_s": largest_gap,
        "endpoint_tolerance_s": endpoint_tolerance_s,
        "plan_duration_s": plan[-1].time_s - plan[0].time_s,
        "track_duration_s": flown[-1].time_s - flown[0].time_s,
        "verification_scope": "sampled tracking only; not continuous feasibility or lifecycle",
        "clearance": "not a clearance to fly",
        "campaign_ran": False,
    }


def simulator_status() -> dict:
    """Report whether a simulator binary is installed. This does not launch it."""
    return {
        "schema_version": 1,
        "release_class": "SITL_ONLY",
        "gazebo_on_path": any(shutil.which(name) for name in ("gz", "gazebo", "ign")),
        "px4_on_path": shutil.which("px4") is not None,
        "campaign_ran": False,
        "clearance": "not a clearance to fly",
    }
