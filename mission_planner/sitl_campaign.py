"""Compare a flown track with a plan. This module does not start Gazebo.

A live PX4/Gazebo campaign needs the simulator, an aircraft configuration, and
an operator. Those are not present just because this repository is checked
out. The comparison below is the offline check a recorded track must pass.
``simulator_status`` never reports that a campaign ran.
"""

from __future__ import annotations

from dataclasses import dataclass
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


def _interpolate(plan: tuple[TrackSample, ...], time_s: float) -> TrackSample:
    if time_s < plan[0].time_s - 1e-9 or time_s > plan[-1].time_s + 1e-9:
        raise ValueError("flown sample lies outside the planned duration")
    time_s = min(max(time_s, plan[0].time_s), plan[-1].time_s)
    for left, right in zip(plan, plan[1:], strict=False):
        if left.time_s <= time_s <= right.time_s:
            span = right.time_s - left.time_s
            fraction = 0.0 if span == 0.0 else (time_s - left.time_s) / span
            yaw = left.yaw_rad + fraction * _wrap(right.yaw_rad - left.yaw_rad)
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
) -> dict:
    """Return the position and yaw error of a recorded track against the plan."""
    if maximum_position_error_m <= 0.0 or maximum_yaw_error_rad <= 0.0:
        raise ValueError("tracking thresholds must be positive")
    plan = _load_track(plan_path)
    flown = _load_track(flown_path)
    position_error = 0.0
    yaw_error = 0.0
    for sample in flown:
        reference = _interpolate(plan, sample.time_s)
        position_error = max(
            position_error,
            math.dist(
                (sample.north_m, sample.east_m, sample.down_m),
                (reference.north_m, reference.east_m, reference.down_m),
            ),
        )
        yaw_error = max(yaw_error, abs(_wrap(sample.yaw_rad - reference.yaw_rad)))
    accepted = (
        position_error <= maximum_position_error_m + 1e-12
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
