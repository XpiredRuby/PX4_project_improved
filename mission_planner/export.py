"""Deterministic adapter from planner candidates to the existing controller CSV."""

from __future__ import annotations

import csv
from pathlib import Path

from .retiming import CandidateTrajectory


FIELDS = (
    "time",
    "x",
    "y",
    "z",
    "yaw",
    "vx",
    "vy",
    "vz",
    "yaw_rate",
    "ax",
    "ay",
    "az",
    "jx",
    "jy",
    "jz",
    "yaw_acceleration",
    "yaw_jerk",
    "planner_contract_version",
)


def save_controller_csv(
    path: str | Path,
    candidate: CandidateTrajectory,
) -> None:
    """Write every controller-consumed position and derivative explicitly."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
            writer.writeheader()
            for point in candidate.points:
                writer.writerow({
                    "time": point.time_s,
                    "x": point.north_m,
                    "y": point.east_m,
                    "z": point.down_m,
                    "yaw": point.yaw_rad,
                    "vx": point.north_velocity_m_s,
                    "vy": point.east_velocity_m_s,
                    "vz": point.down_velocity_m_s,
                    "yaw_rate": point.yaw_rate_rad_s,
                    "ax": point.north_acceleration_m_s2,
                    "ay": point.east_acceleration_m_s2,
                    "az": point.down_acceleration_m_s2,
                    "jx": point.north_jerk_m_s3,
                    "jy": point.east_jerk_m_s3,
                    "jz": point.down_jerk_m_s3,
                    "yaw_acceleration": point.yaw_acceleration_rad_s2,
                    "yaw_jerk": point.yaw_jerk_rad_s3,
                    "planner_contract_version": 1,
                })
        temporary.replace(destination)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
