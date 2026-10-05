#!/usr/bin/env python3
"""Compile a strict local-NED waypoint mission into controller CSV."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

from export import save_trajectory_csv
from waypoint_mission import load_mission, plan_mission


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--summary",
        type=Path,
        help="Optional machine-readable planning summary",
    )
    return parser.parse_args(argv)


def compile_plan(plan_path: Path, output_path: Path, summary_path: Path | None = None):
    mission = load_mission(plan_path)
    planned = plan_mission(mission)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    save_trajectory_csv(output_path, planned.points)
    trajectory_sha256 = hashlib.sha256(output_path.read_bytes()).hexdigest()
    summary = {
        "schema_version": 1,
        "coordinate_frame": mission.frame,
        "source_plan": str(plan_path),
        "source_plan_sha256": hashlib.sha256(plan_path.read_bytes()).hexdigest(),
        "trajectory": str(output_path),
        "trajectory_sha256": trajectory_sha256,
        "point_count": len(planned.points),
        "duration_s": planned.duration_s,
        "stop_arrival_s": planned.stop_arrival_s,
        "stop_departure_s": planned.stop_departure_s,
        "limits": asdict(mission.limits),
        "safety": asdict(mission.safety),
        "stops": [asdict(stop) for stop in mission.stops],
        "segments": [asdict(segment) for segment in planned.segments],
    }
    if summary_path is not None:
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    return summary


def main(argv=None) -> None:
    args = parse_args(argv)
    summary = compile_plan(args.plan, args.output, args.summary)
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
