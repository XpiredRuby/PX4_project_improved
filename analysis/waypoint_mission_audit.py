#!/usr/bin/env python3
"""Audit that every planned waypoint hold was reached in a completed run."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd


REQUIRED_COLUMNS = {
    "phase",
    "mission_time_s",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "desired_x",
    "desired_y",
    "desired_z",
    "planned_vx",
    "planned_vy",
    "planned_vz",
}


def _check(name: str, passed: bool, detail: str) -> dict[str, object]:
    return {"name": name, "passed": bool(passed), "detail": detail}


def _finite_frame(frame: pd.DataFrame, columns: list[str]) -> pd.DataFrame:
    converted = frame[columns].apply(pd.to_numeric, errors="coerce")
    return converted.replace([np.inf, -np.inf], np.nan).dropna()


def build_waypoint_audit(
    frame: pd.DataFrame,
    summary: dict,
    manifest: dict,
    *,
    position_p95_limit_m: float = 0.75,
    speed_p95_limit_m_s: float = 0.40,
    planned_speed_limit_m_s: float = 1e-6,
) -> dict:
    missing = sorted(REQUIRED_COLUMNS - set(frame.columns))
    if missing:
        raise ValueError("flight log missing columns: " + ", ".join(missing))
    for name, value in (
        ("position_p95_limit_m", position_p95_limit_m),
        ("speed_p95_limit_m_s", speed_p95_limit_m_s),
        ("planned_speed_limit_m_s", planned_speed_limit_m_s),
    ):
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ValueError(f"{name} must be numeric")
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")

    stops = summary.get("stops")
    arrivals = summary.get("stop_arrival_s")
    departures = summary.get("stop_departure_s")
    if not isinstance(stops, list) or not stops:
        raise ValueError("planning summary has no stops")
    if not isinstance(arrivals, dict) or not isinstance(departures, dict):
        raise ValueError("planning summary lacks stop timing maps")

    trajectory = frame.loc[frame["phase"].astype(str) == "TRAJECTORY"].copy()
    numeric_names = sorted(REQUIRED_COLUMNS - {"phase"})
    numeric = _finite_frame(trajectory, numeric_names)
    trajectory = trajectory.loc[numeric.index].copy()
    for name in numeric_names:
        trajectory[name] = numeric[name]

    checks = []
    stop_results = []
    prior_arrival = -math.inf
    for index, stop in enumerate(stops):
        if not isinstance(stop, dict) or not isinstance(stop.get("name"), str):
            raise ValueError(f"planning summary stop {index} is invalid")
        name = stop["name"]
        if name not in arrivals or name not in departures:
            raise ValueError(f"planning summary lacks timing for stop {name}")
        arrival = float(arrivals[name])
        departure = float(departures[name])
        if not (
            math.isfinite(arrival)
            and math.isfinite(departure)
            and prior_arrival < arrival <= departure
        ):
            raise ValueError(f"planning summary timing for stop {name} is invalid")
        prior_arrival = arrival
        hold_duration = departure - arrival
        if hold_duration <= 0.0:
            half_width = 0.075
            window = trajectory.loc[
                (trajectory["mission_time_s"] >= arrival - half_width)
                & (trajectory["mission_time_s"] <= arrival + half_width)
            ]
        else:
            evidence_duration = min(0.75, max(0.25, hold_duration / 2.0))
            window = trajectory.loc[
                (trajectory["mission_time_s"] >= departure - evidence_duration)
                & (trajectory["mission_time_s"] <= departure + 1e-6)
            ]

        evidence_count = len(window)
        if evidence_count:
            position_error = np.sqrt(
                (window["x"] - window["desired_x"]) ** 2
                + (window["y"] - window["desired_y"]) ** 2
                + (window["z"] - window["desired_z"]) ** 2
            )
            speed = np.sqrt(
                window["vx"] ** 2
                + window["vy"] ** 2
                + window["vz"] ** 2
            )
            planned_speed = np.sqrt(
                window["planned_vx"] ** 2
                + window["planned_vy"] ** 2
                + window["planned_vz"] ** 2
            )
            position_p95 = float(np.percentile(position_error, 95))
            speed_p95 = float(np.percentile(speed, 95))
            planned_speed_max = float(planned_speed.max())
        else:
            position_p95 = speed_p95 = planned_speed_max = math.inf

        reached = (
            evidence_count >= 3
            and position_p95 <= position_p95_limit_m
            and speed_p95 <= speed_p95_limit_m_s
            and planned_speed_max <= planned_speed_limit_m_s
        )
        stop_result = {
            "index": index,
            "name": name,
            "arrival_s": arrival,
            "departure_s": departure,
            "evidence_samples": evidence_count,
            "position_error_p95_m": (
                position_p95 if math.isfinite(position_p95) else None
            ),
            "speed_p95_m_s": speed_p95 if math.isfinite(speed_p95) else None,
            "planned_speed_max_m_s": (
                planned_speed_max if math.isfinite(planned_speed_max) else None
            ),
            "passed": reached,
        }
        stop_results.append(stop_result)
        checks.append(_check(
            f"Stop {index + 1} reached: {name}",
            reached,
            (
                f"samples={evidence_count}, position_p95={position_p95:.3f}m, "
                f"speed_p95={speed_p95:.3f}m/s, "
                f"planned_speed_max={planned_speed_max:.6f}m/s"
            ),
        ))

    final_departure = float(departures[stops[-1]["name"]])
    maximum_mission_time = (
        float(trajectory["mission_time_s"].max()) if len(trajectory) else -math.inf
    )
    route_complete = maximum_mission_time >= final_departure - 0.1
    checks.append(_check(
        "Entire waypoint route consumed",
        route_complete,
        f"mission_time_max={maximum_mission_time:.3f}s, final={final_departure:.3f}s",
    ))
    shutdown_complete = (
        manifest.get("outcome") == "SUCCESS"
        and manifest.get("cleanup_status") == "not_required"
        and manifest.get("cleanup_error") is None
        and manifest.get("final_state", {}).get("armed") is False
        and manifest.get("final_state", {}).get("landed_state") == 1
    )
    checks.append(_check(
        "Successful touchdown, disarm, and shutdown recorded",
        shutdown_complete,
        (
            f"outcome={manifest.get('outcome')}, "
            f"cleanup={manifest.get('cleanup_status')}"
        ),
    ))
    return {
        "schema_version": 1,
        "limits": {
            "position_error_p95_m": position_p95_limit_m,
            "speed_p95_m_s": speed_p95_limit_m_s,
            "planned_speed_max_m_s": planned_speed_limit_m_s,
        },
        "stop_count": len(stops),
        "stops": stop_results,
        "checks": checks,
        "overall_passed": all(check["passed"] for check in checks),
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    frame = pd.read_csv(args.log, low_memory=False)
    summary = json.loads(args.summary.read_text(encoding="utf-8"))
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    audit = build_waypoint_audit(frame, summary, manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(audit, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(audit, indent=2, sort_keys=True, allow_nan=False))
    if not audit["overall_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
