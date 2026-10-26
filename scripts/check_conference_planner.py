#!/usr/bin/env python3
"""Repeated arbitrary-input/reference audits. Never starts a flight transport."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission_planner.preview import preview  # noqa: E402
from analysis.reference_audit import audit_reference  # noqa: E402
from mission_planner.stop_turn_execution import audit_stop_turn_artifacts  # noqa: E402


def request(points, speed=1.0, height=20.0, cap=3.0, **motion):
    raw = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
    base = raw["path"]["points"][0]
    raw["path"].update(
        closure="open_stop",
        points=[
            dict(
                base,
                point_id=f"p{i}",
                north_m=n,
                east_m=e,
                down_m=-height,
                hard=False,
                exact=False,
            )
            for i, (n, e) in enumerate(points)
        ],
    )
    raw["motion"].update(
        yaw_mode="tangent",
        fixed_yaw_deg=None,
        target_speed_m_s=speed,
        maximum_speed_m_s=cap,
        **motion,
    )
    return dict(source="drawing", document=raw, planning_mode="closest")


def cases():
    for speed in (0.5, 1.0, 1.25, 1.5, 2.0, 3.0, 5.0):
        yield f"straight-speed-{speed:g}", request([(0, 0), (35, 0)], speed=speed)
    yield "explicit-low-cap", request([(0, 0), (35, 0)], speed=2.0, cap=1.25)
    for height in (0.1, 1.0, 30.0, 50.0):
        yield f"height-{height:g}", request([(0, 0), (8, 0)], height=height)
    geometries = {
        "reversal": [(0, 0), (10, 0), (0, 0)],
        "five-stops": [(0, 0), (0, 8), (8, 8), (8, 0), (3, 0), (3, 4)],
        "duplicate-hover": [(0, 0)] * 3,
        "tiny-detail": [(0, 0), (0.000001, 0), (0, 0.000001)],
        "figure-eight": [
            (12 * math.sin(t), 6 * math.sin(2 * t))
            for t in [i * math.tau / 64 for i in range(65)]
        ],
        "dense-10000": [(0, 0)]
        + [
            (15 * math.sin(i * 0.001), 8 * math.sin(i * 0.002)) for i in range(1, 10000)
        ],
        "outside-boundary": [(0, 0), (90, 80), (-70, -60), (0, 0)],
        "vin": [
            (0, 0),
            (8, 0),
            (0, 3),
            (8, 6),
            (1, 8),
            (7, 8),
            (8, 8),
            (1, 8),
            (1, 11),
            (7, 11),
            (7, 14),
            (1, 14),
        ],
    }
    for name, points in geometries.items():
        value = request(points, speed=2.0)
        if name == "five-stops":
            value["document"]["path"]["maximum_deviation_m"] = 0.0
            for p in value["document"]["path"]["points"]:
                p["hard"] = True
        yield name, value
    yield (
        "impossible-deadline-adjusted",
        request([(0, 0), (12, 0)], requested_duration_s=0.01, time_mode="maximum"),
    )
    for laps in (1, 2):
        yield (
            f"circle-{laps}-lap",
            dict(
                source="interview",
                planning_mode="closest",
                document=dict(
                    schema_version=1,
                    mission_id="circle-regression",
                    vehicle_model_id="px4-x500-sitl-v1",
                    environment_id="empty-world-v1",
                    pattern="circle",
                    altitude_m=20.0,
                    maximum_deviation_m=0.5,
                    maximum_speed_m_s=3.0,
                    target_speed_m_s=2.0,
                    yaw_mode="tangent",
                    fixed_yaw_deg=None,
                    time_mode="minimize",
                    requested_duration_s=None,
                    circle=dict(center_north_m=8.0, center_east_m=0.0),
                    loop_count=laps,
                ),
            ),
        )


def run(output, repeats):
    output.mkdir(parents=True, exist_ok=False)
    sources = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
               for directory in ('mission_planner', 'analysis', 'scripts', 'config/planner')
               for p in sorted((ROOT / directory).rglob('*'))
               if p.is_file() and p.suffix in ('.py', '.json')}
    identity = dict(
        source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        source_files=sources,
        source_fingerprint=hashlib.sha256(json.dumps(sources, sort_keys=True).encode()).hexdigest(),
        note='Commit plus exact working-source hashes; may include uncommitted changes.'
    )
    results = []
    for name, payload in cases():
        for repeat in range(repeats):
            started = time.perf_counter()
            item = dict(case=name, repeat=repeat + 1)
            try:
                value = preview(payload)
                if not value["accepted"]:
                    raise ValueError(value.get("error", "not accepted"))
                rows = list(csv.DictReader(io.StringIO(value["trajectory_csv"])))
                full = list(
                    csv.DictReader(io.StringIO(value["mission"]["trajectory_csv"]))
                )
                assert all(
                    float(b["time"]) > float(a["time"])
                    for a, b in zip(full, full[1:], strict=False)
                )
                assert all(math.isfinite(float(row[k])) for row in rows for k in row)
                assert all(
                    abs(float(r[k])) < 1e-8
                    for r in (full[0], full[-1])
                    for k in (
                        "x",
                        "y",
                        "z",
                        "vx",
                        "vy",
                        "vz",
                        "ax",
                        "ay",
                        "az",
                        "jx",
                        "jy",
                        "jz",
                    )
                )
                digest = hashlib.sha256(value["trajectory_csv"].encode()).hexdigest()
                assert digest == value["summary"]["controller_trajectory_sha256"]
                item.update(
                    passed=True,
                    trajectory_sha256=digest,
                    route_rows=len(rows),
                    mission_rows=len(full),
                    speed=value["summary"]["speed_preferences"],
                    route_duration_s=value["summary"]["retiming"][
                        "selected_duration_s"
                    ],
                    method=value["planning_adjustments"]["method"],
                    adjustments=value["planning_adjustments"]["adjustments"],
                    original_rounding_met=value["planning_adjustments"][
                        "original_rounding_met"
                    ],
                )
                item["independent_reference_audit"] = audit_reference(
                    value["trajectory_csv"], value["vehicle_model"], value["environment"]
                )
                if not item["independent_reference_audit"]["passed"]:
                    raise ValueError("Independent exported-reference diagnostic failed")
                if value["preview_settings"].get("planner_backend") == "stop_turn":
                    folder = output / (name + f"-{repeat + 1}")
                    folder.mkdir()
                    (folder / "trajectory.csv").write_text(value["trajectory_csv"])
                    for key in (
                        "summary",
                        "verification",
                        "intent",
                        "vehicle_model",
                        "environment",
                    ):
                        (folder / (key + ".json")).write_text(
                            json.dumps(value[key], allow_nan=False)
                        )
                    item["diverse_audit"] = audit_stop_turn_artifacts(
                        folder, for_execution=False
                    )
                if (
                    repeat
                    and item["trajectory_sha256"] != results[-1]["trajectory_sha256"]
                ):
                    raise ValueError("Repeated native compilation is not deterministic")
            except Exception as exc:
                item.update(passed=False, error=f"{type(exc).__name__}: {exc}")
            item["elapsed_s"] = time.perf_counter() - started
            results.append(item)
            print(name, repeat + 1, item["passed"], item.get("error", ""), flush=True)
            (output / "results.json").write_text(
                json.dumps(
                    dict(
                        scope="offline references only",
                        simulator_ran=False,
                        source_identity=identity,
                        cases=results,
                    ),
                    indent=2,
                    allow_nan=False,
                )
                + "\n"
            )
    return all(r["passed"] for r in results)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, choices=range(1, 4), default=2)
    args = parser.parse_args()
    sys.exit(0 if run(args.output, args.repeats) else 1)
