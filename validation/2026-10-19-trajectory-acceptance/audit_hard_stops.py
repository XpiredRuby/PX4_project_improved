#!/usr/bin/env python3
"""Independently score actual interior stops in a completed Gazebo rehearsal.

Thresholds declared before evaluating the hard-stop campaign case: <=0.5 m
position error, <=0.2 m/s actual 3-D speed, <=0.1 s mission-clock mismatch.
This checks sampled near-rest behavior; exact zero physical velocity is not
claimed. Simulator truth is read after flight, never used for control.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import unittest

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "analysis"))
from sitl_ground_truth import audit_ground_truth, source_window, ordered_samples, interpolation_coverage  # noqa: E402


def score_stop(clock, physical, velocity, reference_time, expected_position):
    clock = np.asarray(clock, dtype=float)
    if (clock.ndim != 1 or not len(clock) or not np.isfinite(clock).all()
            or np.any(np.diff(clock) < 0)
            or np.asarray(physical).shape != (len(clock), 3)
            or np.asarray(velocity).shape != (len(clock), 3)):
        return {"passed": False, "error": "missing or unordered stop evidence"}
    index = int(np.argmin(np.abs(clock-reference_time)))
    mismatch = float(abs(clock[index]-reference_time))
    error = float(np.linalg.norm(physical[index]-expected_position))
    speed = float(np.linalg.norm(velocity[index]))
    passed = bool(np.isfinite([mismatch, error, speed]).all()
                  and mismatch <= .1 and error <= .5 and speed <= .2)
    return {"passed": passed, "clock_mismatch_s": mismatch,
            "position_error_m": error if np.isfinite(error) else None,
            "actual_speed_m_s": speed if np.isfinite(speed) else None}


def assess(folder):
    from pyulog import ULog
    folder = Path(folder)
    run = folder / "batch" / folder.name
    plan_path = folder / "plan/trajectory.csv"
    intent_path = folder / "plan/intent.json"
    assessment_path = folder / "assessment.json"
    accepted = json.loads(assessment_path.read_text())
    if accepted.get("sampled_rehearsal_passed") is not True:
        raise ValueError("the independent whole-mission assessment must pass first")
    telemetry, = run.glob("research_log_*.csv")
    ulog, = (folder / "runtime").rglob("*.ulg")
    for path in (plan_path, telemetry, ulog):
        if hashlib.sha256(path.read_bytes()).hexdigest() != accepted["evidence_sha256"].get(str(path.relative_to(folder))):
            raise ValueError("stop evidence changed since the whole-mission assessment")
    names = ["vehicle_global_position_groundtruth", "vehicle_local_position",
             "vehicle_local_position_groundtruth"]
    data = {d.name: d.data for d in ULog(str(ulog), message_name_filter_list=names).data_list}
    df = pd.read_csv(telemetry, low_memory=False)
    _, position = audit_ground_truth(df, data[names[0]], data[names[1]])
    samples = df.position_time_boot_ms.to_numpy(dtype=float)*1000.
    times, values, _ = ordered_samples(source_window(data[names[2]], samples), ("vx", "vy", "vz"))
    velocity = np.column_stack([np.interp(samples, times, values[k]) for k in ("vx", "vy", "vz")])
    velocity[~interpolation_coverage(samples, times)] = np.nan
    translation = next(c["detail"]["launch_translation_ned_m"] for c in accepted["checks"]
                       if c["name"] == "Runtime reference follows exact CSV with one translation")
    route = df.phase.eq("TRAJECTORY").to_numpy()
    clock = df.loc[route, "mission_time_s"].to_numpy(dtype=float)
    plan = pd.read_csv(plan_path)
    positions = plan[["x", "y", "z"]].to_numpy(dtype=float)
    derivatives = plan[["vx", "vy", "vz", "ax", "ay", "az", "jx", "jy", "jz"]].to_numpy(dtype=float)
    intent = json.loads(intent_path.read_text())
    result = []
    for point in intent["path"]["points"][1:-1]:
        if not point["hard"]:
            continue
        expected = np.array([point["north_m"], point["east_m"], point["down_m"]])
        match = ((np.linalg.norm(positions-expected, axis=1) <= 1e-6)
                 & (np.max(np.abs(derivatives), axis=1) <= 1e-6))
        indices = np.flatnonzero(match)
        if not len(indices):
            result.append({"point_id": point["point_id"], "passed": False,
                           "error": "no stationary execution reference at the required stop"})
            continue
        stop_time = float(plan.time.iloc[indices[0]])
        result.append({"point_id": point["point_id"], "planned_stop_s": stop_time,
                       **score_stop(clock, position[route], velocity[route], stop_time,
                                    expected+np.asarray(translation))})
    return {"passed": bool(result) and all(c["passed"] for c in result),
            "release_class": "SITL_ONLY", "stops": result,
            "thresholds": {"position_m": .5, "speed_m_s": .2, "clock_s": .1},
            "scope": "sampled near-rest at interior hard points, not continuous stopping proof",
            "implementation_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "evidence_sha256": {str(p.relative_to(folder)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (plan_path, intent_path, assessment_path, telemetry, ulog)}}


class StopEvidenceTests(unittest.TestCase):
    def test_near_rest_with_correct_translation_passes(self):
        point = np.array([[2., 3., -20.]])
        self.assertTrue(score_stop([1.], point, np.zeros((1, 3)), 1., point[0])["passed"])

    def test_reference_stop_cannot_hide_actual_motion_position_error_or_gap(self):
        for position, speed, clock in ((0., .21, 1.), (.51, 0., 1.), (0., 0., 1.11),
                                        (float("nan"), 0., 1.)):
            with self.subTest(position=position, speed=speed, clock=clock):
                self.assertFalse(score_stop([clock], np.array([[position, 0., 0.]]),
                                            np.array([[speed, 0., 0.]]), 1., np.zeros(3))["passed"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case_folder", type=Path, nargs="?")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.self_test:
        unittest.main(argv=[sys.argv[0]])
    elif args.case_folder is None or args.output is None:
        parser.error("case_folder and --output are required")
    else:
        report = assess(args.case_folder)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        print(json.dumps(report, indent=2))
        raise SystemExit(0 if report["passed"] else 1)
