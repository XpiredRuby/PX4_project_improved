#!/usr/bin/env python3
"""Tests for stop-by-stop waypoint flight evidence."""
# ruff: noqa: E402

import math
from pathlib import Path
import sys
import unittest

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis"))
from waypoint_mission_audit import build_waypoint_audit


class WaypointMissionAuditTests(unittest.TestCase):
    def inputs(self):
        stops = [
            {"name": "one"},
            {"name": "two"},
            {"name": "three"},
        ]
        arrivals = {"one": 0.0, "two": 2.0, "three": 4.0}
        departures = {"one": 1.0, "two": 3.0, "three": 5.0}
        rows = []
        for name in ("one", "two", "three"):
            arrival = arrivals[name]
            departure = departures[name]
            for sample in range(21):
                time_s = arrival + sample * (departure - arrival) / 20.0
                rows.append({
                    "phase": "TRAJECTORY",
                    "mission_time_s": time_s,
                    "x": 0.1,
                    "y": 0.0,
                    "z": -8.0,
                    "vx": 0.05,
                    "vy": 0.0,
                    "vz": 0.0,
                    "desired_x": 0.0,
                    "desired_y": 0.0,
                    "desired_z": -8.0,
                    "planned_vx": 0.0,
                    "planned_vy": 0.0,
                    "planned_vz": 0.0,
                })
        summary = {
            "stops": stops,
            "stop_arrival_s": arrivals,
            "stop_departure_s": departures,
        }
        manifest = {
            "outcome": "SUCCESS",
            "cleanup_status": "not_required",
            "cleanup_error": None,
            "final_state": {"armed": False, "landed_state": 1},
        }
        return pd.DataFrame(rows), summary, manifest

    def test_all_stops_and_shutdown_pass_with_bounded_evidence(self):
        frame, summary, manifest = self.inputs()
        audit = build_waypoint_audit(frame, summary, manifest)
        self.assertTrue(audit["overall_passed"])
        self.assertEqual(audit["stop_count"], 3)
        self.assertTrue(all(stop["passed"] for stop in audit["stops"]))

    def test_tracking_or_planned_motion_failure_is_not_hidden(self):
        for field, value in (
            ("x", 1.0),
            ("vx", 1.0),
            ("planned_vx", 0.1),
        ):
            frame, summary, manifest = self.inputs()
            frame.loc[frame["mission_time_s"] >= 4.25, field] = value
            with self.subTest(field=field):
                audit = build_waypoint_audit(frame, summary, manifest)
                self.assertFalse(audit["overall_passed"])
                self.assertFalse(audit["stops"][-1]["passed"])

    def test_missing_stop_evidence_and_failed_shutdown_fail(self):
        frame, summary, manifest = self.inputs()
        frame = frame.loc[frame["mission_time_s"] < 4.0]
        manifest["outcome"] = "ABORTED_TO_LAND"
        audit = build_waypoint_audit(frame, summary, manifest)
        self.assertFalse(audit["overall_passed"])
        self.assertEqual(audit["stops"][-1]["evidence_samples"], 0)
        self.assertIsNone(audit["stops"][-1]["position_error_p95_m"])

    def test_invalid_inputs_are_rejected(self):
        frame, summary, manifest = self.inputs()
        for update in (
            {"drop": "x"},
            {"limit": math.nan},
            {"limit": 0.0},
        ):
            with self.subTest(update=update), self.assertRaises(ValueError):
                if "drop" in update:
                    build_waypoint_audit(
                        frame.drop(columns=[update["drop"]]), summary, manifest
                    )
                else:
                    build_waypoint_audit(
                        frame,
                        summary,
                        manifest,
                        position_p95_limit_m=update["limit"],
                    )


if __name__ == "__main__":
    unittest.main()
