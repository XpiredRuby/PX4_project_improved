"""Look-at yaw, fly-through points, maps, rotors, and SITL review records."""

from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from mission_planner.acknowledgement import (
    issue_sitl_acknowledgement,
    load_sitl_acknowledgement,
)
from mission_planner.contracts import MissionIntent, PathIntent
from mission_planner.geometry import build_smooth_path
from mission_planner.mapping import (
    OccupancyMap,
    clearance_margin_m,
    corridor_polyline,
)
from mission_planner.plan_interview import intent_from_interview
from mission_planner.retiming import retime_stop_path
from mission_planner.rotors import rotor_duration_s, yaw_torque_available_nm
from mission_planner.sitl_campaign import compare_flown_track, simulator_status
from mission_planner.verification import verify_candidate_trajectory
from tests.test_planner_retiming import geometry, motion, vehicle
from tests.test_planner_verification import environment


def _frame():
    return {
        "name": "LOCAL_NED",
        "units": "m",
        "origin_id": "launch-001",
        "altitude_datum": "launch",
    }


def _map(occupied: str, *, rows: int, columns: int, localization: float = 0.0):
    return OccupancyMap.from_dict({
        "schema_version": 1,
        "map_id": "yard-v1",
        "frame": _frame(),
        "origin_north_m": 0.0,
        "origin_east_m": 0.0,
        "resolution_m": 1.0,
        "rows": rows,
        "columns": columns,
        "occupied": occupied,
        "localization_radius_m": localization,
    })


class PlannerFinishTests(unittest.TestCase):
    def test_exact_fly_through_lies_on_the_curve(self):
        path = PathIntent.from_dict({
            "kind": "polyline",
            "closure": "open_stop",
            "maximum_deviation_m": 0.75,
            "points": [
                {
                    "point_id": "start",
                    "north_m": 0.0,
                    "east_m": 0.0,
                    "down_m": -8.0,
                    "hard": True,
                    "exact": False,
                },
                {
                    "point_id": "gate",
                    "north_m": 6.0,
                    "east_m": 0.0,
                    "down_m": -8.0,
                    "hard": False,
                    "exact": True,
                },
                {
                    "point_id": "finish",
                    "north_m": 6.0,
                    "east_m": 6.0,
                    "down_m": -8.0,
                    "hard": True,
                    "exact": False,
                },
            ],
        })
        plan = build_smooth_path(
            path,
            control_spacing_m=1.0,
            verification_resolution_m=0.05,
        )
        gate = np.asarray([6.0, 0.0, -8.0])
        grid = np.linspace(0.0, 1.0, 200)
        distance = math.inf
        for _ in range(12):
            distances = [
                float(np.linalg.norm(np.asarray(plan.curve.evaluate(float(value))) - gate))
                for value in grid
            ]
            index = int(np.argmin(distances))
            distance = distances[index]
            center = float(grid[index])
            half = float(grid[1] - grid[0])
            grid = np.linspace(max(0.0, center - half), min(1.0, center + half), 40)
        self.assertLess(distance, 1e-8)

    def test_look_at_heading_is_certified_and_verified(self):
        target = (0.0, 40.0, -8.0)
        plan = retime_stop_path(
            geometry(),
            motion(yaw_mode="look_at", look_at=target),
            vehicle(),
        )
        self.assertLessEqual(plan.summary.heading_error_bound_rad, math.radians(2.0))
        report = verify_candidate_trajectory(
            geometry(),
            plan,
            motion(yaw_mode="look_at", look_at=target),
            vehicle(),
            environment(),
            maximum_deviation_m=0.75,
        )
        self.assertTrue(report.accepted)

    def test_corridor_avoids_a_wall_and_a_crossing_path_does_not(self):
        rows = columns = 10
        cells = ["0"] * (rows * columns)
        for row in range(6):
            cells[row * columns + 4] = "1"
        occupancy = _map("".join(cells), rows=rows, columns=columns)
        route = corridor_polyline(
            occupancy,
            start=(0.2, 0.2),
            goal=(0.2, 8.2),
            clearance_m=0.0,
            deviation_m=0.0,
        )
        self.assertGreater(max(point[0] for point in route), 5.0)
        blocked = PathIntent.from_dict({
            "kind": "polyline",
            "closure": "open_stop",
            "maximum_deviation_m": 0.2,
            "points": [
                {
                    "point_id": "start",
                    "north_m": 0.2,
                    "east_m": 0.2,
                    "down_m": -8.0,
                    "hard": False,
                    "exact": False,
                },
                {
                    "point_id": "finish",
                    "north_m": 0.2,
                    "east_m": 8.2,
                    "down_m": -8.0,
                    "hard": False,
                    "exact": False,
                },
            ],
        })
        curve = build_smooth_path(
            blocked,
            control_spacing_m=0.5,
            verification_resolution_m=0.05,
        ).curve
        self.assertLess(clearance_margin_m(curve, occupancy, clearance_m=0.0), 0.0)
        raw = {
            "schema_version": 1,
            "mission_id": "corridor-v1",
            "pattern": "corridor",
            "altitude_m": 8.0,
            "maximum_deviation_m": 0.25,
            "maximum_speed_m_s": 1.0,
            "yaw_mode": "fixed",
            "fixed_yaw_deg": 0.0,
            "time_mode": "minimize",
            "vehicle_model_id": "px4-x500-sitl-v1",
            "environment_id": "yard-v1",
            "corridor": {"goal_north_m": 0.2, "goal_east_m": 8.2},
        }
        intent, loops = intent_from_interview(
            raw, occupancy=occupancy, clearance_m=0.0
        )
        parsed = MissionIntent.from_dict(intent)
        self.assertEqual(loops, 1)
        self.assertEqual(parsed.path.points[0].north_m, 0.0)
        self.assertEqual(parsed.path.points[0].east_m, 0.0)
        self.assertGreater(
            max(point.north_m for point in parsed.path.points),
            5.0,
        )

    def test_paired_rotors_lose_yaw_torque_as_thrust_approaches_the_ceiling(self):
        self.assertAlmostEqual(
            yaw_torque_available_nm(
                20.0,
                rotor_thrust_minimum_n=0.0,
                rotor_thrust_maximum_n=10.0,
                yaw_torque_per_thrust_m=0.05,
            ),
            1.0,
        )
        self.assertEqual(
            yaw_torque_available_nm(
                40.0,
                rotor_thrust_minimum_n=0.0,
                rotor_thrust_maximum_n=10.0,
                yaw_torque_per_thrust_m=0.05,
            ),
            0.0,
        )
        duration, name = rotor_duration_s(
            acceleration_numerator=0.0,
            yaw_acceleration_numerator=50.0,
            mass_kg=2.0,
            yaw_inertia_kg_m2=0.04,
            rotor_thrust_minimum_n=0.0,
            rotor_thrust_maximum_n=10.0,
            yaw_torque_per_thrust_m=1e-4,
        )
        self.assertEqual(name, "rotor")
        self.assertGreater(duration, 1.0)

    def test_acknowledgement_rejects_a_stale_or_mismatched_review(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trajectory = root / "trajectory.csv"
            trajectory.write_text("time,x\n0,0\n", encoding="utf-8")
            digest = hashlib.sha256(trajectory.read_bytes()).hexdigest()
            candidate = "a" * 64
            report = "b" * 64
            summary = {
                "schema_version": 1,
                "release_class": "SITL_ONLY",
                "controller_trajectory_sha256": digest,
                "candidate_trajectory_sha256": candidate,
                "verification_report_sha256": report,
            }
            summary_path = root / "summary.json"
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            expiry = (datetime.now(timezone.utc) + timedelta(hours=1)).isoformat()
            record = issue_sitl_acknowledgement(
                controller_trajectory_sha256=digest,
                candidate_trajectory_sha256=candidate,
                verification_report_sha256=report,
                expires_at=expiry,
                acknowledged_by="sitl-reviewer",
            )
            acknowledgement = root / "ack.json"
            acknowledgement.write_text(json.dumps(record), encoding="utf-8")
            self.assertEqual(
                load_sitl_acknowledgement(
                    acknowledgement, summary_path, trajectory
                )["statement"],
                "not a clearance to fly",
            )
            record["expires_at"] = (
                datetime.now(timezone.utc) - timedelta(minutes=1)
            ).isoformat()
            acknowledgement.write_text(json.dumps(record), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "expired"):
                load_sitl_acknowledgement(acknowledgement, summary_path, trajectory)

    def test_track_comparison_does_not_claim_a_simulator_campaign(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = root / "plan.csv"
            flown = root / "flown.csv"
            header = "time,x,y,z,yaw\n"
            plan.write_text(header + "0,0,0,-8,0\n2,2,0,-8,0\n", encoding="utf-8")
            flown.write_text(header + "0,0,0,-8,0\n2,2.1,0,-8,0\n", encoding="utf-8")
            report = compare_flown_track(
                plan,
                flown,
                maximum_position_error_m=0.2,
                maximum_yaw_error_rad=0.1,
            )
            self.assertTrue(report["accepted"])
            self.assertFalse(report["campaign_ran"])
            self.assertIn("not a clearance to fly", report["clearance"])
            flown.write_text(header + "0,0,0,-8,0\n2,4,0,-8,0\n", encoding="utf-8")
            drifted = compare_flown_track(
                plan,
                flown,
                maximum_position_error_m=0.2,
                maximum_yaw_error_rad=0.1,
            )
            self.assertFalse(drifted["accepted"])
        status = simulator_status()
        self.assertFalse(status["campaign_ran"])
        self.assertIn("not a clearance to fly", status["clearance"])


if __name__ == "__main__":
    unittest.main()
