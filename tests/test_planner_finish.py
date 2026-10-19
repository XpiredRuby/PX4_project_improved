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
from mission_planner.explain import PlanningError
from mission_planner.plan_drawn_path import compile_drawn_path
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

    def test_circle_exit_flies_the_seam_and_stops_at_the_exit(self):
        raw = _circle_interview(loop_count=2, exit=[{"north_m": 0.0, "east_m": -6.0}])
        intent, loops = intent_from_interview(raw)
        self.assertEqual(loops, 1)
        parsed = MissionIntent.from_dict(intent)
        self.assertEqual(parsed.path.closure, "open_stop")
        points = parsed.path.points
        self.assertEqual((points[-1].north_m, points[-1].east_m), (0.0, -6.0))
        origin = [
            index
            for index, point in enumerate(points)
            if math.hypot(point.north_m, point.east_m) < 1e-6
        ]
        self.assertEqual(len(origin), 3)
        self.assertEqual(origin[0], 0)
        self.assertFalse(points[0].exact)
        for index in origin[1:]:
            self.assertTrue(points[index].exact)
            self.assertFalse(points[index].hard)
            self.assertGreater(
                max(point.north_m for point in points[origin[origin.index(index) - 1]:index]),
                7.0,
            )
        self.assertEqual(points[origin[-1] + 1].point_id, "exit-0")
        self.assertEqual(parsed.motion.yaw_mode, "fixed")

        one_lap, one_loops = intent_from_interview(
            _circle_interview(exit=[{"north_m": 0.0, "east_m": -6.0}])
        )
        flown = MissionIntent.from_dict(one_lap)
        self.assertEqual(one_loops, 1)
        self.assertEqual(
            sum(point.exact for point in flown.path.points[1:-1]),
            1,
        )
        plan = build_smooth_path(
            flown.path,
            control_spacing_m=1.0,
            verification_resolution_m=0.1,
        )
        gate = np.asarray([0.0, 0.0, -8.0])
        grid = np.linspace(0.2, 0.98, 200)
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
            grid = np.linspace(
                max(0.2, center - half),
                min(0.98, center + half),
                40,
            )
        self.assertLess(distance, 1e-8)
        timed = retime_stop_path(
            plan,
            flown.motion,
            vehicle(),
            sample_period_s=0.1,
        )
        self.assertEqual(timed.summary.segment_count, 1)
        self.assertEqual(timed.summary.loop_count, 1)
        start, finish = timed.points[0], timed.points[-1]
        self.assertLess(abs(start.north_m) + abs(start.east_m), 1e-6)
        self.assertLess(_horizontal_speed(start), 1e-6)
        self.assertAlmostEqual(finish.north_m, 0.0, places=6)
        self.assertAlmostEqual(finish.east_m, -6.0, places=6)
        self.assertLess(_horizontal_speed(finish), 1e-6)
        self.assertGreater(finish.time_s, 1.0)
        far = [point for point in timed.points if point.north_m > 6.0]
        self.assertGreater(
            max(_horizontal_speed(point) for point in far),
            0.2,
        )
        interior = [
            point
            for point in timed.points
            if 1.0 < point.time_s < finish.time_s - 1.0
        ]
        seam = min(interior, key=lambda point: math.hypot(point.north_m, point.east_m))
        self.assertLess(math.hypot(seam.north_m, seam.east_m), 1.0)
        self.assertGreater(_horizontal_speed(seam), 1e-3)

    def test_explicit_heading_tracks_the_request_and_a_fast_turn_takes_longer(self):
        samples = [
            {"parameter": 0.0, "yaw_deg": 0.0},
            {"parameter": 1.0, "yaw_deg": 90.0},
        ]
        requested = motion(yaw_mode="profile", yaw_profile=samples)
        plan = retime_stop_path(geometry(), requested, vehicle())
        report = verify_candidate_trajectory(
            geometry(),
            plan,
            requested,
            vehicle(),
            environment(),
            maximum_deviation_m=0.75,
        )
        self.assertTrue(report.accepted)
        self.assertLessEqual(plan.summary.heading_error_bound_rad, math.radians(2.0))
        self.assertAlmostEqual(plan.points[0].yaw_rad, 0.0, delta=math.radians(2.0))
        self.assertAlmostEqual(
            plan.points[-1].yaw_rad, math.radians(90.0), delta=math.radians(2.0)
        )
        self.assertLess(abs(plan.points[0].yaw_rate_rad_s), 1e-6)
        self.assertLess(abs(plan.points[-1].yaw_rate_rad_s), 1e-6)
        self.assertGreater(
            max(abs(point.yaw_rate_rad_s) for point in plan.points),
            0.05,
        )
        fast = motion(
            yaw_mode="profile",
            yaw_profile=[
                {"parameter": 0.0, "yaw_deg": 0.0},
                {"parameter": 1.0, "yaw_deg": 1440.0},
            ],
        )
        spun = retime_stop_path(geometry(), fast, vehicle())
        held = retime_stop_path(geometry(), motion(), vehicle())
        self.assertGreater(
            spun.summary.selected_duration_s,
            held.summary.selected_duration_s,
        )
        sharp = motion(
            yaw_mode="profile",
            yaw_profile=[
                {"parameter": 0.0, "yaw_deg": 0.0},
                {"parameter": 0.499, "yaw_deg": 0.0},
                {"parameter": 0.501, "yaw_deg": 180.0},
                {"parameter": 1.0, "yaw_deg": 180.0},
            ],
        )
        with self.assertRaisesRegex(PlanningError, "explicit heading"):
            retime_stop_path(geometry(), sharp, vehicle())

    def test_explicit_heading_is_refused_when_a_stop_splits_the_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            intent = {
                "schema_version": 1,
                "mission_id": "split-yaw-v1",
                "release_class": "SITL_ONLY",
                "frame": _frame(),
                "path": {
                    "kind": "polyline",
                    "closure": "open_stop",
                    "maximum_deviation_m": 0.5,
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
                            "north_m": 8.0,
                            "east_m": 0.0,
                            "down_m": -8.0,
                            "hard": True,
                            "exact": False,
                        },
                        {
                            "point_id": "finish",
                            "north_m": 8.0,
                            "east_m": 8.0,
                            "down_m": -8.0,
                            "hard": True,
                            "exact": False,
                        },
                    ],
                },
                "motion": {
                    "target_speed_m_s": 1.0,
                    "maximum_speed_m_s": 1.25,
                    "yaw_mode": "profile",
                    "fixed_yaw_deg": None,
                    "look_at_north_m": None,
                    "look_at_east_m": None,
                    "look_at_down_m": None,
                    "yaw_profile": [
                        {"parameter": 0.0, "yaw_deg": 0.0},
                        {"parameter": 1.0, "yaw_deg": 20.0},
                    ],
                    "time_mode": "minimize",
                    "requested_duration_s": None,
                },
                "vehicle_model_id": "px4-x500-sitl-v1",
                "environment_id": "empty-world-v1",
                "objective_priority": [
                    "legal_and_safety",
                    "vehicle_and_environment",
                    "intent_fidelity",
                    "tracking_margin_and_energy",
                    "duration",
                ],
            }
            intent_path = root / "intent.json"
            intent_path.write_text(json.dumps(intent), encoding="utf-8")
            with self.assertRaisesRegex(PlanningError, "one rest-to-rest piece"):
                compile_drawn_path(
                    intent_path=intent_path,
                    vehicle_path=ROOT / "config/planner/x500_sitl_vehicle_model.json",
                    environment_path=ROOT / "config/planner/empty_world_environment.json",
                    trajectory_path=root / "trajectory.csv",
                    report_path=root / "report.json",
                    summary_path=root / "summary.json",
                    control_spacing_m=0.5,
                    verification_resolution_m=0.05,
                    sample_period_s=0.1,
                    limit_scale=0.9,
                    controller_cruise_height_m=8.0,
                    deviation_utilization=0.75,
                )

    def test_circle_entry_meets_the_seam_in_motion_and_stops(self):
        intent, loops = intent_from_interview(
            _circle_interview(
                maximum_deviation_m=1.5,
                entry=[{"north_m": 0.0, "east_m": 6.0}],
            )
        )
        self.assertEqual(loops, 1)
        parsed = MissionIntent.from_dict(intent)
        self.assertEqual(parsed.path.closure, "open_stop")
        self.assertEqual(parsed.path.points[0].point_id, "circle-launch")
        self.assertEqual(
            (parsed.path.points[1].north_m, parsed.path.points[1].east_m),
            (0.0, 6.0),
        )
        self.assertEqual(parsed.path.points[-1].north_m, 0.0)
        self.assertEqual(parsed.path.points[-1].east_m, 0.0)
        self.assertTrue(any(
            point.exact and math.hypot(point.north_m, point.east_m) < 1e-9
            for point in parsed.path.points[1:-1]
        ))
        with self.assertRaisesRegex(PlanningError, "must leave the launch point"):
            intent_from_interview(
                _circle_interview(entry=[{"north_m": 0.0, "east_m": 0.0}])
            )
        plan = build_smooth_path(
            parsed.path,
            control_spacing_m=1.0,
            verification_resolution_m=0.1,
        )
        timed = retime_stop_path(plan, parsed.motion, vehicle(), sample_period_s=0.1)
        start, finish = timed.points[0], timed.points[-1]
        self.assertLess(_horizontal_speed(start), 1e-6)
        self.assertLess(_horizontal_speed(finish), 1e-6)
        self.assertAlmostEqual(finish.north_m, 0.0, places=6)
        self.assertAlmostEqual(finish.east_m, 0.0, places=6)
        entry_times = [
            point.time_s
            for point in timed.points
            if point.north_m < 2.0 and point.east_m > 3.0
        ]
        far = [point for point in timed.points if point.north_m > 6.0]
        self.assertTrue(entry_times)
        self.assertLess(min(entry_times), min(point.time_s for point in far))
        self.assertGreater(max(_horizontal_speed(point) for point in far), 0.2)
        after_entry = [
            point for point in timed.points if point.time_s > max(entry_times)
        ]
        seam = next(
            point
            for point in after_entry
            if math.hypot(point.north_m, point.east_m) < 0.75
        )
        self.assertGreater(_horizontal_speed(seam), 1e-3)
        self.assertLess(seam.time_s, finish.time_s - 1.0)

    def test_circle_exit_rejects_a_stop_on_the_seam_and_other_patterns(self):
        with self.assertRaisesRegex(PlanningError, "must leave the circle seam"):
            intent_from_interview(
                _circle_interview(exit=[{"north_m": 0.0, "east_m": 0.0}])
            )
        with self.assertRaisesRegex(ValueError, "at least one point"):
            intent_from_interview(_circle_interview(exit=[]))
        with self.assertRaisesRegex(PlanningError, "only valid on a circle"):
            intent_from_interview({
                "schema_version": 1,
                "mission_id": "waypoints-v1",
                "pattern": "waypoints",
                "altitude_m": 8.0,
                "maximum_deviation_m": 0.5,
                "maximum_speed_m_s": 1.0,
                "yaw_mode": "fixed",
                "fixed_yaw_deg": 0.0,
                "time_mode": "minimize",
                "vehicle_model_id": "px4-x500-sitl-v1",
                "environment_id": "empty-world-v1",
                "exit": [{"north_m": 4.0, "east_m": 0.0}],
            })

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
                maximum_sample_gap_s=2.0,
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
                maximum_sample_gap_s=2.0,
            )
            self.assertFalse(drifted["accepted"])
        status = simulator_status()
        self.assertFalse(status["campaign_ran"])
        self.assertIn("not a clearance to fly", status["clearance"])


ROOT = Path(__file__).resolve().parents[1]


def _circle_interview(**overrides):
    raw = {
        "schema_version": 1,
        "mission_id": "circle-exit-v1",
        "pattern": "circle",
        "altitude_m": 8.0,
        "maximum_deviation_m": 0.5,
        "maximum_speed_m_s": 1.25,
        "target_speed_m_s": 1.0,
        "yaw_mode": "fixed",
        "fixed_yaw_deg": 0.0,
        "time_mode": "minimize",
        "vehicle_model_id": "px4-x500-sitl-v1",
        "environment_id": "empty-world-v1",
        "loop_count": 1,
        "circle": {"center_north_m": 4.0, "center_east_m": 0.0},
    }
    raw.update(overrides)
    return raw


def _horizontal_speed(point) -> float:
    return math.hypot(point.north_velocity_m_s, point.east_velocity_m_s)


if __name__ == "__main__":
    unittest.main()
