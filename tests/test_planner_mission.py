"""Tangent yaw, closed laps, hard-point stops, interview, and review."""

from dataclasses import asdict, replace
import json
import math
import tempfile
import unittest
from pathlib import Path

from mission_planner.contracts import MissionIntent, PathIntent
from mission_planner.flatness import maximum_tilt_rad, thrust_interval_n
from mission_planner.geometry import build_smooth_path
from mission_planner.plan_drawn_path import compile_drawn_path
from mission_planner.plan_interview import intent_from_interview
from mission_planner.retiming import retime_stop_path
from mission_planner.review import render_review
from mission_planner.verification import verify_candidate_trajectory
from tests.test_planner_retiming import geometry, motion, vehicle
from tests.test_planner_verification import environment


ROOT = Path(__file__).resolve().parents[1]


class PlannerMissionTests(unittest.TestCase):
    def test_tangent_yaw_stays_inside_its_certificate_and_the_yaw_limits(self):
        plan = retime_stop_path(geometry(), motion(yaw_mode="tangent"), vehicle())
        summary = plan.summary
        self.assertLessEqual(
            summary.heading_error_bound_rad,
            math.radians(2.0),
        )
        self.assertLessEqual(
            max(abs(point.yaw_rate_rad_s) for point in plan.points),
            math.radians(vehicle().maximum_yaw_rate_deg_s) * 0.9 + 1e-9,
        )
        self.assertEqual(plan.points[0].yaw_rate_rad_s, 0.0)
        self.assertEqual(plan.points[-1].yaw_rate_rad_s, 0.0)
        report = verify_candidate_trajectory(
            geometry(),
            plan,
            motion(yaw_mode="tangent"),
            vehicle(),
            environment(),
            maximum_deviation_m=0.75,
        )
        self.assertTrue(report.accepted)

    def test_two_laps_return_to_the_seam_with_two_turns_of_heading(self):
        coordinates = []
        radius = 8.0
        for index in range(12):
            angle = math.tau * index / 12
            coordinates.append((
                radius * math.cos(angle) - radius,
                radius * math.sin(angle),
                -8.0,
            ))
        coordinates.append(coordinates[0])
        smooth = build_smooth_path(
            path_from(coordinates, closure="periodic"),
            control_spacing_m=1.0,
            verification_resolution_m=0.05,
        )
        plan = retime_stop_path(
            smooth,
            motion(yaw_mode="tangent"),
            vehicle(),
            loop_count=2,
        )
        self.assertEqual(plan.points[-1].path_parameter, 2.0)
        self.assertAlmostEqual(plan.points[0].north_m, plan.points[-1].north_m, places=5)
        self.assertAlmostEqual(
            plan.points[-1].yaw_rad - plan.points[0].yaw_rad,
            2.0 * math.tau,
            places=2,
        )
        self.assertLess(plan.summary.heading_error_bound_rad, math.radians(2.0))
        report = verify_candidate_trajectory(
            smooth,
            plan,
            motion(yaw_mode="tangent"),
            vehicle(),
            environment(),
            maximum_deviation_m=0.5,
        )
        self.assertTrue(report.accepted)

    def test_a_tighter_tilt_limit_stretches_the_duration(self):
        baseline = retime_stop_path(geometry(), motion(), vehicle())
        tighter = retime_stop_path(
            geometry(),
            motion(),
            replace(vehicle(), maximum_tilt_deg=1.0),
        )
        self.assertGreater(
            tighter.summary.selected_duration_s,
            baseline.summary.selected_duration_s,
        )
        self.assertLessEqual(tighter.summary.tilt_bound_deg, 1.0 + 1e-6)
        self.assertEqual(tighter.summary.limiting_constraint, "tilt")

    def test_hard_point_is_a_stop_on_the_flown_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            intent = {
                "schema_version": 1,
                "mission_id": "hard-stop-v1",
                "release_class": "SITL_ONLY",
                "frame": {
                    "name": "LOCAL_NED",
                    "units": "m",
                    "origin_id": "launch-001",
                    "altitude_datum": "launch",
                },
                "path": {
                    "kind": "polyline",
                    "closure": "open_stop",
                    "maximum_deviation_m": 0.5,
                    "points": [
                        point("start", 0.0, 0.0, hard=True),
                        point("gate", 8.0, 0.0, hard=True),
                        point("finish", 8.0, 8.0, hard=True),
                    ],
                },
                "motion": {
                    "target_speed_m_s": 1.0,
                    "maximum_speed_m_s": 1.25,
                    "yaw_mode": "fixed",
                    "fixed_yaw_deg": 0.0,
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
            summary = compile_drawn_path(
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
                review_path=root / "review.html",
            )
            self.assertEqual(summary["retiming"]["segment_count"], 2)
            rows = (root / "trajectory.csv").read_text(encoding="utf-8").splitlines()
            header = rows[0].split(",")
            north = header.index("x")
            east = header.index("y")
            velocity_n = header.index("vx")
            velocity_e = header.index("vy")
            stopped = False
            for row in rows[1:]:
                fields = row.split(",")
                if (
                    abs(float(fields[north]) - 8.0) < 0.05
                    and abs(float(fields[east])) < 0.05
                    and math.hypot(float(fields[velocity_n]), float(fields[velocity_e]))
                    < 1e-6
                ):
                    stopped = True
            self.assertTrue(stopped)
            review = (root / "review.html").read_text(encoding="utf-8")
            self.assertIn("not a clearance to fly", review)
            self.assertNotIn("safe to fly", review.lower())

    def test_interview_circle_starts_at_launch_and_closes(self):
        raw = json.loads(
            (ROOT / "missions/interview_circle_example.json").read_text(encoding="utf-8")
        )
        intent, loops = intent_from_interview(raw)
        self.assertEqual(loops, 1)
        parsed = MissionIntent.from_dict(intent)
        self.assertEqual(parsed.path.closure, "periodic")
        self.assertEqual(parsed.path.points[0].north_m, 0.0)
        self.assertEqual(parsed.path.points[0].east_m, 0.0)
        self.assertEqual(parsed.motion.yaw_mode, "tangent")

    def test_tilt_of_an_acceleration_ball_is_the_tangent_angle(self):
        acceleration = 1.0
        self.assertAlmostEqual(
            maximum_tilt_rad(acceleration),
            math.asin(acceleration / 9.80665),
            places=12,
        )
        low, high = thrust_interval_n(acceleration, 2.0)
        self.assertAlmostEqual(low, 2.0 * (9.80665 - 1.0), places=9)
        self.assertAlmostEqual(high, 2.0 * (9.80665 + 1.0), places=9)

    def test_review_names_the_limit_and_refuses_a_flight_clearance(self):
        plan = retime_stop_path(geometry(), motion(), vehicle())
        page = render_review(
            {
                "retiming": asdict(plan.summary),
                "limitations": ["SITL_ONLY", "not a clearance to fly"],
            },
            plan,
        )
        self.assertIn("not a clearance to fly", page)
        self.assertIn(plan.summary.limiting_constraint, page)
        self.assertNotIn("safe to fly", page.lower())


def path_from(coordinates, *, closure="open_stop"):
    return PathIntent.from_dict({
        "kind": "polyline",
        "closure": closure,
        "maximum_deviation_m": 0.5,
        "points": [
            {
                "point_id": f"point-{index}",
                "north_m": coordinate[0],
                "east_m": coordinate[1],
                "down_m": coordinate[2],
                "hard": False,
            }
            for index, coordinate in enumerate(coordinates)
        ],
    })


def point(identifier, north, east, *, hard):
    return {
        "point_id": identifier,
        "north_m": north,
        "east_m": east,
        "down_m": -8.0,
        "hard": hard,
    }


if __name__ == "__main__":
    unittest.main()
