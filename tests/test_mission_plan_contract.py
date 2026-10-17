"""Validate mission geometry before arming and deterministic trajectory queries."""
# ruff: noqa: E402
import csv
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))
from PID_position_new import PositionController
from trajectory import Trajectory, TrajectoryPoint
from mission_config import MissionConfig
import offboard_runner


class MissionPlanContractTests(unittest.TestCase):
    def controller(self):
        # Full configured controller, without a live transport.
        with patch("PID_position_new.Trajectory", return_value=SimpleNamespace(
                points=[TrajectoryPoint(time=0.), TrajectoryPoint(time=1.)], duration=1.)):
            return PositionController()

    def test_geometry_is_rejected_during_construction_before_transport(self):
        outside = TrajectoryPoint(time=1., x=MissionConfig().max_operating_radius_m + 1.)
        with patch("PID_position_new.Trajectory", return_value=SimpleNamespace(
                points=[TrajectoryPoint(), outside], duration=1.)):
            with self.assertRaisesRegex(RuntimeError, "pre-arm validation.*radius"):
                PositionController()

    def test_navigation_uncertainty_budget_applies_to_entire_trajectory(self):
        c = self.controller()
        c.configure_cruise_height({"gps_vertical_accuracy_m": 1.})
        minimum = c.minimum_trajectory_height_m
        c.trajectory.points[1].z = c.cruise_height_m - minimum + .01
        with self.assertRaisesRegex(RuntimeError, "clearance budget"):
            c.validate_mission_plan(require_home=False)
        c.trajectory.points[1].z = c.cruise_height_m - minimum
        c.validate_mission_plan(require_home=False)

    def test_yaw_derivative_limits_are_checked_before_transport(self):
        limits = MissionConfig()
        cases = (
            ("yaw_rate", math.radians(limits.max_yaw_rate_deg_s + 1.0)),
            (
                "yaw_acceleration",
                math.radians(limits.max_yaw_acceleration_deg_s2 + 1.0),
            ),
            ("yaw_jerk", math.radians(limits.max_yaw_jerk_deg_s3 + 1.0)),
        )
        for field, value in cases:
            point = TrajectoryPoint(time=1.0)
            setattr(point, field, value)
            with self.subTest(field=field), patch(
                "PID_position_new.Trajectory",
                return_value=SimpleNamespace(
                    points=[TrajectoryPoint(), point], duration=1.0
                ),
            ), self.assertRaisesRegex(RuntimeError, "yaw"):
                PositionController()

    def test_impossible_uncertainty_and_nonfinite_accuracy_fail(self):
        c = self.controller()
        for value in (-1., math.nan, math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                c.configure_cruise_height({"gps_vertical_accuracy_m": value})
        c.config = replace(c.config, max_height_above_launch_m=15.)
        with self.assertRaisesRegex(RuntimeError, "ceiling"):
            c.configure_cruise_height({"gps_vertical_accuracy_m": 10.})

    def test_prearm_plan_failure_never_requests_arm(self):
        c = Mock()
        c.master = object()
        c.trajectory_path = ROOT / "controller" / "trajectory.csv"
        c.validate_mission_plan.side_effect = RuntimeError("invalid plan")
        with (patch.object(offboard_runner, "PositionController", return_value=c),
              patch.object(offboard_runner, "RunRecord"),
              patch.object(offboard_runner, "audit_px4_configuration", return_value={}),
              patch.object(offboard_runner, "wait_for_bootstrap_ready"),
              patch.object(offboard_runner, "wait_for_initial_ground_state"),
              patch.object(offboard_runner, "capture_launch_reference", return_value=SimpleNamespace(vertical_accuracy_m=1.)),
              patch.object(offboard_runner, "heartbeat_snapshot", return_value=(None,) * 3),
              patch.object(offboard_runner, "final_state_snapshot", return_value={}),
              patch.object(offboard_runner, "request_arm") as arm):
            with self.assertRaisesRegex(RuntimeError, "invalid plan"):
                offboard_runner.main()
        arm.assert_not_called()

    def test_constructor_rejection_finalizes_manifest_without_connecting(self):
        record = Mock()
        with (patch.object(offboard_runner, "RunRecord", return_value=record),
              patch.object(offboard_runner, "PositionController", side_effect=ValueError("invalid trajectory"))):
            with self.assertRaisesRegex(ValueError, "invalid trajectory"):
                offboard_runner.main()
        self.assertEqual(record.finalize.call_args.kwargs["outcome"], "PREARM_REJECTED")
        self.assertEqual(record.finalize.call_args.kwargs["final_state"], {})

    def test_planner_summary_binds_trajectory_and_live_cruise_height(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            trajectory = directory / "trajectory.csv"
            trajectory.write_bytes(b"verified trajectory")
            summary = directory / "summary.json"
            summary.write_text(json.dumps({
                "schema_version": 1,
                "release_class": "SITL_ONLY",
                "controller_trajectory_sha256": hashlib.sha256(
                    trajectory.read_bytes()
                ).hexdigest(),
                "controller_cruise_height_m": 17.0,
            }), encoding="utf-8")
            expected = offboard_runner.load_planner_summary(summary, trajectory)
            self.assertEqual(expected, 17.0)
            offboard_runner.validate_planner_cruise_height(
                SimpleNamespace(cruise_height_m=17.0),
                expected,
            )
            with self.assertRaisesRegex(RuntimeError, "does not match"):
                offboard_runner.validate_planner_cruise_height(
                    SimpleNamespace(cruise_height_m=18.0),
                    expected,
                )
            trajectory.write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "does not match"):
                offboard_runner.load_planner_summary(summary, trajectory)

    def test_invalid_planner_summary_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            trajectory = directory / "trajectory.csv"
            trajectory.write_bytes(b"trajectory")
            summary = directory / "summary.json"
            for value in (
                [],
                {"schema_version": 2},
                {
                    "schema_version": 1,
                    "release_class": "FLIGHT_CANDIDATE",
                },
            ):
                summary.write_text(json.dumps(value), encoding="utf-8")
                with self.subTest(value=value), self.assertRaises(ValueError):
                    offboard_runner.load_planner_summary(summary, trajectory)
            summary.write_text('{"controller_cruise_height_m": NaN}')
            with self.assertRaisesRegex(ValueError, "non-finite"):
                offboard_runner.load_planner_summary(summary, trajectory)

    def test_drawn_path_csv_requires_planner_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            trajectory = Path(directory) / "drawn.csv"
            trajectory.write_text(
                "time,x,planner_contract_version\n0,0,1\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "require --planner-summary"):
                offboard_runner.load_planner_summary(None, trajectory)

    def test_explicit_legacy_trajectory_requires_hash_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            trajectory = directory / "waypoint.csv"
            trajectory.write_text("time,x\n0,0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "require --planner-summary"):
                offboard_runner.load_planner_summary(
                    None,
                    trajectory,
                    required=True,
                )
            summary = directory / "summary.json"
            summary.write_text(json.dumps({
                "schema_version": 1,
                "trajectory_sha256": hashlib.sha256(
                    trajectory.read_bytes()
                ).hexdigest(),
            }), encoding="utf-8")
            self.assertIsNone(
                offboard_runner.load_planner_summary(
                    summary,
                    trajectory,
                    required=True,
                )
            )


class TrajectoryContractTests(unittest.TestCase):
    fields = ["time", "x", "y", "z", "vx", "vy", "vz", "yaw", "yaw_rate", "ax", "ay", "az"]

    def load(self, times=(0., 1., 2.), missing=None, empty=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.csv"
            fields = [name for name in self.fields if name != missing]
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=fields)
                writer.writeheader()
                for t in times:
                    row = {name: 0. for name in fields}
                    row.update({name: value for name, value in (("time", t), ("x", t * t)) if name in fields})
                    if empty is not None:
                        row[empty] = ""
                    writer.writerow(row)
            return Trajectory(path)

    def test_missing_or_empty_required_coordinates_cannot_silently_become_zero(self):
        for name in self.fields:
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.load(missing=name)
            with self.subTest(empty=name), self.assertRaises(ValueError):
                self.load(empty=name)

    def test_invalid_times_are_rejected(self):
        for times in ((1., 2.), (0., 0.), (0., math.nan), (0., math.inf)):
            with self.subTest(times=times), self.assertRaises(ValueError):
                self.load(times=times)

    def test_backward_query_after_completion_uses_correct_segment(self):
        trajectory = self.load()
        self.assertEqual(trajectory.get_target(3.).x, 4.)
        self.assertTrue(trajectory.finished)
        self.assertAlmostEqual(trajectory.get_target(.5).x, .5)
        self.assertFalse(trajectory.finished)
        self.assertEqual(trajectory.get_target(-1.).x, 0.)
        for time in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                trajectory.get_target(time)

    def test_large_finite_headings_do_not_hang_or_overflow_interpolation(self):
        trajectory = self.load()
        for start, finish in ((0., 1e300), (-1e308, 1e308)):
            with self.subTest(start=start, finish=finish):
                trajectory.points[0].yaw = start
                trajectory.points[1].yaw = finish
                self.assertTrue(math.isfinite(trajectory.get_target(.5).yaw))
        for finish in (math.pi, -math.pi):
            trajectory.points[0].yaw = 0.
            trajectory.points[1].yaw = finish
            self.assertAlmostEqual(trajectory.get_target(.5).yaw, finish / 2.)

    def test_controller_can_select_an_explicit_prevalidated_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "five-stops.csv"
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=self.fields)
                writer.writeheader()
                for time_s in (0.0, 1.0):
                    writer.writerow({
                        name: (
                            time_s if name == "time"
                            else 1.0 if name == "x" and time_s else 0.0
                        )
                        for name in self.fields
                    })
            controller = PositionController(trajectory_path=path)
            self.assertEqual(controller.trajectory_path, path.resolve())
            self.assertEqual(controller.trajectory.points[-1].x, 1.0)

    def test_runner_trajectory_argument_is_explicit(self):
        path = Path("planned.csv")
        summary = Path("planned.summary.json")
        arguments = offboard_runner.parse_args([
            "--trajectory",
            str(path),
            "--planner-summary",
            str(summary),
        ])
        self.assertEqual(arguments.trajectory, path)
        self.assertEqual(arguments.planner_summary, summary)
