"""Validate mission geometry before arming and deterministic trajectory queries."""
# ruff: noqa: E402
import csv
from dataclasses import replace
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
