#!/usr/bin/env python3

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

GENERATOR = Path(__file__).resolve().parents[1] / "trajectory_generator"
sys.path.insert(0, str(GENERATOR))

from commands import HoverCommand, LineCommand, RotateCommand, TurnCommand
from export import FIELDS, save_trajectory_csv
from planner import TrajectoryPlanner
from pose import Pose
from profiles import SCurveProfile, TrapezoidalProfile
from segments import MotionProfile
from trajectory import TrajectoryGenerator


class FixedTrajectoryGeneratorTests(unittest.TestCase):
    def test_profiles_include_exact_endpoint(self):
        cases = (
            SCurveProfile(
                distance=1.03,
                start_speed=0.0,
                cruise_speed=1.0,
                end_speed=0.0,
                max_acceleration=1.0,
                max_deceleration=1.0,
                max_jerk=2.0,
            ),
            TrapezoidalProfile(
                distance=1.03,
                start_speed=0.0,
                cruise_speed=1.0,
                end_speed=0.0,
                max_acceleration=1.0,
                max_deceleration=1.0,
            ),
        )
        for profile in cases:
            with self.subTest(profile=type(profile).__name__):
                states = profile.sample(0.05)
                self.assertAlmostEqual(states[0].time, 0.0)
                self.assertAlmostEqual(states[-1].time, profile.total_time)
                self.assertAlmostEqual(states[-1].position, profile.distance)
                self.assertAlmostEqual(states[-1].velocity, profile.v1)
                self.assertAlmostEqual(states[-1].acceleration, 0.0)
                self.assertAlmostEqual(states[-1].jerk, 0.0)
                intervals = [
                    right.time - left.time
                    for left, right in zip(states, states[1:])
                ]
                self.assertTrue(all(interval > 0.0 for interval in intervals))
                self.assertLessEqual(max(intervals), 0.05 + 1e-10)

    def test_smooth_turn_preserves_compatible_boundary_speed(self):
        commands = [
            LineCommand(distance=30.0, speed=2.0, heading=0.0),
            TurnCommand(angle=-180.0, radius=10.0, speed=1.0),
            LineCommand(distance=30.0, speed=2.0, heading=0.0),
        ]
        profiles = TrajectoryPlanner().plan_speeds(commands)
        self.assertEqual(profiles[0].end_speed, 1.0)
        self.assertEqual(profiles[1].start_speed, 1.0)
        self.assertEqual(profiles[1].end_speed, 1.0)
        self.assertEqual(profiles[2].start_speed, 1.0)

    def test_compatible_straight_segments_keep_boundary_speed(self):
        commands = [
            LineCommand(distance=5.0, speed=2.0, heading=0.0),
            LineCommand(distance=5.0, speed=1.0, heading=0.0),
        ]
        profiles = TrajectoryPlanner().plan_speeds(commands)
        self.assertEqual(profiles[0].end_speed, 1.0)
        self.assertEqual(profiles[1].start_speed, 1.0)

    def test_arc_derivatives_include_curvature_terms(self):
        generator = TrajectoryGenerator(
            start_pose=Pose(0.0, 0.0, -8.0, 0.0),
            dt=0.05,
            profile_type="scurve",
        )
        motion = MotionProfile(
            start_speed=2.0,
            cruise_speed=2.0,
            end_speed=2.0,
            max_acceleration=1.0,
            max_deceleration=1.0,
            max_jerk=1.0,
        )
        generator.add_turn(
            TurnCommand(angle=-90.0, radius=10.0, speed=2.0),
            motion,
        )
        points = generator.sample_arc(generator.segments[0])
        start = points[0]
        end = points[-1]
        middle = min(
            points,
            key=lambda point: abs(point.yaw + math.pi / 4.0),
        )

        self.assertAlmostEqual(start.vx, 2.0, places=10)
        self.assertAlmostEqual(start.vy, 0.0, places=10)
        self.assertAlmostEqual(start.ax, 0.0, places=10)
        self.assertAlmostEqual(start.ay, 0.0, places=10)
        self.assertAlmostEqual(start.yaw_rate, 0.0, places=10)
        self.assertAlmostEqual(end.yaw_rate, 0.0, places=10)

        speed = math.hypot(middle.vx, middle.vy)
        tx = math.cos(middle.yaw)
        ty = math.sin(middle.yaw)
        nx = -ty
        ny = tx
        curvature = middle.yaw_rate / speed

        tangent_acceleration = middle.ax * tx + middle.ay * ty
        normal_acceleration = middle.ax * nx + middle.ay * ny
        tangent_jerk = middle.jx * tx + middle.jy * ty
        normal_jerk = middle.jx * nx + middle.jy * ny

        self.assertAlmostEqual(tangent_acceleration, 0.0, places=8)
        self.assertAlmostEqual(
            normal_acceleration,
            speed ** 2 * curvature,
            places=8,
        )
        self.assertAlmostEqual(
            tangent_jerk,
            -(speed ** 3) * curvature ** 2,
            places=8,
        )
        self.assertAlmostEqual(normal_jerk, 0.0, places=8)
        self.assertAlmostEqual(
            middle.yaw_acceleration,
            0.0,
            places=8,
        )
        self.assertAlmostEqual(middle.yaw_jerk, 0.0, places=8)

    def test_whole_mission_has_exact_connections_and_is_repeatable(self):
        commands = [
            LineCommand(
                distance=30.0,
                speed=2.0,
                heading=0.0,
                acceleration=1.0,
            ),
            TurnCommand(angle=-180.0, radius=10.0, speed=1.0),
            LineCommand(
                distance=30.0,
                speed=2.0,
                heading=0.0,
                acceleration=1.0,
            ),
            RotateCommand(angle=-90.0, yaw_rate=20.0),
            LineCommand(
                distance=20.0,
                speed=2.0,
                heading=0.0,
                acceleration=1.0,
            ),
            HoverCommand(duration=5.0),
        ]
        planner = TrajectoryPlanner()
        profiles = planner.plan_speeds(commands)
        generator = TrajectoryGenerator(
            start_pose=Pose(0.0, 0.0, -8.0, 0.0),
            dt=0.05,
            profile_type="scurve",
            default_jerk=1.0,
            default_yaw_jerk=1.0,
        )

        first = generator.generate(commands, profiles)
        first_signature = [
            (
                point.time,
                point.x,
                point.y,
                point.z,
                point.yaw,
                point.vx,
                point.vy,
                point.yaw_rate,
            )
            for point in first
        ]
        second = generator.generate(commands, profiles)
        second_signature = [
            (
                point.time,
                point.x,
                point.y,
                point.z,
                point.yaw,
                point.vx,
                point.vy,
                point.yaw_rate,
            )
            for point in second
        ]
        self.assertEqual(first_signature, second_signature)

        self.assertTrue(
            all(
                right.time > left.time
                for left, right in zip(first, first[1:])
            )
        )
        final = generator.pose
        self.assertAlmostEqual(first[-1].x, final.x, places=10)
        self.assertAlmostEqual(first[-1].y, final.y, places=10)
        self.assertAlmostEqual(first[-1].z, final.z, places=10)
        self.assertAlmostEqual(first[-1].yaw, final.yaw, places=10)
        self.assertAlmostEqual(first[-1].vx, 0.0, places=10)
        self.assertAlmostEqual(first[-1].vy, 0.0, places=10)
        self.assertAlmostEqual(first[-1].yaw_rate, 0.0, places=10)

        expected_connections = [
            (30.0, 0.0, 0.0, 1.0),
            (30.0, -20.0, -math.pi, 1.0),
            (0.0, -20.0, -math.pi, 0.0),
        ]
        for x, y, yaw, expected_speed in expected_connections:
            matches = [
                point
                for point in first
                if math.isclose(point.x, x, abs_tol=1e-9)
                and math.isclose(point.y, y, abs_tol=1e-9)
                and math.isclose(point.yaw, yaw, abs_tol=1e-9)
            ]
            self.assertTrue(matches, f"missing connection {(x, y, yaw)}")
            self.assertTrue(
                any(
                    math.isclose(
                        math.hypot(point.vx, point.vy),
                        expected_speed,
                        abs_tol=1e-10,
                    )
                    and abs(point.yaw_rate) <= 1e-10
                    for point in matches
                )
            )

        moving_turn_steps = [
            abs(right.yaw_rate - left.yaw_rate)
            for left, right in zip(first, first[1:])
            if math.hypot(left.vx, left.vy) > 0.5
            and math.hypot(right.vx, right.vy) > 0.5
        ]
        self.assertLess(max(moving_turn_steps), 0.01)

    def test_export_contains_all_derivatives(self):
        generator = TrajectoryGenerator(
            start_pose=Pose(0.0, 0.0, -8.0, 0.0),
            dt=0.05,
            profile_type="scurve",
        )
        commands = [
            LineCommand(distance=2.0, speed=1.0, heading=0.0),
            HoverCommand(duration=0.13),
        ]
        trajectory = generator.generate(
            commands,
            TrajectoryPlanner().plan_speeds(commands),
        )

        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "trajectory.csv"
            save_trajectory_csv(destination, trajectory)
            with destination.open(newline="") as handle:
                reader = csv.DictReader(handle)
                rows = list(reader)

        self.assertEqual(reader.fieldnames, FIELDS)
        self.assertTrue(rows)
        for field in (
            "jx",
            "jy",
            "jz",
            "yaw_acceleration",
            "yaw_jerk",
        ):
            self.assertIn(field, rows[0])

    def test_hover_uses_exact_requested_duration(self):
        generator = TrajectoryGenerator(
            start_pose=Pose(1.0, 2.0, -3.0, 0.4),
            dt=0.05,
            profile_type="scurve",
        )
        generator.add_hover(HoverCommand(duration=0.13), None)
        points = generator.sample_hover(generator.segments[0])
        self.assertEqual(
            [round(point.time, 10) for point in points],
            [0.0, 0.05, 0.1, 0.13],
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
