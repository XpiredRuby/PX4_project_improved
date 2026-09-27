#!/usr/bin/env python3

import math
import os
import sys
import time
import unittest
from pathlib import Path


OVERLAY = Path("/mnt/f/PX4/research/randomized")
RUNTIME = Path("/home/xpire/vishnu-runtime/Position_Controller")
sys.path.insert(0, str(OVERLAY))
os.chdir(RUNTIME)

from RandomizedPositionController import RandomizedPositionController
from vishnu_offboard_runner import (
    post_disarm_ready,
    release_propulsion_ready,
)
from spawn_config import build_pose, parser, quaternion_from_rpy_deg, validate_pose


class RandomizedOverlayTests(unittest.TestCase):
    def make_controller(self, height=6.0):
        controller = RandomizedPositionController(
            spawn_height_m=height,
            ground_reference=(0.0, 0.0, 0.0, 0.0),
        )
        controller.x0 = 1.0
        controller.y0 = -1.0
        controller.z0 = -height
        controller.yaw0 = 0.2
        controller.target_x = controller.x0
        controller.target_y = controller.y0
        controller.target_z = controller.z0
        controller.land_x = controller.x0
        controller.land_y = controller.y0
        controller.land_yaw_unwrapped = controller.yaw0
        controller.ground_target_z = controller.z0 + height
        controller.pid_x.setpoint = controller.x0
        controller.pid_y.setpoint = controller.y0
        controller.pid_z.setpoint = controller.z0
        return controller

    def test_random_pose_is_seeded_and_bounded(self):
        args = parser().parse_args(["--mode", "random", "--seed", "12345"])
        first = build_pose(args)
        second = build_pose(args)
        self.assertEqual(first, second)
        self.assertLessEqual(abs(first.x), 2.0)
        self.assertLessEqual(abs(first.y), 2.0)
        self.assertGreaterEqual(first.z, 3.0)
        self.assertLessEqual(first.z, 10.0)
        self.assertEqual(args.tilt_max_deg, 2.0)
        self.assertLessEqual(abs(first.roll_deg), 2.0)
        self.assertLessEqual(abs(first.pitch_deg), 2.0)

    def test_manual_pose_requires_xyz_and_rejects_unsafe_tilt(self):
        with self.assertRaisesRegex(ValueError, "requires --x, --y, and --z"):
            build_pose(parser().parse_args(["--mode", "manual", "--x", "1"]))
        args = parser().parse_args(
            [
                "--mode",
                "manual",
                "--x",
                "1",
                "--y",
                "-1",
                "--z",
                "5",
                "--roll-deg",
                "16",
            ]
        )
        with self.assertRaisesRegex(ValueError, "safety-limited"):
            build_pose(args)

    def test_quaternion_is_normalized(self):
        q = quaternion_from_rpy_deg(8.0, -6.0, 123.0)
        self.assertAlmostEqual(math.sqrt(sum(value * value for value in q)), 1.0)

    def test_launcher_height_defines_local_ground(self):
        controller = self.make_controller(height=7.5)
        state = controller.state
        state.x, state.y, state.z, state.yaw = 1.2, -0.8, -7.4, 0.3
        now = time.monotonic()
        state.position_received = True
        state.attitude_received = True
        state.heartbeat_received = True
        state.position_received_at = now
        state.attitude_received_at = now
        state.heartbeat_received_at = now
        controller.initialize_airborne_target()
        self.assertAlmostEqual(controller.ground_from_height_z, 0.1)
        self.assertAlmostEqual(controller.ground_target_z, 0.0)
        self.assertEqual(controller.ground_target_source, "stored_ground_reference")
        self.assertAlmostEqual(controller.estimated_agl_m(-7.4), 7.4)

    def test_launcher_height_is_backup_without_stored_ground(self):
        controller = RandomizedPositionController(spawn_height_m=7.5)
        state = controller.state
        state.x, state.y, state.z, state.yaw = 1.2, -0.8, -7.4, 0.3
        now = time.monotonic()
        state.position_received = True
        state.attitude_received = True
        state.heartbeat_received = True
        state.position_received_at = now
        state.attitude_received_at = now
        state.heartbeat_received_at = now
        controller.initialize_airborne_target()
        self.assertAlmostEqual(controller.ground_target_z, 0.1)
        self.assertEqual(controller.ground_target_source, "launcher_height_backup")

    def test_verified_gazebo_height_replaces_requested_height(self):
        controller = self.make_controller(height=8.8)
        controller.set_realized_spawn_height(8.57)
        self.assertAlmostEqual(controller.spawn_height_m, 8.57)

    def test_trajectory_is_translated_to_airborne_origin(self):
        controller = self.make_controller(height=6.0)
        controller.mission_time = 0.0
        result = controller.trajectory_controller(1.0, -1.0, -6.0, 0.05)
        self.assertAlmostEqual(result["desired"][0], controller.x0)
        self.assertAlmostEqual(result["desired"][1], controller.y0)
        self.assertAlmostEqual(result["desired"][2], controller.z0)

    def test_descent_rate_slows_inside_final_metre(self):
        controller = self.make_controller()
        rates = [
            controller.descent_rate_target(height)
            for height in (10.0, 1.0, 0.5, 0.3, 0.05)
        ]
        self.assertEqual(rates[0], controller.max_descent_rate)
        self.assertTrue(all(left >= right for left, right in zip(rates, rates[1:])))
        self.assertLess(rates[-1], rates[0])
        self.assertEqual(controller.descent_rate_target(0.0), 0.0)

    def test_landing_deceleration_is_rate_limited(self):
        controller = self.make_controller()
        controller.current_vz_cmd = 0.5
        controller.landing_controller(1.0, -1.0, -0.30, 0.2, 0.05)
        self.assertGreaterEqual(controller.current_vz_cmd, 0.5 - 0.65 * 0.05)
        self.assertLess(controller.current_vz_cmd, 0.5)

    def test_handoff_requires_low_altitude_velocity_xy_and_tilt_hold(self):
        controller = self.make_controller()
        controller.phase = "LAND"
        controller.phase_enter_time = 990.0
        controller.state.vz = 0.10
        controller.state.roll = math.radians(2.0)
        controller.state.pitch = math.radians(1.0)
        controller.update_phase(1.0, -1.0, -0.20, 1000.0)
        self.assertEqual(controller.phase, "LAND")
        controller.update_phase(1.0, -1.0, -0.20, 1000.40)
        self.assertEqual(controller.phase, "DONE")

    def test_stabilize_requires_continuous_hold(self):
        controller = self.make_controller()
        controller.phase = "STABILIZE"
        controller.fixture_released = True
        controller.phase_enter_time = 100.0
        controller.state.x = controller.x0
        controller.state.y = controller.y0
        controller.state.z = controller.z0
        controller.state.vx = controller.state.vy = controller.state.vz = 0.0
        controller.state.roll = controller.state.pitch = 0.0
        controller.state.roll_rate = 0.0
        controller.state.pitch_rate = 0.0
        controller.state.yaw_rate = 0.0
        controller.update_phase(controller.x0, controller.y0, controller.z0, 101.0)
        self.assertEqual(controller.phase, "STABILIZE")
        controller.update_phase(controller.x0, controller.y0, controller.z0, 102.3)
        self.assertEqual(controller.phase, "TRAJECTORY")

    def test_fixture_release_ready_allows_bounded_initial_tilt(self):
        controller = self.make_controller()
        controller.phase = "STABILIZE"
        controller.phase_enter_time = 100.0
        controller.state.vx = controller.state.vy = controller.state.vz = 0.0
        controller.state.roll = math.radians(2.0)
        controller.state.pitch = math.radians(-2.0)
        controller.state.roll_rate = 0.0
        controller.state.pitch_rate = 0.0
        controller.state.yaw_rate = 0.0
        controller.update_phase(controller.x0, controller.y0, controller.z0, 101.0)
        self.assertFalse(controller.release_ready)
        controller.update_phase(controller.x0, controller.y0, controller.z0, 101.25)
        self.assertTrue(controller.release_ready)
        self.assertEqual(controller.phase, "STABILIZE")

    def test_stabilization_guard_rejects_large_excursion(self):
        controller = self.make_controller()
        controller.phase = "STABILIZE"
        controller.fixture_released = True
        controller.phase_enter_time = 100.0
        controller.state.vx = controller.state.vy = controller.state.vz = 0.0
        controller.state.roll = controller.state.pitch = 0.0
        controller.state.roll_rate = controller.state.pitch_rate = 0.0
        controller.state.yaw_rate = 0.0
        with self.assertRaisesRegex(RuntimeError, "excursion limit"):
            controller.update_phase(
                controller.x0 + controller.stabilize_max_excursion_m + 0.01,
                controller.y0,
                controller.z0,
                101.0,
            )

    def test_stabilization_guard_rejects_high_horizontal_speed(self):
        controller = self.make_controller()
        controller.phase = "STABILIZE"
        controller.fixture_released = True
        controller.phase_enter_time = 100.0
        controller.state.vx = controller.stabilize_max_horizontal_speed_m_s + 0.01
        controller.state.vy = controller.state.vz = 0.0
        controller.state.roll = controller.state.pitch = 0.0
        controller.state.roll_rate = controller.state.pitch_rate = 0.0
        controller.state.yaw_rate = 0.0
        with self.assertRaisesRegex(RuntimeError, "speed limit"):
            controller.update_phase(
                controller.x0,
                controller.y0,
                controller.z0,
                101.0,
            )

    def test_stabilization_guard_rejects_large_vertical_excursion(self):
        controller = self.make_controller()
        controller.phase = "STABILIZE"
        controller.fixture_released = True
        controller.phase_enter_time = 100.0
        controller.state.vx = controller.state.vy = controller.state.vz = 0.0
        controller.state.roll = controller.state.pitch = 0.0
        controller.state.roll_rate = controller.state.pitch_rate = 0.0
        controller.state.yaw_rate = 0.0
        with self.assertRaisesRegex(RuntimeError, "vertical excursion limit"):
            controller.update_phase(
                controller.x0,
                controller.y0,
                controller.z0 + controller.stabilize_max_vertical_excursion_m + 0.01,
                101.0,
            )

    def test_stabilization_guard_rejects_high_vertical_speed(self):
        controller = self.make_controller()
        controller.phase = "STABILIZE"
        controller.fixture_released = True
        controller.phase_enter_time = 100.0
        controller.state.vx = controller.state.vy = 0.0
        controller.state.vz = controller.stabilize_max_vertical_speed_m_s + 0.01
        controller.state.roll = controller.state.pitch = 0.0
        controller.state.roll_rate = controller.state.pitch_rate = 0.0
        controller.state.yaw_rate = 0.0
        with self.assertRaisesRegex(RuntimeError, "vertical speed limit"):
            controller.update_phase(
                controller.x0,
                controller.y0,
                controller.z0,
                101.0,
            )

    def test_release_propulsion_gate_requires_fresh_near_hover_state(self):
        ready = {
            "target_thrust": 0.65,
            "min_motor_output": 480.0,
            "mean_motor_output": 680.0,
            "attitude_target_age_s": 0.05,
            "actuator_age_s": 0.05,
        }
        self.assertTrue(release_propulsion_ready(ready))
        for field, unsafe_value in (
            ("target_thrust", 0.60),
            ("mean_motor_output", 600.0),
            ("attitude_target_age_s", 0.30),
            ("actuator_age_s", 0.30),
        ):
            candidate = dict(ready)
            candidate[field] = unsafe_value
            self.assertFalse(release_propulsion_ready(candidate), field)

    def test_post_disarm_gate_requires_zero_output_and_settled_motion(self):
        ready = {
            "armed": False,
            "vx": 0.02,
            "vy": -0.01,
            "vz": 0.01,
            "motors": (0.0, 0.0, 0.0, 0.0),
            "heartbeat_age_s": 0.05,
            "position_age_s": 0.05,
            "actuator_age_s": 0.05,
        }
        self.assertTrue(post_disarm_ready(ready))
        for field, unsafe_value in (
            ("armed", True),
            ("vx", 0.16),
            ("vz", 0.11),
            ("heartbeat_age_s", 1.60),
            ("position_age_s", 0.60),
            ("actuator_age_s", 0.60),
        ):
            candidate = dict(ready)
            candidate[field] = unsafe_value
            self.assertFalse(post_disarm_ready(candidate), field)
        candidate = dict(ready)
        candidate["motors"] = (0.0, 0.0, 2.0, 0.0)
        self.assertFalse(post_disarm_ready(candidate), "motors")

    def test_fixture_held_controller_sends_neutral_velocity(self):
        controller = self.make_controller()
        controller.phase = "STABILIZE"
        controller.fixture_released = False
        result = controller.stabilize_controller(5.0, -4.0, -2.0, 0.05)
        self.assertEqual(result["command"], (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
