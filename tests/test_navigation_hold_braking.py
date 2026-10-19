"""Regression evidence for bounded HOLD braking and unchanged normal motion."""
import math
import random
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controller"))
from PID_position_new import PositionController  # noqa: E402
from mission_config import MissionConfig  # noqa: E402
from mission_state import NavigationState  # noqa: E402


class NavigationHoldBrakingTests(unittest.TestCase):
    def test_moving_hold_command_brakes_without_a_jump(self):
        controller = PositionController()
        controller.previous_velocity_command = (1.6, 0.0, 0.2)
        controller.navigation_state = NavigationState.HOLD
        controller.navigation_hold_reference = (0., 0., 0., 0.)
        controller.phase_clock_s = 15.
        previous = controller.previous_velocity_command
        speeds = []
        for _ in range(12):
            control = controller.navigation_hold_controller(.4, 0., 0., 0., .05)
            command, _ = controller._slew_limit_velocity_command(control["command"], .05)
            self.assertLessEqual(math.hypot(command[0]-previous[0],
                                           command[1]-previous[1]), .15+1e-9)
            self.assertLessEqual(abs(command[2]-previous[2]), .05+1e-9)
            controller._assert_command_invariants(command, 0.)
            controller._advance_phase_clock(.05)
            self.assertEqual(controller.phase_clock_s, 15.)
            speeds.append(command[0])
            previous = command
        self.assertLessEqual(speeds[10], 0.)
        self.assertEqual(controller.navigation_hold_reference, (0., 0., 0., 0.))

    def test_all_navigation_states_obey_their_vector_slew_limit(self):
        rng = random.Random(20261003)
        for state in (NavigationState.HEALTHY, NavigationState.DEGRADED,
                      NavigationState.HOLD):
            controller = PositionController()
            controller.navigation_state = state
            limit = (3. if state == NavigationState.HOLD else 1.5)
            for _ in range(1000):
                controller.navigation_confidence = rng.random()
                dt = rng.uniform(.001, .2)
                previous = controller.previous_velocity_command
                request = tuple(rng.uniform(-10., 10.) for _ in range(3))
                command, _ = controller._slew_limit_velocity_command(
                    controller._limit_velocity_command(*request)[0], dt)
                self.assertLessEqual(math.hypot(command[0]-previous[0],
                                               command[1]-previous[1]), limit*dt+1e-9)
                self.assertLessEqual(abs(command[2]-previous[2]), dt+1e-9)
                controller._assert_command_invariants(command, 0.)

    def test_recovery_immediately_returns_to_normal_slew(self):
        controller = PositionController()
        controller.navigation_state = NavigationState.HOLD
        controller.previous_velocity_command = (1., 0., 0.)
        controller._slew_limit_velocity_command((0., 0., 0.), .05)
        previous = controller.previous_velocity_command
        controller.navigation_state = NavigationState.HEALTHY
        command, _ = controller._slew_limit_velocity_command((0., 0., 0.), .05)
        self.assertAlmostEqual(previous[0]-command[0], .075)

    def test_measured_velocity_damping_opposes_motion_and_is_logged(self):
        controller = PositionController()
        controller.navigation_hold_reference = (0., 0., 0., 0.)
        controller.navigation_state = NavigationState.HOLD
        result = controller.navigation_hold_controller(0., 0., 0., 0., .05,
                                                       vx=1., vy=-.5)
        self.assertAlmostEqual(result["command"][0], -.8)
        self.assertAlmostEqual(result["command"][1], .4)
        self.assertAlmostEqual(result["pid"][0]["d"], -.8)
        self.assertAlmostEqual(result["pid"][1]["d"], .4)
        self.assertAlmostEqual(result["pid"][0]["correction"], -.8)
        self.assertEqual(controller.pid_x.Kd, 0.)
        for value in (math.nan, math.inf, -0.1, 1.51):
            with self.subTest(value=value), self.assertRaises(ValueError):
                replace(MissionConfig(), navigation_hold_velocity_damping=value).validate()

    def test_cruise_retimes_clock_and_feedforward_together(self):
        from mission_state import MissionPhase
        from trajectory import TrajectoryPoint
        from unittest.mock import Mock

        controller = PositionController()
        controller.phase = MissionPhase.TRAJECTORY
        controller.phase_clock_s = 10.
        target = TrajectoryPoint(x=1., vx=2., vy=0.)
        controller.trajectory = Mock()
        controller.trajectory.get_target.return_value = target
        self.assertAlmostEqual(controller._motion_speed_scale(), .5)
        controller._advance_phase_clock(.05)
        self.assertAlmostEqual(controller.phase_clock_s, 10.025)
        result = controller.trajectory_controller(
            controller.x0+1.-controller.trajectory_origin.x,
            controller.y0-controller.trajectory_origin.y,
            controller.target_z-controller.trajectory_origin.z, .05)
        self.assertAlmostEqual(result["planned"][0], 1.)
        self.assertAlmostEqual(result["command"][0], 1.)
        limited, _ = controller._limit_velocity_command(3., 4., 0.)
        self.assertAlmostEqual(math.hypot(*limited[:2]), 1.25)
        controller.navigation_state = NavigationState.HOLD
        limited, _ = controller._limit_velocity_command(3., 4., 0.)
        self.assertAlmostEqual(math.hypot(*limited[:2]), 3.)
        for changes in ({"cruise_reference_speed_m_s": 0.},
                        {"cruise_command_speed_limit_m_s": .9},
                        {"cruise_command_speed_limit_m_s": 3.1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(MissionConfig(), **changes).validate()

    def test_hold_braking_configuration_rejects_unsafe_values(self):
        for value in (math.nan, math.inf, -1., 0., 1.4, 3.01):
            with self.subTest(value=value), self.assertRaises(ValueError):
                replace(MissionConfig(), navigation_hold_accel_limit_m_s2=value).validate()
        replace(MissionConfig(), navigation_hold_accel_limit_m_s2=1.5).validate()

    def test_native_landing_reference_is_frozen_from_hold_or_home(self):
        from mission_state import MissionPhase
        controller = PositionController()
        controller.phase = MissionPhase.TRAJECTORY
        controller.navigation_hold_reference = (12., -3., -20., 0.)
        controller.tracking_reference = (13., -2., -20.)
        controller.begin_failsafe_handoff()
        self.assertEqual(controller.native_landing_reference_xy, (12., -3.))
        controller.navigation_hold_reference = (99., 99., -20., 0.)
        controller._freeze_native_landing_reference()
        self.assertEqual(controller.native_landing_reference_xy, (12., -3.))

        controller = PositionController()
        controller.phase = MissionPhase.HANDOFF
        controller.land_x, controller.land_y = 4., 5.
        controller.tracking_reference = (4.1, 5.2, -20.)
        controller.prepare_native_land_handoff()
        self.assertEqual(controller.native_landing_reference_xy, (4., 5.))


if __name__ == "__main__":
    unittest.main()
