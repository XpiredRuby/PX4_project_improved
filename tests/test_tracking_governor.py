"""Reference progress slows without reducing position-feedback authority."""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controller"))
from mission_config import MissionConfig  # noqa: E402
from tracking_governor import TrackingGovernor  # noqa: E402


class TrackingGovernorTests(unittest.TestCase):
    def test_slowdown_and_recovery_are_bounded_and_asymmetric(self):
        cfg = MissionConfig()
        governor = TrackingGovernor(cfg)
        for _ in range(100):
            value = governor.update(5.0, 0.05)
            self.assertGreaterEqual(value, cfg.tracking_minimum_speed_scale)
            self.assertLessEqual(value, 1.0)
        low = governor.scale
        recovered = governor.update(0.0, 0.05)
        self.assertGreater(recovered, low)
        self.assertLess(recovered, low + 0.02)

    def test_nominal_tracking_and_disabled_baseline(self):
        governor = TrackingGovernor(MissionConfig())
        self.assertEqual(governor.update(0.2, 0.05), 1.0)
        governor = TrackingGovernor(replace(MissionConfig(),
                                           tracking_governor_enabled=False))
        self.assertEqual(governor.update(100.0, 0.05), 1.0)

    def test_invalid_inputs_and_invalid_configuration_fail(self):
        governor = TrackingGovernor(MissionConfig())
        for value in (math.nan, math.inf, -1.0):
            with self.assertRaises(ValueError):
                governor.update(value, 0.05)
            with self.assertRaises(ValueError):
                governor.update(0.0, value)
        for changes in ({"tracking_slowdown_full_m": 0.3},
                        {"tracking_scale_rise_tau_s": math.nan},
                        {"tracking_minimum_speed_scale": 0.0},
                        {"max_horizontal_speed_m_s": math.inf}):
            with self.assertRaises(ValueError):
                replace(MissionConfig(), **changes).validate()

    def test_stall_does_not_jump_scale(self):
        a, b = TrackingGovernor(MissionConfig()), TrackingGovernor(MissionConfig())
        self.assertAlmostEqual(a.update(4.0, 100.0), b.update(4.0, 0.1))

    def test_integrated_clock_and_feedforward_share_scale_feedback_keeps_authority(self):
        from PID_position_new import PositionController
        from mission_state import MissionPhase, NavigationState
        controller = PositionController()
        controller.phase = MissionPhase.TRAJECTORY
        controller.phase_clock_s = 0.
        controller.navigation_state = NavigationState.HEALTHY
        controller.tracking_reference = (2., 0., 0.)
        for _ in range(30):
            controller._update_tracking_governor({"x": 0., "y": 0.}, .05)
        scale = controller._motion_speed_scale()
        self.assertLess(scale, .4)
        controller._advance_phase_clock(.05)
        self.assertAlmostEqual(controller.phase_clock_s, .05 * scale)
        limited, _ = controller._limit_velocity_command(0.8, 0., 0.)
        self.assertEqual(limited, (0.8, 0., 0.))
        controller.navigation_state = NavigationState.HOLD
        clock = controller.phase_clock_s
        controller._advance_phase_clock(.05)
        self.assertEqual(controller.phase_clock_s, clock)
