#!/usr/bin/env python3
# ruff: noqa: E402
"""Deterministic fault and invariant checks that do not require PX4 SITL."""

import math
import random
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))

from mission_state import MissionPhase, NavigationState
from offboard_runner import check_px4_safety_config
from PID_position_new import PositionController
from VehicleState import VehicleState


class FaultInjectionTests(unittest.TestCase):
    @staticmethod
    def navigation_snapshot(controller, **overrides):
        snapshot = {
            "x": 1.0,
            "y": -2.0,
            "z": -20.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "yaw": 0.2,
            "gps_age_s": 0.1,
            "gps_source_age_s": 0.1,
            "gps_source_regressed": False,
            "gps_fix_type": 3,
            "gps_satellites_visible": 12,
            "gps_hdop": 0.9,
            "gps_vdop": 1.1,
            "gps_horizontal_accuracy_m": 0.8,
            "gps_vertical_accuracy_m": 1.5,
            "estimator_age_s": 0.1,
            "estimator_flags": controller._required_estimator_mask(),
            "estimator_velocity_ratio": 0.2,
            "estimator_pos_horiz_ratio": 0.2,
            "estimator_pos_vert_ratio": 0.2,
        }
        snapshot.update(overrides)
        return snapshot

    def test_mission_state_machine_rejects_skipped_phase(self):
        controller = PositionController()
        with self.assertRaisesRegex(RuntimeError, "TAKEOFF->RETURN_HOME"):
            controller._transition(MissionPhase.RETURN_HOME, 10.0)
        self.assertEqual(controller.phase_snapshot(), MissionPhase.TAKEOFF)

        controller._transition(MissionPhase.TRAJECTORY, 11.0)
        controller._transition(MissionPhase.RETURN_HOME, 12.0)
        controller._transition(MissionPhase.ALIGN, 13.0)
        controller._transition(MissionPhase.HANDOFF, 14.0)
        self.assertTrue(controller.handoff_ready_event.is_set())

    def test_return_request_is_an_event_and_phase_gated(self):
        controller = PositionController()
        self.assertFalse(controller.request_return_home("too early"))
        controller.phase = MissionPhase.TRAJECTORY
        self.assertTrue(controller.request_return_home("test timeout"))
        self.assertTrue(controller.return_request_event.is_set())
        self.assertEqual(controller.return_request_reason, "test timeout")

    def test_hold_uses_last_fully_trusted_reference(self):
        controller = PositionController()
        controller.home_reference_ready = False
        good = self.navigation_snapshot(controller)
        controller._update_navigation_supervisor(good, 10.0)

        jumped = self.navigation_snapshot(
            controller,
            x=55.0,
            y=-48.0,
            z=-3.0,
            gps_fix_type=2,
        )
        controller._update_navigation_supervisor(jumped, 10.1)
        controller._update_navigation_supervisor(jumped, 11.2)
        self.assertEqual(controller.navigation_state, NavigationState.HOLD)
        self.assertEqual(
            controller.navigation_hold_reference,
            (good["x"], good["y"], good["z"], good["yaw"]),
        )

    def test_px4_failsafe_configuration_contract(self):
        safe = {
            "COM_OF_LOSS_T": 0.5,
            "COM_OBL_RC_ACT": 4,
            "COM_DISARM_LAND": 2.0,
        }
        check_px4_safety_config(safe)
        for change, expected in (
            ({"COM_OF_LOSS_T": 2.0}, "COM_OF_LOSS_T"),
            ({"COM_OBL_RC_ACT": 0}, "COM_OBL_RC_ACT"),
            ({"COM_DISARM_LAND": -1.0}, "COM_DISARM_LAND"),
        ):
            with self.subTest(change=change):
                with self.assertRaisesRegex(RuntimeError, expected):
                    check_px4_safety_config(dict(safe, **change))

    def test_random_commands_always_obey_speed_and_slew_limits(self):
        controller = PositionController()
        rng = random.Random(20260929)
        for _ in range(5000):
            controller.navigation_confidence = rng.random()
            requested = tuple(rng.uniform(-100.0, 100.0) for _ in range(3))
            dt = rng.uniform(0.001, 0.20)
            previous = controller.previous_velocity_command
            limited, _ = controller._limit_velocity_command(*requested)
            command, _ = controller._slew_limit_velocity_command(limited, dt)

            self.assertLessEqual(
                math.hypot(command[0], command[1]),
                controller.max_horizontal_speed + 1e-9,
            )
            self.assertLessEqual(
                abs(command[2]),
                controller.max_vertical_speed + 1e-9,
            )
            self.assertLessEqual(
                math.hypot(command[0] - previous[0], command[1] - previous[1]),
                controller.config.command_xy_accel_limit_m_s2 * dt + 1e-9,
            )
            self.assertLessEqual(
                abs(command[2] - previous[2]),
                controller.config.command_z_accel_limit_m_s2 * dt + 1e-9,
            )
            controller._assert_command_invariants(command, 0.0)

        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            controller._assert_command_invariants((math.nan, 0.0, 0.0), 0.0)
        with self.assertRaisesRegex(RuntimeError, "Horizontal"):
            controller._assert_command_invariants((3.1, 0.0, 0.0), 0.0)
        with self.assertRaisesRegex(RuntimeError, "Vertical"):
            controller._assert_command_invariants((0.0, 0.0, 1.1), 0.0)

    def test_random_navigation_faults_are_bounded_and_classified(self):
        controller = PositionController()
        rng = random.Random(314159)
        for _ in range(2000):
            snapshot = self.navigation_snapshot(
                controller,
                gps_fix_type=rng.randint(0, 6),
                gps_satellites_visible=rng.randint(0, 20),
                gps_hdop=rng.uniform(0.4, 6.0),
                gps_vdop=rng.uniform(0.5, 8.0),
                gps_horizontal_accuracy_m=rng.uniform(0.1, 12.0),
                gps_vertical_accuracy_m=rng.uniform(0.2, 18.0),
                estimator_velocity_ratio=rng.uniform(0.0, 2.0),
                estimator_pos_horiz_ratio=rng.uniform(0.0, 2.0),
                estimator_pos_vert_ratio=rng.uniform(0.0, 2.0),
            )
            reasons = controller._navigation_health_reasons(snapshot)
            confidence = controller._navigation_confidence(snapshot)
            self.assertGreaterEqual(confidence, 0.0)
            self.assertLessEqual(confidence, 1.0)
            known_bad = (
                snapshot["gps_fix_type"] < controller.config.gps_min_fix_type
                or snapshot["gps_satellites_visible"]
                < controller.config.gps_min_satellites
                or snapshot["gps_horizontal_accuracy_m"]
                > controller.config.gps_max_horizontal_accuracy_m
                or snapshot["gps_vertical_accuracy_m"]
                > controller.config.gps_max_vertical_accuracy_m
                or snapshot["estimator_velocity_ratio"]
                > controller.config.estimator_max_test_ratio
                or snapshot["estimator_pos_horiz_ratio"]
                > controller.config.estimator_max_test_ratio
                or snapshot["estimator_pos_vert_ratio"]
                > controller.config.estimator_max_test_ratio
            )
            if known_bad:
                self.assertTrue(reasons)

    def test_timestamp_regression_is_latched_and_sample_rejected(self):
        state = VehicleState()
        first = SimpleNamespace(
            time_boot_ms=100,
            x=1.0,
            y=2.0,
            z=3.0,
            vx=0.1,
            vy=0.2,
            vz=0.3,
        )
        state.update_position(first)
        advanced_at = state.position_source_advanced_at
        state.update_position(SimpleNamespace(**{**vars(first), "time_boot_ms": 100}))
        self.assertEqual(state.position_source_advanced_at, advanced_at)

        state.update_position(
            SimpleNamespace(**{**vars(first), "time_boot_ms": 99, "x": 999.0})
        )
        self.assertTrue(state.position_source_regressed)
        self.assertEqual(state.x, 1.0)
        self.assertLessEqual(advanced_at, time.monotonic())


if __name__ == "__main__":
    unittest.main(verbosity=2)
