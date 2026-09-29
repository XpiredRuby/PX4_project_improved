#!/usr/bin/env python3
# ruff: noqa: E402
"""Control-policy tests for navigation degradation and unknown ground height."""

import math
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))

from PID_position_new import NavigationEstimateLost, PositionController
from minimum_jerk import MinimumJerkSegment
from offboard_runner import check_gps_imu_estimator_config


class SafetySupervisorTests(unittest.TestCase):
    def test_estimator_source_audit_rejects_unverified_or_extra_sources(self):
        gps_imu = {
            "EKF2_GPS_CTRL": 7,
            "EKF2_HGT_REF": 1,
            "EKF2_BARO_CTRL": 0,
            "EKF2_MAG_TYPE": 5,
            "EKF2_OF_CTRL": 0,
            "EKF2_EV_CTRL": 0,
            "EKF2_RNG_CTRL": 0,
            "EKF2_AGP_CTRL": 0,
        }
        check_gps_imu_estimator_config(gps_imu)
        check_gps_imu_estimator_config(dict(gps_imu, EKF2_GPS_CTRL=15))
        with self.assertRaisesRegex(RuntimeError, "Cannot verify"):
            check_gps_imu_estimator_config({"EKF2_GPS_CTRL": 7})
        with self.assertRaisesRegex(RuntimeError, "EKF2_BARO_CTRL=1"):
            check_gps_imu_estimator_config(dict(gps_imu, EKF2_BARO_CTRL=1))
        with self.assertRaisesRegex(RuntimeError, "EKF2_MAG_TYPE=6"):
            check_gps_imu_estimator_config(dict(gps_imu, EKF2_MAG_TYPE=6))
        with self.assertRaisesRegex(RuntimeError, "EKF2_GPS_CTRL=5"):
            check_gps_imu_estimator_config(dict(gps_imu, EKF2_GPS_CTRL=5))

    def make_controller(self):
        controller = PositionController()
        controller.x0 = controller.y0 = controller.z0 = 0.0
        controller.yaw0 = 0.0
        controller.phase = "TRAJECTORY"
        controller.phase_enter_time = time.monotonic()
        return controller

    @staticmethod
    def navigation_snapshot(controller):
        return {
            "x": 0.0, "y": 0.0, "z": -20.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "yaw": 0.0,
            "gps_age_s": 0.1,
            "gps_source_age_s": 0.1,
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

    def test_cruise_height_budgets_vertical_uncertainty_and_ceiling(self):
        controller = self.make_controller()
        controller.configure_cruise_height(
            {"gps_vertical_accuracy_m": 2.0}
        )
        self.assertAlmostEqual(controller.cruise_height_m, 20.0)
        self.assertAlmostEqual(controller.takeoff_altitude, -20.0)

        with self.assertRaisesRegex(RuntimeError, "above the configured"):
            controller.configure_cruise_height(
                {"gps_vertical_accuracy_m": 8.0}
            )

    def test_bad_gps_holds_path_then_requires_stable_recovery(self):
        controller = self.make_controller()
        good = self.navigation_snapshot(controller)
        bad = dict(good, gps_fix_type=2)
        controller._update_navigation_supervisor(good, 10.0)
        controller._update_navigation_supervisor(bad, 10.1)
        controller._update_navigation_supervisor(bad, 11.2)
        self.assertEqual(controller.navigation_state, "HOLD")

        controller.phase_clock_s = 5.0
        controller._advance_phase_clock(0.05)
        self.assertEqual(controller.phase_clock_s, 5.0)

        controller._update_navigation_supervisor(good, 11.3)
        controller._update_navigation_supervisor(good, 12.0)
        self.assertEqual(controller.navigation_state, "HOLD")
        controller._update_navigation_supervisor(good, 13.4)
        self.assertEqual(controller.navigation_state, "HEALTHY")
        controller._advance_phase_clock(0.05)
        self.assertGreater(controller.phase_clock_s, 5.0)

    def test_invalid_local_estimate_yields_to_px4(self):
        controller = self.make_controller()
        bad = self.navigation_snapshot(controller)
        bad["estimator_flags"] = 0
        controller._update_navigation_supervisor(bad, 10.0)
        with self.assertRaises(NavigationEstimateLost):
            controller._update_navigation_supervisor(bad, 10.3)
        self.assertEqual(controller.failure_action, "PX4_FAILSAFE")

    def test_slowed_clock_scales_trajectory_feedforward(self):
        controller = self.make_controller()
        controller.phase = "TAKEOFF"
        controller.navigation_confidence = 0.5
        controller.takeoff_segment = MinimumJerkSegment.from_limits(
            (0.0, 0.0, 0.0), (0.0, 0.0, -20.0),
            max_speed=0.8, max_accel=1.0, max_jerk=1.5,
        )
        controller.phase_clock_s = controller.takeoff_segment.duration / 2
        target = controller.takeoff_segment.sample(controller.phase_clock_s)
        control = controller.takeoff_controller(
            0.0, 0.0, target.z, 0.0, 0.05
        )
        expected_vz = target.vz * controller._navigation_speed_scale()
        self.assertAlmostEqual(control["planned"][2], expected_vz)
        self.assertAlmostEqual(control["command"][2], expected_vz)

    def test_analytic_peak_matches_dense_independent_sampling(self):
        segment = MinimumJerkSegment.from_limits(
            (4.0, -2.0, -19.0), (0.0, 0.0, -20.0),
            max_speed=2.0, max_accel=1.0, max_jerk=1.5,
            start_velocity=(0.5, -0.2, 0.1),
        )
        analytic = segment.peak_kinematics()
        dense = [0.0, 0.0, 0.0]
        for index in range(5001):
            point = segment.sample(index * segment.duration / 5000)
            for axis, components in enumerate((
                (point.vx, point.vy, point.vz),
                (point.ax, point.ay, point.az),
                (point.jx, point.jy, point.jz),
            )):
                dense[axis] = max(dense[axis], math.hypot(*components))
        for peak, sampled in zip(analytic, dense, strict=True):
            self.assertGreaterEqual(peak + 1e-8, sampled)
            self.assertAlmostEqual(peak, sampled, delta=0.001)


if __name__ == "__main__":
    unittest.main(verbosity=2)
