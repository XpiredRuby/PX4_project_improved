#!/usr/bin/env python3
"""Regression tests for derived PX4 state-analysis signals."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))

from analyze_run import (
    body_specific_force_to_ned,
    build_safety_audit,
    enrich_derived_signals,
    quaternion_to_euler,
    sampled_reference_derivatives,
)
from analyze_fault_run import build_navigation_response_audit


class AnalysisPipelineTests(unittest.TestCase):
    def test_identity_quaternion_converts_to_zero_euler(self):
        roll, pitch, yaw = quaternion_to_euler(
            [1.0], [0.0], [0.0], [0.0]
        )
        np.testing.assert_allclose([roll[0], pitch[0], yaw[0]], 0.0, atol=1e-12)

    def test_quaternion_yaw_conversion(self):
        half = math.pi / 4.0
        roll, pitch, yaw = quaternion_to_euler(
            [math.cos(half)], [0.0], [0.0], [math.sin(half)]
        )
        np.testing.assert_allclose([roll[0], pitch[0]], 0.0, atol=1e-12)
        self.assertAlmostEqual(yaw[0], math.pi / 2.0, places=12)

    def test_level_stationary_specific_force_becomes_zero_ned_acceleration(self):
        ax, ay, az = body_specific_force_to_ned(
            [0.0], [0.0], [0.0], [0.0], [0.0], [-9.80665]
        )
        np.testing.assert_allclose([ax[0], ay[0], az[0]], 0.0, atol=1e-12)

    def test_enrichment_adds_finite_attitude_and_acceleration(self):
        frame = pd.DataFrame({
            "attitude_target_q0": [1.0] * 5,
            "attitude_target_q1": [0.0] * 5,
            "attitude_target_q2": [0.0] * 5,
            "attitude_target_q3": [0.0] * 5,
            "roll": [0.0] * 5,
            "pitch": [0.0] * 5,
            "yaw": [0.0] * 5,
            "imu_xacc": [0.0] * 5,
            "imu_yacc": [0.0] * 5,
            "imu_zacc": [-9.80665] * 5,
        })
        enrich_derived_signals(frame)
        expected = {
            "px4_target_roll", "px4_target_pitch", "px4_target_yaw_from_q",
            "derived_actual_ax_ned_raw", "derived_actual_ay_ned_raw",
            "derived_actual_az_ned_raw", "derived_actual_ax_ned_filtered",
            "derived_actual_ay_ned_filtered", "derived_actual_az_ned_filtered",
        }
        self.assertTrue(expected.issubset(frame.columns))
        self.assertTrue(np.isfinite(frame[list(expected)].to_numpy()).all())

    @staticmethod
    def successful_run_frame():
        phases = [
            "TAKEOFF",
            "TRAJECTORY",
            "RETURN_HOME",
            "ALIGN",
            "HANDOFF",
            "PX4_LAND",
        ]
        return pd.DataFrame({
            "phase": phases,
            "armed": [True, True, True, True, True, False],
            "landed_state": [1, 2, 2, 2, 4, 1],
            "cmd_vx": [0.0, 1.5, -1.0, 0.1, 0.0, np.nan],
            "cmd_vy": [0.0, 0.5, 0.2, 0.0, 0.0, np.nan],
            "cmd_vz": [-0.5, 0.0, 0.2, 0.0, 0.0, np.nan],
            "effective_horizontal_speed_limit": [3.0] * 6,
            "effective_vertical_speed_limit": [1.0] * 6,
            "setpoint_max_gap_s": [0.05] * 6,
            "offboard_stream_max_gap_s": [0.5] * 6,
            "position_source_regressed": [False] * 6,
            "gps_source_regressed": [False] * 6,
            "navigation_state": ["HEALTHY"] * 6,
            "elapsed_s": [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
            "x": [0.0] * 6,
            "y": [0.0] * 6,
            "desired_x": [0.0] * 6,
            "desired_y": [0.0] * 6,
            "vx": [0.0] * 6,
            "vy": [0.0] * 6,
            "vz": [0.0] * 6,
            "roll": [0.0] * 6,
            "pitch": [0.0] * 6,
            "actuator_age_s": [0.01] * 6,
            "actuator_output_0": [0.2] * 5 + [0.0],
            "actuator_output_1": [0.2] * 5 + [0.0],
            "actuator_output_2": [0.2] * 5 + [0.0],
            "actuator_output_3": [0.2] * 5 + [0.0],
        })

    def test_safety_audit_passes_complete_bounded_run(self):
        audit = build_safety_audit(self.successful_run_frame())
        self.assertTrue(audit["overall_passed"])
        self.assertTrue(all(item["passed"] for item in audit["checks"]))

    def test_reference_acceleration_uses_retimed_velocity_and_irregular_real_time(self):
        times = np.array([0., .04, .10, .15, .23])
        frame = pd.DataFrame({"elapsed_s": times, "planned_vx": 2. * times,
                              "planned_vy": 0., "planned_vz": .2,
                              "planned_ax": 8., "phase": "TRAJECTORY"})
        enrich_derived_signals(frame)
        np.testing.assert_allclose(frame.derived_reference_ax_ned, 2., atol=1e-12)
        np.testing.assert_allclose(frame.derived_reference_jx_ned, 0., atol=1e-10)
        np.testing.assert_allclose(frame.planned_ax, 8.)

    def test_reference_derivatives_do_not_bridge_hold_native_land_or_gaps(self):
        frame = pd.DataFrame({"elapsed_s": np.arange(9) * .05,
                              "planned_vx": [1.] * 3 + [0.] * 3 + [np.nan] * 3,
                              "planned_vy": 0., "planned_vz": 0.,
                              "phase": ["TRAJECTORY"] * 6 + ["PX4_LAND"] * 3,
                              "navigation_state": ["HEALTHY"] * 3 + ["HOLD"] * 6})
        result = sampled_reference_derivatives(frame)
        np.testing.assert_allclose(result["derived_reference_ax_ned"][:6], 0., atol=1e-12)
        self.assertTrue(np.isnan(result["derived_reference_ax_ned"][6:]).all())
        frame = frame.iloc[:6].copy()
        frame["elapsed_s"] = [0., .05, .1, .5, .55, .6]
        frame["navigation_state"] = "HEALTHY"
        result = sampled_reference_derivatives(frame)
        np.testing.assert_allclose(result["derived_reference_ax_ned"], 0., atol=1e-12)
        frame.loc[frame.index[-1], "elapsed_s"] = .4
        with self.assertRaises(ValueError):
            sampled_reference_derivatives(frame)

    def test_navigation_response_keeps_hold_bounds_and_quality_verdict(self):
        frame = self.successful_run_frame().iloc[[0, 1, 1, 1, 1, 2, 3, 4, 5]].reset_index(drop=True)
        frame["elapsed_s"] = np.arange(len(frame), dtype=float)
        frame["phase_clock_s"] = 0.
        frame["z"] = -10.
        frame.loc[1, "navigation_state"] = "DEGRADED"
        frame.loc[2:3, "navigation_state"] = "HOLD"
        frame.loc[3, "x"] = .3
        manifest = {"outcome": "SUCCESS", "cleanup_error": None,
                    "final_state": {"armed": False, "landed_state": 1}}
        self.assertTrue(build_navigation_response_audit(frame, manifest)["overall_passed"])
        frame.loc[1, "navigation_state"] = "HOLD"
        self.assertTrue(build_navigation_response_audit(frame, manifest)["overall_passed"])
        self.assertFalse(build_navigation_response_audit(frame, manifest, allow_recovery=False)["overall_passed"])
        frame.loc[3, "x"] = 1.75
        self.assertFalse(build_navigation_response_audit(frame, manifest)["overall_passed"])
        frame.loc[3, "x"] = .3
        manifest["outcome"] = "UNSAFE_TOUCHDOWN"
        self.assertFalse(build_navigation_response_audit(frame, manifest)["overall_passed"])

    def test_navigation_timeout_landing_requires_specific_reason_and_cleanup(self):
        frame = self.successful_run_frame().iloc[[0, 1, 1, 1, 1, 5]].reset_index(drop=True)
        frame["elapsed_s"] = np.arange(len(frame), dtype=float)
        frame["phase_clock_s"] = 0.
        frame["z"] = -10.
        frame.loc[1, "navigation_state"] = "DEGRADED"
        frame.loc[2:3, "navigation_state"] = "HOLD"
        frame.loc[4:, "navigation_state"] = "LOST"
        manifest = {"outcome": "ABORTED_TO_LAND", "cleanup_error": None,
                    "reason": "RuntimeError: Navigation remained unhealthy for 8.0s",
                    "cleanup_status": "px4_land_and_disarm_confirmed",
                    "final_state": {"armed": False, "landed_state": 1, "failure_action": "LAND"}}
        self.assertTrue(build_navigation_response_audit(frame, manifest, allow_recovery=False)["overall_passed"])
        for change in ({"cleanup_status": "failed"}, {"reason": "Other controller fault"},
                       {"cleanup_error": "landing timed out"}):
            self.assertFalse(build_navigation_response_audit(frame, dict(manifest, **change))["overall_passed"])

    def test_safety_audit_exposes_command_and_completion_failures(self):
        frame = self.successful_run_frame()
        frame.loc[1, "cmd_vx"] = 3.5
        frame.loc[5, "armed"] = True
        frame.loc[2, "gps_source_regressed"] = True

        audit = build_safety_audit(frame)
        failed = {
            item["name"]
            for item in audit["checks"]
            if not item["passed"]
        }

        self.assertFalse(audit["overall_passed"])
        self.assertEqual(
            failed,
            {
                "Automatic disarm confirmed",
                "Command envelope respected",
                "No telemetry time regression",
                "Automatic disarm timing",
                "Propulsion outputs zero after disarm",
            },
        )

    def test_failed_failsafe_landing_preserves_measured_quality_and_shutdown(self):
        frame = self.successful_run_frame()
        frame.loc[5, "phase"] = "PX4_FAILSAFE"
        frame.loc[5, "x"] = 29.0
        manifest = {"outcome": "UNSAFE_TOUCHDOWN"}

        audit = build_navigation_response_audit(frame, manifest)
        checks = {item["name"]: item for item in audit["checks"]}
        self.assertFalse(audit["overall_passed"])
        self.assertFalse(checks["Expected navigation response"]["passed"])
        self.assertFalse(checks["Nominal phase sequence"]["passed"])
        self.assertFalse(checks["Touchdown position within limit"]["passed"])
        self.assertIn("error=29.000m", checks["Touchdown position within limit"]["detail"])
        for name in ("Touchdown dynamics within limits", "No post-touchdown bounce",
                     "Automatic disarm timing", "Propulsion outputs zero after disarm"):
            self.assertTrue(checks[name]["passed"], checks[name])

    def test_failed_failsafe_without_contact_still_fails_completion_checks(self):
        frame = self.successful_run_frame()
        frame.loc[5, "phase"] = "PX4_FAILSAFE"
        frame.loc[5, "landed_state"] = 2
        audit = build_navigation_response_audit(frame, {"outcome": "UNSAFE_TOUCHDOWN"})
        checks = {item["name"]: item for item in audit["checks"]}
        self.assertFalse(audit["overall_passed"])
        self.assertFalse(checks["Touchdown position within limit"]["passed"])
        self.assertFalse(checks["Propulsion outputs zero after disarm"]["passed"])
        self.assertIn("no native landing ON_GROUND sample",
                      checks["Touchdown position within limit"]["detail"])

    def test_safety_audit_rejects_unsafe_touchdown_and_live_propulsion(self):
        frame = self.successful_run_frame()
        frame.loc[5, "x"] = 2.0
        frame.loc[5, "vx"] = 0.8
        frame.loc[5, "vz"] = 0.7
        frame.loc[5, "roll"] = math.radians(20.0)
        frame.loc[5, "actuator_output_0"] = 0.2

        audit = build_safety_audit(frame)
        failed = {
            item["name"]
            for item in audit["checks"]
            if not item["passed"]
        }

        self.assertFalse(audit["overall_passed"])
        self.assertEqual(
            failed,
            {
                "Touchdown dynamics within limits",
                "Touchdown position within limit",
                "Propulsion outputs zero after disarm",
            },
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
