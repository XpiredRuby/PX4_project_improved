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
)


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
        })

    def test_safety_audit_passes_complete_bounded_run(self):
        audit = build_safety_audit(self.successful_run_frame())
        self.assertTrue(audit["overall_passed"])
        self.assertTrue(all(item["passed"] for item in audit["checks"]))

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
            },
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
