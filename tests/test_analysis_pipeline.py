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


if __name__ == "__main__":
    unittest.main(verbosity=2)
