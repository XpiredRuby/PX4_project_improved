#!/usr/bin/env python3
# ruff: noqa: E402
"""Tests for expected-failsafe evidence checks."""

import sys
import unittest
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "analysis"))

from analyze_fault_run import (
    fault_phase_check,
    failsafe_manifest_checks,
    hold_evidence,
    navigation_fault_check,
    navigation_recovery_check,
    recovery_manifest_checks,
)


class FaultRunAnalysisTests(unittest.TestCase):
    def test_expected_failsafe_sequence_accepts_valid_prefix(self):
        frame = pd.DataFrame({
            "phase": [
                "TAKEOFF",
                "TAKEOFF",
                "TRAJECTORY",
                "PX4_FAILSAFE",
                "PX4_FAILSAFE",
            ],
            "navigation_state": [
                "HEALTHY",
                "DEGRADED",
                "HOLD",
                "HOLD",
                "HOLD",
            ],
        })

        self.assertTrue(fault_phase_check(frame)[0])
        self.assertTrue(navigation_fault_check(frame)[0])

    def test_failsafe_sequence_rejects_resumed_offboard_phase(self):
        frame = pd.DataFrame({
            "phase": ["TAKEOFF", "PX4_FAILSAFE", "TRAJECTORY"],
        })

        self.assertFalse(fault_phase_check(frame)[0])

    def test_hold_evidence_requires_frozen_clock_and_bounded_drift(self):
        frame = pd.DataFrame({
            "phase": ["TAKEOFF", "TAKEOFF", "PX4_FAILSAFE"],
            "navigation_state": ["HOLD", "HOLD", "HOLD"],
            "elapsed_s": [1.0, 2.0, 3.0],
            "phase_clock_s": [0.4, 0.4, 0.4],
            "x": [2.0, 2.1, 2.1],
            "y": [-1.0, -1.1, -1.1],
            "z": [-10.0, -10.2, -9.0],
        })

        passed, _, metrics = hold_evidence(frame)

        self.assertTrue(passed)
        self.assertAlmostEqual(metrics["duration_s"], 1.0)
        self.assertLess(metrics["max_xy_drift_m"], 0.2)

    def test_manifest_requires_full_landing_cleanup(self):
        safe = {
            "outcome": "PX4_FAILSAFE",
            "cleanup_status": "px4_failsafe_land_and_disarm_confirmed",
            "cleanup_error": None,
            "final_state": {
                "armed": False,
                "landed_state": 1,
                "failure_action": "PX4_FAILSAFE",
            },
        }
        unsafe = {
            **safe,
            "cleanup_status": "px4_failsafe_takeover_confirmed",
            "final_state": {**safe["final_state"], "armed": True},
        }

        self.assertTrue(
            all(item["passed"] for item in failsafe_manifest_checks(safe))
        )
        self.assertFalse(
            all(item["passed"] for item in failsafe_manifest_checks(unsafe))
        )

    def test_recovery_requires_hold_then_stable_health(self):
        recovered = pd.DataFrame({
            "navigation_state": [
                "HEALTHY",
                "DEGRADED",
                "HOLD",
                "HOLD",
                "HEALTHY",
            ],
        })
        incomplete = pd.DataFrame({
            "navigation_state": ["HEALTHY", "DEGRADED", "HEALTHY"],
        })

        self.assertTrue(navigation_recovery_check(recovered)[0])
        self.assertFalse(navigation_recovery_check(incomplete)[0])

    def test_recovery_manifest_requires_success_ground_and_disarm(self):
        safe = {
            "outcome": "SUCCESS",
            "cleanup_error": None,
            "final_state": {"armed": False, "landed_state": 1},
        }
        unsafe = {
            **safe,
            "final_state": {"armed": True, "landed_state": 2},
        }

        self.assertTrue(
            all(item["passed"] for item in recovery_manifest_checks(safe))
        )
        self.assertFalse(
            all(item["passed"] for item in recovery_manifest_checks(unsafe))
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
