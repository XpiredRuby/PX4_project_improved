#!/usr/bin/env python3
# ruff: noqa: E402
"""Tests for machine-readable mission manifests."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))

from mission_state import MissionOutcome
from PID_position_new import PositionController
from run_record import RunRecord


class RunRecordTests(unittest.TestCase):
    def test_manifest_records_context_and_final_outcome_atomically(self):
        controller = PositionController()
        with tempfile.TemporaryDirectory() as directory:
            record = RunRecord(output_dir=directory, run_id="test-run")
            record.attach_context(
                controller,
                {"COM_OF_LOSS_T": 0.5, "COM_OBL_RC_ACT": 4.0},
            )
            record.finalize(
                MissionOutcome.SUCCESS,
                "touchdown confirmed",
                {
                    "armed": False,
                    "landed_state": 1,
                    "mission_phase": "HANDOFF",
                },
            )

            data = json.loads(record.path.read_text(encoding="utf-8"))
            self.assertEqual(data["schema_version"], 1)
            self.assertEqual(data["run_id"], "test-run")
            self.assertEqual(data["outcome"], "SUCCESS")
            self.assertEqual(data["reason"], "touchdown confirmed")
            self.assertFalse(data["final_state"]["armed"])
            self.assertIsNone(data["random_seed"])
            self.assertFalse(data["trajectory_randomized"])
            self.assertEqual(data["px4_parameters"]["COM_OF_LOSS_T"], 0.5)
            self.assertIn("mission_timeout_s", data["controller_config"])
            self.assertEqual(len(data["source_sha256"]["trajectory.csv"]), 64)
            self.assertEqual(
                data["controller_runtime"]["trajectory_path"],
                str(controller.trajectory_path),
            )
            self.assertEqual(
                len(data["controller_runtime"]["trajectory_sha256"]), 64
            )
            self.assertFalse(record.path.with_suffix(".json.tmp").exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
