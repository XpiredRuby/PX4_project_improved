#!/usr/bin/env python3
"""Unit tests for mentor-facing plot helpers."""

import unittest
import sys
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))

from presentation_plots import FIXED_LABEL, fixed_run, phase_spans


class PresentationPlotTests(unittest.TestCase):
    def test_fixed_run_selects_only_validated_profile(self):
        frame = pd.DataFrame(
            {
                "run_label": ["formal_baseline", FIXED_LABEL, FIXED_LABEL],
                "value": [1, 2, 3],
            }
        )
        selected = fixed_run(frame)
        self.assertEqual(selected["value"].tolist(), [2, 3])

    def test_phase_spans_preserves_phase_order(self):
        frame = pd.DataFrame(
            {
                "phase": ["TAKEOFF", "TAKEOFF", "TRAJECTORY", "LAND"],
                "elapsed_s": [0.0, 1.0, 2.0, 3.0],
            }
        )
        self.assertEqual(
            phase_spans(frame),
            [("TAKEOFF", 0.0, 1.0), ("TRAJECTORY", 2.0, 2.0), ("LAND", 3.0, 3.0)],
        )

    def test_fixed_run_rejects_missing_profile(self):
        with self.assertRaisesRegex(ValueError, "No rows"):
            fixed_run(pd.DataFrame({"run_label": ["formal_baseline"]}))


if __name__ == "__main__":
    unittest.main(verbosity=2)
