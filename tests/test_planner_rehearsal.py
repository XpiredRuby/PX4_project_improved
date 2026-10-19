"""Independent truth and missing-evidence regressions for rehearsal scoring."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from planner_rehearsal import assess_frames  # noqa: E402


class RehearsalTests(unittest.TestCase):
    def evidence(self):
        time = np.arange(0., 2.01, .05)
        phase = ["TRAJECTORY"] * 21 + ["PX4_LAND"] * 20
        position = np.zeros((len(time), 3))
        df = pd.DataFrame({"elapsed_s": time, "wall_time": 1000+time,
                           "position_time_boot_ms": time*1000, "phase": phase,
                           "mission_time_s": np.minimum(time, 1.), "target_yaw_unwrapped": 0.,
                           "desired_x": 0., "desired_y": 0., "desired_z": 0., "target_yaw": 0.,
                           "landed_state": [2]*30+[1]*11,
                           "native_landing_reference_x": 0., "native_landing_reference_y": 0.})
        plan = pd.DataFrame({"time": [0., 1.], "x": 0., "y": 0., "z": 0., "yaw": 0.})
        return [df, plan, position, np.zeros(len(time)), np.zeros_like(position),
                {"overall_passed": True}, {"armed": False, "landed_state": 1,
                                           "actuators": [0.]*4, "continuous_confirmation_s": 2.}]

    def test_complete_truth_and_lifecycle_pass_only_sampled_sitl_scope(self):
        result = assess_frames(*self.evidence())
        self.assertTrue(result["sampled_rehearsal_passed"])
        self.assertFalse(result["hardware_approval"])
        self.assertIn("wall-time", result["timing_scope"])

    def test_truth_error_cannot_be_hidden_by_perfect_estimated_reference(self):
        for kind in ("position", "heading", "velocity", "missing"):
            args = self.evidence()
            if kind == "position":
                args[2][5, 0] = 1.1
            elif kind == "heading":
                args[3][5] = np.radians(11.)
            elif kind == "velocity":
                args[4][30, 2] = .21
            else:
                args[2][10] = np.nan
            with self.subTest(kind=kind):
                self.assertFalse(assess_frames(*args)["sampled_rehearsal_passed"])

    def test_partial_route_rewritten_reference_and_unconfirmed_shutdown_fail(self):
        for kind in ("partial", "reference", "yaw", "shutdown", "nan_output", "clock"):
            args = deepcopy(self.evidence())
            if kind == "partial":
                args[1].loc[1, "time"] = 2.
            elif kind == "reference":
                args[0].loc[8, "desired_x"] = .01
            elif kind == "yaw":
                args[0].loc[8, "target_yaw_unwrapped"] = .01
            elif kind == "shutdown":
                args[-1]["continuous_confirmation_s"] = 1.99
            elif kind == "nan_output":
                args[-1]["actuators"][2] = float("nan")
            else:
                args[0]["position_time_boot_ms"] *= .5
            with self.subTest(kind=kind):
                self.assertFalse(assess_frames(*args)["sampled_rehearsal_passed"])


if __name__ == "__main__":
    unittest.main()
