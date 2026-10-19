import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from trajectory_quality import trajectory_quality  # noqa: E402


class TrajectoryQualityTests(unittest.TestCase):
    def frame(self):
        t = np.arange(10) * 0.05
        return pd.DataFrame({"elapsed_s": t, "phase": "TRAJECTORY",
                             "x": t, "y": 0., "z": 0.,
                             "desired_x": t + .1, "desired_y": 0.,
                             "desired_z": 0., "cmd_vx": t,
                             "cmd_vy": 0., "cmd_vz": 0.})

    def test_known_error_acceleration_and_jerk(self):
        result = trajectory_quality(self.frame())["phases"]["TRAJECTORY"]
        self.assertAlmostEqual(result["xy_error_m"]["rms"], .1)
        self.assertAlmostEqual(result["command_xy_accel_m_s2"]["rms"], 1.)
        self.assertLess(result["command_xy_jerk_m_s3"]["max_abs"], 1e-10)

    def test_do_not_differentiate_across_phase_boundary_or_gap(self):
        frame = self.frame()
        frame.loc[5:, "phase"] = "PX4_LAND"
        frame.loc[5:, "cmd_vx"] += 100.
        result = trajectory_quality(frame)
        self.assertAlmostEqual(result["phases"]["TRAJECTORY"]["command_xy_accel_m_s2"]["max_abs"], 1.)
        native = result["phases"]["PX4_LAND"]
        self.assertFalse(native["offboard_reference_active"])
        self.assertIsNone(native["command_xy_accel_m_s2"])
        self.assertIsNone(native["xy_error_m"])
        self.assertIsNone(native["z_error_m"])
        self.assertIsNone(native["command_acceleration_effort_m2_s3"])
        self.assertGreater(native["estimated_path_length_m"], 0.)
        frame.loc[5:, "elapsed_s"] += 2.
        self.assertEqual(trajectory_quality(frame)["sampling"]["gaps_over_0_25_s"], 1)

    def test_reject_duplicate_time(self):
        frame = self.frame()
        frame.loc[1, "elapsed_s"] = 0.
        with self.assertRaises(ValueError):
            trajectory_quality(frame)
