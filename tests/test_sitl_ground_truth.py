import math
import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from sitl_ground_truth import EARTH_RADIUS_M, audit_ground_truth, geodetic_to_ned, ordered_samples  # noqa: E402


class GroundTruthTests(unittest.TestCase):
    def test_invalid_truth_clocks_reject_alignment(self):
        df = pd.DataFrame({"position_time_boot_ms": [1.]})
        for invalid in ([], [float("nan")], [float("inf")], [2000., 1000.]):
            with self.subTest(times=invalid):
                with self.assertRaisesRegex(ValueError, "ordered and finite"):
                    audit_ground_truth(df, {"timestamp": invalid}, {"timestamp": [1000.]})
                with self.assertRaisesRegex(ValueError, "ordered and finite"):
                    audit_ground_truth(df, {"timestamp": [1000.], "lat": [47.],
                                          "lon": [8.], "alt": [500.]}, {"timestamp": invalid})

    def test_identical_source_duplicates_collapse_but_conflicting_values_reject(self):
        data = {"timestamp": [1000., 1000., 2000.], "alt": [500., 500., 501.]}
        times, values, duplicates = ordered_samples(data, ("alt",))
        np.testing.assert_array_equal(times, [1000., 2000.])
        np.testing.assert_array_equal(values["alt"], [500., 501.])
        self.assertEqual(duplicates, 1)
        data["alt"][1] = 502.
        with self.assertRaisesRegex(ValueError, "conflicting"):
            ordered_samples(data, ("alt",))
        for bad in ([500.], [500., math.nan, 501.]):
            data["alt"] = bad
            with self.assertRaisesRegex(ValueError, "finite value per timestamp"):
                ordered_samples(data, ("alt",))

    def test_known_north_east_up_displacements(self):
        north_lat = 47. + np.degrees(1. / EARTH_RADIUS_M)
        east_lon = 8. + np.degrees(2. / (EARTH_RADIUS_M * np.cos(np.radians(47.))))
        result = geodetic_to_ned(north_lat, east_lon, 503., 47., 8., 500.)[0]
        np.testing.assert_allclose(result, [1., 2., -3.], atol=1e-8)

    def test_uncovered_truth_is_not_extrapolated(self):
        df = pd.DataFrame({"position_time_boot_ms": [0., 1., 2., 3.],
                           "phase": "TRAJECTORY", "desired_x": 0., "desired_y": 0.,
                           "desired_z": 0., "x": 0., "y": 0., "z": 0.})
        truth = {"timestamp": np.array([1000., 2000.]), "lat": [47., 47.],
                 "lon": [8., 8.], "alt": [500., 500.]}
        local = {"timestamp": np.array([1000., 2000.]), "ref_lat": [47., 47.],
                 "ref_lon": [8., 8.], "ref_alt": [500., 500.]}
        result, physical = audit_ground_truth(df, truth, local)
        self.assertEqual(result["coverage"]["covered_rows"], 2)
        self.assertTrue(np.all(np.isnan(physical[[0, 3]])))
        self.assertEqual(result["phases"]["TRAJECTORY"]["physical_xy_tracking_error_m"]["rms"], 0.)
        df["phase"] = "PX4_FAILSAFE"
        result, _ = audit_ground_truth(df, truth, local)
        native = result["phases"]["PX4_FAILSAFE"]
        self.assertFalse(native["offboard_reference_active"])
        self.assertIsNone(native["physical_xy_tracking_error_m"])
        self.assertIsNone(native["physical_z_tracking_error_m"])
        self.assertEqual(native["estimator_xy_error_m"]["rms"], 0.)
