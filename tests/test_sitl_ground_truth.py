import math
import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from sitl_ground_truth import (  # noqa: E402
    EARTH_RADIUS_M, audit_ground_truth, audit_physical_position,
    geodetic_to_ned, interpolation_coverage, ordered_samples,
)


class GroundTruthTests(unittest.TestCase):
    @staticmethod
    def contact_frame():
        return pd.DataFrame({"phase": ["TRAJECTORY", "TRAJECTORY", "PX4_LAND", "PX4_LAND"],
                             "landed_state": [2, 2, 2, 1],
                             "navigation_state": ["HOLD", "HOLD", "HEALTHY", "HEALTHY"],
                             "native_landing_reference_x": [math.nan, math.nan, 0., 0.],
                             "native_landing_reference_y": [math.nan, math.nan, 0., 0.]})

    def test_independent_position_fails_even_when_estimator_reports_zero_error(self):
        df = self.contact_frame()
        df["x"] = df["y"] = 0.
        truth = np.array([[0., 0., -5.], [.1, 0., -5.], [0., 0., -1.], [28., 0., 0.]])
        result = audit_physical_position(df, truth)
        self.assertFalse(result["overall_passed"])
        self.assertFalse(next(x for x in result["checks"] if x["name"] ==
                              "Physical touchdown position within limit")["passed"])
        truth[-1, 0] = .1
        self.assertTrue(audit_physical_position(df, truth)["overall_passed"])

    def test_missing_truth_contact_and_changed_landing_reference_cannot_pass(self):
        for kind in ("missing_truth", "no_contact", "changed_reference", "missing_reference"):
            with self.subTest(kind=kind):
                df = self.contact_frame()
                truth = np.zeros((4, 3))
                if kind == "missing_truth":
                    truth[-1] = np.nan
                elif kind == "no_contact":
                    df["landed_state"] = 2
                elif kind == "changed_reference":
                    df.loc[3, "native_landing_reference_x"] = 28.
                    truth[-1, 0] = 28.
                else:
                    df = df.drop(columns="native_landing_reference_x")
                self.assertFalse(audit_physical_position(df, truth)["overall_passed"])

    def test_each_hold_episode_requires_covered_bounded_physical_motion(self):
        for axis in (0, 2):
            df = self.contact_frame()
            truth = np.zeros((4, 3))
            truth[1, axis] = 1.01
            self.assertFalse(audit_physical_position(df, truth)["overall_passed"])
            truth[1, axis] = np.nan
            self.assertFalse(audit_physical_position(df, truth)["overall_passed"])
        df = pd.concat([self.contact_frame().iloc[:2], self.contact_frame()], ignore_index=True)
        df.loc[2, "navigation_state"] = "HEALTHY"
        truth = np.zeros((6, 3))
        truth[:2, 0] = 20.
        self.assertTrue(audit_physical_position(df, truth)["overall_passed"])

    def test_long_truth_gaps_are_not_bridged_but_exact_samples_are_usable(self):
        times = np.array([1_000., 1_001_000.])
        actual = interpolation_coverage(np.array([0., 1_000., 500_000., 1_001_000., 1_002_000.]), times)
        np.testing.assert_array_equal(actual, [False, True, False, True, False])

    def test_local_origin_reset_is_held_until_reported_not_interpolated(self):
        df = pd.DataFrame({"position_time_boot_ms": [100., 200.], "phase": "TRAJECTORY",
                           "desired_x": 0., "desired_y": 0., "desired_z": 0.,
                           "x": 0., "y": 0., "z": 0.})
        truth = {"timestamp": [0., 200_000.], "lat": [47., 47.],
                 "lon": [8., 8.], "alt": [500., 500.]}
        local = {"timestamp": [0., 200_000.], "ref_lat": [47., 47.],
                 "ref_lon": [8., 8.], "ref_alt": [500., 510.]}
        _, physical = audit_ground_truth(df, truth, local)
        np.testing.assert_allclose(physical[:, 2], [0., 10.])

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
        self.assertFalse(result["overall_passed"])
        self.assertFalse(native["offboard_reference_active"])
        self.assertIsNone(native["physical_xy_tracking_error_m"])
        self.assertIsNone(native["physical_z_tracking_error_m"])
        self.assertEqual(native["estimator_xy_error_m"]["rms"], 0.)
