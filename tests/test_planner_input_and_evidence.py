"""Adversarial input and evidence-coverage regressions."""

from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import unittest

from mission_planner.contracts import load_contract
from mission_planner.plan_interview import intent_from_interview
from mission_planner.sitl_campaign import compare_flown_track


ROOT = Path(__file__).resolve().parents[1]


class InterviewBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.raw = json.loads((ROOT / "missions/interview_circle_example.json").read_text())

    def test_loop_count_and_version_do_not_coerce_types(self):
        for field, values in (
            ("loop_count", [True, False, 1.5, 1.0, "2", None]),
            ("schema_version", [True, 1.0, "1"]),
        ):
            for value in values:
                with self.subTest(field=field, value=value):
                    raw = dict(self.raw, **{field: value})
                    with self.assertRaises(ValueError):
                        intent_from_interview(raw)

    def test_unused_or_unknown_pattern_fields_are_rejected(self):
        for raw in (
            dict(self.raw, circle={**self.raw["circle"], "radius_typo": 3}),
            dict(self.raw, survey={"length_m": 5}),
            {key: value for key, value in self.raw.items() if key != "circle"},
        ):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                intent_from_interview(raw)

    def test_waypoint_flags_must_be_actual_booleans(self):
        raw = deepcopy(self.raw)
        raw.pop("circle")
        raw["pattern"] = "waypoints"
        raw["waypoints"] = [
            {"north_m": 0, "east_m": 0},
            {"north_m": 5, "east_m": 0},
        ]
        for field in ("hard", "exact"):
            for value in ("false", 1, 0, None):
                with self.subTest(field=field, value=value):
                    bad = deepcopy(raw)
                    bad["waypoints"][1][field] = value
                    with self.assertRaisesRegex(ValueError, "boolean"):
                        intent_from_interview(bad)
        intent, loops = intent_from_interview(raw)
        self.assertEqual(loops, 1)
        self.assertIs(intent["path"]["points"][1]["hard"], False)
        with self.assertRaises(ValueError):
            intent_from_interview(dict(raw, loop_count=2))

    def test_normalized_interview_is_validated_before_return(self):
        for change in (
            {"maximum_speed_m_s": math.nan},
            {"yaw_mode": "fixed", "fixed_yaw_deg": None},
            {"time_mode": "exact", "requested_duration_s": None},
        ):
            with self.subTest(change=change), self.assertRaises(ValueError):
                intent_from_interview(dict(self.raw, **change))

    def test_generated_survey_budget_is_checked_before_expansion(self):
        raw = dict(self.raw)
        raw.pop("circle")
        raw["pattern"] = "survey"
        raw["survey"] = {"length_m": 10, "width_m": 1e12, "lane_spacing_m": .1}
        with self.assertRaisesRegex(ValueError, "point budget"):
            intent_from_interview(raw)

    def test_json_duplicate_keys_and_nonfinite_constants_are_rejected(self):
        for document in ('{"x":1,"x":2}', '{"nested":{"a":1,"a":2}}',
                         '{"x":NaN}', '{"x":Infinity}'):
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "input.json"
                path.write_text(document)
                with self.subTest(document=document), self.assertRaises(ValueError):
                    load_contract(path, lambda raw: raw)


class TrackEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.plan = self.root / "plan.csv"
        self.flown = self.root / "flown.csv"
        self.write(self.plan, [0, .25, .5, .75, 1.])

    @staticmethod
    def write(path, times):
        path.write_text("time,x,y,z,yaw\n" + "".join(
            f"{time},{time},0,-8,0\n" for time in times
        ))

    def compare(self, **changes):
        return compare_flown_track(self.plan, self.flown, **{
            "maximum_position_error_m": .2,
            "maximum_yaw_error_rad": .1,
            **changes,
        })

    def test_partial_track_cannot_pass_by_having_zero_error(self):
        for times in ([.25, .5, .75], [0, .25], [.75, 1.]):
            self.write(self.flown, times)
            with self.subTest(times=times):
                report = self.compare()
                self.assertEqual(report["position_error_m"], 0.)
                self.assertFalse(report["coverage_passed"])
                self.assertFalse(report["accepted"])

    def test_endpoint_only_track_cannot_hide_missing_middle(self):
        self.write(self.flown, [0., 1.])
        report = self.compare()
        self.assertTrue(report["coverage_passed"])
        self.assertFalse(report["sample_gaps_passed"])
        self.assertFalse(report["accepted"])

    def test_complete_track_pass_is_scoped_to_sampled_tracking(self):
        self.write(self.flown, [0, .25, .5, .75, 1.])
        report = self.compare()
        self.assertTrue(report["accepted"])
        self.assertFalse(report["campaign_ran"])
        self.assertIn("sampled", report["verification_scope"])

    def test_invalid_thresholds_cannot_relax_acceptance(self):
        self.write(self.flown, [0., 1.])
        for field in ("maximum_position_error_m", "maximum_yaw_error_rad",
                      "maximum_sample_gap_s"):
            for value in (math.nan, math.inf, 0., -1., True):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    self.compare(**{field: value})
        for value in (math.nan, math.inf, -1., True, .5):
            with self.subTest(tolerance=value), self.assertRaises(ValueError):
                self.compare(endpoint_tolerance_s=value)

    def test_regressed_duplicate_and_nonfinite_samples_are_rejected(self):
        for times in ([0, .5, .5, 1], [0, .75, .5, 1], [0, math.nan, 1]):
            self.write(self.flown, times)
            with self.subTest(times=times), self.assertRaises(ValueError):
                self.compare()
        self.flown.write_text("time,x,y,z,yaw,yaw\n0,0,0,-8,0,0\n1,1,0,-8,0,0\n")
        with self.assertRaisesRegex(ValueError, "duplicate column"):
            self.compare()

    def test_unwrapped_reference_yaw_does_not_erase_a_turn(self):
        self.plan.write_text("time,x,y,z,yaw\n0,0,0,-8,0\n1,1,0,-8,6.283185307179586\n")
        self.write(self.flown, [0, .25, .5, .75, 1.])
        report = self.compare()
        self.assertFalse(report["accepted"])
        self.assertAlmostEqual(report["yaw_error_rad"], math.pi)

    def test_endpoint_tolerance_is_explicit_and_bounded(self):
        self.write(self.flown, [-1e-7, .25, .5, .75, 1.0000001])
        self.assertTrue(self.compare(maximum_sample_gap_s=.251)["accepted"])
        with self.assertRaisesRegex(ValueError, "outside"):
            self.compare(endpoint_tolerance_s=0.)


if __name__ == "__main__":
    unittest.main()
