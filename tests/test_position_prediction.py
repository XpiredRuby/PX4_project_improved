import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controller"))
from position_prediction import predict_position  # noqa: E402


class PositionPredictionTests(unittest.TestCase):
    def snapshot(self, **changes):
        return dict({"x": 1., "y": 2., "z": -3., "vx": 2., "vy": -1.,
                     "vz": .2, "position_age_s": .025}, **changes)

    def test_constant_velocity_projects_to_command_instant(self):
        result, age = predict_position(self.snapshot(), .075, .15)
        for actual, expected in zip(result, (1.05, 1.975, -2.995), strict=True):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(age, .025)

    def test_vector_displacement_capped(self):
        result, _ = predict_position(self.snapshot(vx=100., vy=100.), .075, .15)
        length = math.sqrt(sum((v - initial)**2 for v, initial in
                               zip(result, (1., 2., -3.), strict=True)))
        self.assertAlmostEqual(length, .15)

    def test_stale_negative_or_invalid_age_never_extrapolated(self):
        for age in (.076, -.1, math.nan, math.inf):
            result, used_age = predict_position(self.snapshot(position_age_s=age), .075, .15)
            self.assertEqual(result, (1., 2., -3.))
            self.assertEqual(used_age, 0.)

    def test_prediction_removes_sample_hold_error_for_constant_velocity(self):
        measured_errors, projected_errors = [], []
        for age in (.002, .049, .005, .045, .012):
            # Current x=10, but the latest received position belongs to an
            # earlier sensor instant. Alternate ages model loop/stream phase.
            snapshot = self.snapshot(x=10. - 2. * age, position_age_s=age)
            position, _ = predict_position(snapshot, .075, .15)
            measured_errors.append(abs(10. - snapshot["x"]))
            projected_errors.append(abs(10. - position[0]))
        self.assertGreater(max(measured_errors), .09)
        self.assertLess(max(projected_errors), 1e-12)
