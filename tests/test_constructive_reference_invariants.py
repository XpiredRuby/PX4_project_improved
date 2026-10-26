"""Independent reference checks and adversarial certificate/intent regressions.

These tests never start a flight runner. Polynomial expectations use exact
rational differentiation; they do not call the planner's derivative evaluator.
"""

from copy import deepcopy
from fractions import Fraction
import json
import math
from pathlib import Path
import unittest

import numpy as np

from mission_planner.closest import PreviewBudgetError, _simplify_sketch
from mission_planner.contracts import EnvironmentSnapshot, VehicleModel
from mission_planner.lifecycle import _evaluate
from mission_planner.lifecycle_verification import derivative_bound, verify_phase


ROOT = Path(__file__).resolve().parents[1]
COEFFICIENTS = (0, 0, 0, 0, 35, -84, 70, -20)


def independent_derivative(u, order):
    """Differentiate the septic in exact rational arithmetic."""
    return sum(
        Fraction(c * math.factorial(k), math.factorial(k - order))
        * u ** (k - order)
        for k, c in enumerate(COEFFICIENTS)
        if k >= order
    )


def marked_drawing(count):
    intent = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
    intent["path"]["points"] = [
        dict(
            point_id=f"stop-{i}", north_m=i / 2, east_m=0., down_m=-20.,
            hard=True, exact=False,
        )
        for i in range(count)
    ]
    return intent


class ConstructiveReferenceInvariants(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = VehicleModel.from_dict(json.loads(
            (ROOT / "config/planner/x500_sitl_vehicle_model.json").read_text()
        ))
        cls.environment = EnvironmentSnapshot.from_dict(json.loads(
            (ROOT / "config/planner/empty_world_environment.json").read_text()
        ))

    def test_vehicle_speed_limit_cannot_be_bypassed_by_phase_metadata(self):
        # Speed peaks at exactly 2.1875 * 80 / 35 = 5 m/s. Other limits
        # have ample margin, so the vehicle's 3 m/s cap is decisive.
        phase = dict(
            name="LEG", start=[-40., 0., -20., 0.], end=[40., 0., -20., 0.],
            duration_s=35., speed_limit_m_s=10.,
        )
        report = verify_phase(phase, self.vehicle, self.environment)
        self.assertGreater(report["speed_bound_m_s"], self.vehicle.maximum_speed_m_s)
        self.assertFalse(report["accepted"])
        self.assertFalse(report["checks"]["speed"])

    def test_speed_check_honors_both_phase_and_vehicle_limits(self):
        phase = dict(
            name="LEG", start=[0., 0., -20., 0.], end=[10., 0., -20., 0.],
            duration_s=30., speed_limit_m_s=1.,
        )
        self.assertTrue(verify_phase(phase, self.vehicle, self.environment)["accepted"])
        phase["speed_limit_m_s"] = .1
        self.assertFalse(verify_phase(phase, self.vehicle, self.environment)["checks"]["speed"])

    def test_phase_numeric_fields_refuse_booleans_and_strings(self):
        base = dict(
            name="HOLD", start=[0., 0., -20., 0.], end=[0., 0., -20., 0.],
            duration_s=2., speed_limit_m_s=1.,
        )
        for field in ("duration_s", "speed_limit_m_s"):
            for bad in (True, "2", float("nan"), float("inf")):
                with self.subTest(field=field, value=bad):
                    phase = dict(base, **{field: bad})
                    with self.assertRaises(ValueError):
                        verify_phase(phase, self.vehicle, self.environment)
        for bad in (False, "0", float("nan"), float("inf")):
            with self.subTest(pose_value=bad):
                phase = deepcopy(base)
                phase["start"][0] = bad
                with self.assertRaises(ValueError):
                    verify_phase(phase, self.vehicle, self.environment)

    def test_simplification_cannot_silently_remove_required_stops(self):
        intent = marked_drawing(49)
        original = deepcopy(intent)
        with self.assertRaises(PreviewBudgetError):
            _simplify_sketch(intent, limit=48)
        self.assertEqual(intent, original)

    def test_public_preview_reports_required_mark_budget(self):
        from mission_planner.preview import preview
        request = dict(source="drawing", document=marked_drawing(49),
                       planning_mode="closest")
        original = deepcopy(request)
        with self.assertRaisesRegex(PreviewBudgetError, "no marked point was removed"):
            preview(request)
        self.assertEqual(request, original)

    def test_malformed_phase_shapes_raise_value_error(self):
        for phase in (None, [], {}, dict(start=None),
                      dict(start=[0, 0, -5], end=[0, 0, -5, 0]),
                      dict(start=[0, 0, -5, 0], end="0000")):
            with self.subTest(phase=phase), self.assertRaises(ValueError):
                verify_phase(phase, self.vehicle, self.environment)

    def test_cross_runtime_fixture_matches_python_evaluator(self):
        fixture = json.loads((ROOT / "tests/fixtures/septic_reference.json").read_text())
        for query in fixture["queries"]:
            phase = fixture["phases"][query["phase_index"]]
            duration = float(phase["duration_s"])
            actual = _evaluate(phase["start"], phase["end"], duration,
                               float(Fraction(query["fraction"])) * duration)
            for field, expected in query["expected"].items():
                if field != "time":
                    self.assertAlmostEqual(actual[field], expected, delta=2e-10)

    def test_simplification_cannot_silently_remove_exact_points(self):
        intent = marked_drawing(49)
        for point in intent["path"]["points"]:
            point.update(hard=False, exact=True)
        with self.assertRaises(PreviewBudgetError):
            _simplify_sketch(intent, limit=48)

    def test_marked_points_within_budget_keep_order_and_identity(self):
        intent = marked_drawing(101)
        required = {0, 17, 52, 100}
        for i, point in enumerate(intent["path"]["points"]):
            point["hard"] = i in required
        reduced = _simplify_sketch(intent, limit=8)
        self.assertEqual(
            [p["point_id"] for p in reduced["path"]["points"] if p["hard"]],
            [f"stop-{i}" for i in sorted(required)],
        )
        self.assertLessEqual(len(reduced["path"]["points"]), 8)

    def test_septic_values_and_derivatives_match_rational_oracle(self):
        fields = (
            ("x", "y", "z", "yaw"), ("vx", "vy", "vz", "yaw_rate"),
            ("ax", "ay", "az", "yaw_acceleration"),
            ("jx", "jy", "jz", "yaw_jerk"),
        )
        start = (Fraction(1), Fraction(-2), Fraction(-5), Fraction(7))
        end = (Fraction(3), Fraction(4), Fraction(-6), Fraction(11))
        duration = Fraction(23, 20)
        for i in range(21):
            u = Fraction(i, 20)
            result = _evaluate(list(map(float, start)), list(map(float, end)),
                               float(duration), float(u * duration))
            for order, keys in enumerate(fields):
                derivative = independent_derivative(u, order) / duration**order
                for axis, key in enumerate(keys):
                    expected = (end[axis] - start[axis]) * derivative
                    if order == 0:
                        expected += start[axis]
                    self.assertAlmostEqual(result[key], float(expected), delta=2e-10)

    def test_continuous_bounds_cover_independently_located_extrema(self):
        polynomial = np.polynomial.Polynomial(COEFFICIENTS)
        for order in (1, 2, 3):
            derivative = polynomial.deriv(order)
            critical = [0., 1.] + [
                float(root.real) for root in derivative.deriv().roots()
                if abs(root.imag) < 1e-9 and 0. <= root.real <= 1.
            ]
            exact_peak = max(abs(derivative(u)) for u in critical)
            self.assertGreaterEqual(derivative_bound(order), exact_peak - 1e-9)
            self.assertLess(derivative_bound(order), exact_peak * 1.01)


if __name__ == "__main__":
    unittest.main()
