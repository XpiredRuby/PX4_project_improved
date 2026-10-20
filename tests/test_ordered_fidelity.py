"""Order, repeated visits, continuous approximation and resource refusals."""

import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from mission_planner.geometry import build_smooth_path, BSplinePath
from mission_planner.ordered_fidelity import (
    SplineAudit,
    coupling_within,
    ordered_fidelity,
    audit_controller_csv,
)
from tests.test_planner_geometry import polyline


class OrderedFidelityTests(unittest.TestCase):
    def test_geometric_checker_does_not_call_planner_evaluator(self):
        curve = BSplinePath(
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), (0.0, 0.0, 1.0, 1.0), 1, False
        )
        with patch.object(
            BSplinePath, "evaluate", side_effect=AssertionError("shared evaluator")
        ):
            self.assertTrue(
                ordered_fidelity(curve, curve.control_points, 0.2)["accepted"]
            )

    def test_serialized_reference_detects_changed_field(self):
        from mission_planner.export import save_controller_csv
        from mission_planner.retiming import TimedTrajectoryPoint
        from dataclasses import fields

        points = [
            TimedTrajectoryPoint(
                **{
                    f.name: float(i) if f.name == "time_s" else 0.0
                    for f in fields(TimedTrajectoryPoint)
                }
            )
            for i in (0, 1)
        ]
        candidate = SimpleNamespace(points=points)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "route.csv"
            save_controller_csv(path, candidate)
            self.assertTrue(audit_controller_csv(path, candidate)["accepted"])
            text = path.read_text().splitlines()
            row = text[-1].split(",")
            row[1] = "0.01"
            text[-1] = ",".join(row)
            path.write_text("\n".join(text) + "\n")
            with self.assertRaisesRegex(ValueError, "differs"):
                audit_controller_csv(path, candidate)

    def test_vectorized_coupling_matches_scalar_dynamic_program(self):
        rng = np.random.default_rng(42)
        for _ in range(30):
            distances = rng.random((8, 11))
            for radius in (0.2, 0.5, 0.8, 1.0):
                scalar = np.zeros(distances.shape, dtype=bool)
                for i in range(8):
                    for j in range(11):
                        scalar[i, j] = distances[i, j] <= radius and (
                            (i == 0 and j == 0)
                            or (i > 0 and scalar[i - 1, j])
                            or (j > 0 and scalar[i, j - 1])
                            or (i > 0 and j > 0 and scalar[i - 1, j - 1])
                        )
                self.assertEqual(coupling_within(distances, radius), scalar[-1, -1])

    def test_separate_basis_matches_clamped_and_periodic_geometry(self):
        for closure, points in [
            ("open_stop", [(0, 0, -8), (4, 0, -8), (4, 4, -8)]),
            ("periodic", [(0, 0, -8), (4, 0, -8), (4, 4, -8), (0, 4, -8), (0, 0, -8)]),
        ]:
            g = build_smooth_path(
                polyline(points, maximum_deviation_m=0.75, closure=closure),
                control_spacing_m=0.5,
                verification_resolution_m=0.02,
            )
            audit = SplineAudit(g.curve)
            for u in np.linspace(0, 1, 301):
                self.assertTrue(
                    np.allclose(
                        audit.value(float(u)), g.curve.evaluate(float(u)), atol=1e-10
                    )
                )
            self.assertTrue(
                ordered_fidelity(g.curve, g.source_points, 0.75)["accepted"]
            )

    def test_same_spatial_set_in_wrong_visit_order_is_refused(self):
        # Both cover exactly the same line segment; nearest-point error is zero.
        correct = ((0, 0, 0), (3, 0, 0), (0, 0, 0), (6, 0, 0))
        wrong = ((0, 0, 0), (6, 0, 0), (0, 0, 0), (6, 0, 0))
        curve = BSplinePath(
            control_points=wrong,
            knots=(0.0, 0.0, 1 / 3, 2 / 3, 1.0, 1.0),
            degree=1,
            periodic=False,
        )
        self.assertTrue(ordered_fidelity(curve, wrong, 0.3)["accepted"])
        self.assertFalse(ordered_fidelity(curve, correct, 0.3)["accepted"])

    def test_crossed_loops_cannot_exchange_order(self):
        a, b, c, d, o = (3, 3, 0), (3, -3, 0), (-3, 3, 0), (-3, -3, 0), (0, 0, 0)
        original = (o, a, b, o, c, d, o)
        wrong = (o, c, d, o, a, b, o)
        # Degree-one curves let this test isolate traversal order from smoothing.
        knots = (0.0, 0.0, *tuple(i / 6 for i in range(1, 6)), 1.0, 1.0)
        curve = BSplinePath(wrong, knots, 1, False)
        self.assertFalse(ordered_fidelity(curve, original, 0.5)["accepted"])

    def test_budgets_refuse_instead_of_skipping_order_acceptance(self):
        curve = BSplinePath(
            ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0)), (0.0, 0.0, 1.0, 1.0), 1, False
        )
        with patch("mission_planner.ordered_fidelity.MAX_CELLS", 5):
            with self.assertRaisesRegex(ValueError, "budget"):
                ordered_fidelity(curve, curve.control_points, 0.1)


if __name__ == "__main__":
    unittest.main()
