"""Smooth drawn-path geometry and fidelity-bound tests."""

import math
import unittest

import numpy as np

from mission_planner.contracts import PathIntent
from mission_planner.geometry import (
    BSplinePath,
    build_smooth_path,
    tangent_heading_slope_bound,
)


def polyline(
    coordinates,
    *,
    maximum_deviation_m=0.25,
    closure="open_stop",
    hard_indices=(),
):
    return PathIntent.from_dict({
        "kind": "polyline",
        "closure": closure,
        "maximum_deviation_m": maximum_deviation_m,
        "points": [
            {
                "point_id": f"point-{index}",
                "north_m": coordinate[0],
                "east_m": coordinate[1],
                "down_m": coordinate[2],
                "hard": index in hard_indices,
            }
            for index, coordinate in enumerate(coordinates)
        ],
    })


class PlannerGeometryTests(unittest.TestCase):
    def test_straight_path_is_endpoint_exact_and_collinear(self):
        path = polyline(
            [(0.0, 0.0, -8.0), (10.0, 0.0, -8.0)],
            maximum_deviation_m=0.1,
            hard_indices=(0, 1),
        )
        plan = build_smooth_path(
            path,
            control_spacing_m=1.0,
            verification_resolution_m=0.01,
        )
        self.assertEqual(plan.curve.evaluate(0.0), (0.0, 0.0, -8.0))
        self.assertEqual(plan.curve.evaluate(1.0), (10.0, 0.0, -8.0))
        for parameter in np.linspace(0.0, 1.0, 101):
            point = plan.curve.evaluate(float(parameter))
            self.assertAlmostEqual(point[1], 0.0, places=12)
            self.assertAlmostEqual(point[2], -8.0, places=12)
        self.assertTrue(plan.fidelity.accepted)
        self.assertLessEqual(plan.fidelity.symmetric_upper_bound_m, 0.1)

    def test_corner_is_accepted_or_rejected_by_declared_fidelity(self):
        coordinates = [
            (0.0, 0.0, -8.0),
            (5.0, 0.0, -8.0),
            (5.0, 5.0, -8.0),
        ]
        accepted = build_smooth_path(
            polyline(coordinates, maximum_deviation_m=0.75),
            control_spacing_m=0.5,
            verification_resolution_m=0.01,
        )
        self.assertTrue(accepted.fidelity.accepted)
        with self.assertRaisesRegex(ValueError, "exceeds maximum deviation"):
            build_smooth_path(
                polyline(coordinates, maximum_deviation_m=0.02),
                control_spacing_m=0.5,
                verification_resolution_m=0.005,
            )

    def test_periodic_circle_matches_position_and_derivatives_at_seam(self):
        radius = 5.0
        coordinates = [
            (
                radius * math.cos(2.0 * math.pi * index / 24),
                radius * math.sin(2.0 * math.pi * index / 24),
                -8.0,
            )
            for index in range(24)
        ]
        coordinates.append(coordinates[0])
        plan = build_smooth_path(
            polyline(
                coordinates,
                closure="periodic",
                maximum_deviation_m=0.25,
                hard_indices=(0, len(coordinates) - 1),
            ),
            control_spacing_m=0.5,
            verification_resolution_m=0.01,
        )
        self.assertTrue(plan.curve.periodic)
        for derivative in range(4):
            self.assertTrue(np.allclose(
                plan.curve.evaluate(0.0, derivative),
                plan.curve.evaluate(1.0, derivative),
                atol=1e-9,
                rtol=1e-9,
            ))
        radial_distances = [
            math.hypot(*plan.curve.evaluate(float(parameter))[:2])
            for parameter in np.linspace(0.0, 1.0, 201)
        ]
        self.assertLess(max(abs(value - radius) for value in radial_distances), 0.25)

    def test_analytic_derivatives_match_finite_differences(self):
        plan = build_smooth_path(
            polyline(
                [
                    (0.0, 0.0, -8.0),
                    (3.0, 1.0, -7.5),
                    (6.0, -1.0, -8.5),
                    (9.0, 0.0, -8.0),
                ],
                maximum_deviation_m=0.5,
            ),
            control_spacing_m=0.4,
            verification_resolution_m=0.01,
        )
        parameter = 0.43
        step = 1e-5
        before = np.asarray(plan.curve.evaluate(parameter - step))
        after = np.asarray(plan.curve.evaluate(parameter + step))
        numerical = (after - before) / (2.0 * step)
        analytic = np.asarray(plan.curve.evaluate(parameter, 1))
        self.assertTrue(np.allclose(numerical, analytic, atol=1e-5, rtol=1e-5))
        self.assertGreater(plan.curve.derivative_norm_bound(), 0.0)

    def test_geometry_is_byte_deterministic(self):
        path = polyline(
            [(0.0, 0.0, -8.0), (2.0, 1.0, -8.0), (4.0, 0.0, -8.0)],
            maximum_deviation_m=0.5,
        )
        first = build_smooth_path(
            path,
            control_spacing_m=0.25,
            verification_resolution_m=0.01,
        )
        second = build_smooth_path(
            path,
            control_spacing_m=0.25,
            verification_resolution_m=0.01,
        )
        self.assertEqual(first, second)

    def test_unsupported_or_degenerate_inputs_fail_closed(self):
        internal_hard = polyline(
            [
                (0.0, 0.0, -8.0),
                (1.0, 0.0, -8.0),
                (2.0, 0.0, -8.0),
            ],
            hard_indices=(1,),
        )
        with self.assertRaisesRegex(ValueError, "internal hard points"):
            build_smooth_path(internal_hard, control_spacing_m=0.2)

        waypoint = PathIntent(
            kind="waypoints",
            closure=internal_hard.closure,
            maximum_deviation_m=internal_hard.maximum_deviation_m,
            points=internal_hard.points,
        )
        with self.assertRaisesRegex(ValueError, "polyline"):
            build_smooth_path(waypoint, control_spacing_m=0.2)

        open_depart = polyline(
            [(0.0, 0.0, -8.0), (1.0, 0.0, -8.0)],
            closure="open_depart",
        )
        with self.assertRaisesRegex(ValueError, "open_depart"):
            build_smooth_path(open_depart, control_spacing_m=0.2)

        duplicate = polyline([
            (0.0, 0.0, -8.0),
            (0.0, 0.0, -8.0),
        ])
        with self.assertRaisesRegex(ValueError, "fewer than two"):
            build_smooth_path(duplicate, control_spacing_m=0.2)

        zero_tolerance = polyline(
            [(0.0, 0.0, -8.0), (1.0, 0.0, -8.0)],
            maximum_deviation_m=0.0,
        )
        with self.assertRaisesRegex(ValueError, "zero path deviation"):
            build_smooth_path(zero_tolerance, control_spacing_m=0.2)

        for value in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                build_smooth_path(internal_hard, control_spacing_m=value)

    def test_deviation_budget_is_spent_to_widen_corners(self):
        path = polyline(
            [
                (0.0, 0.0, -8.0),
                (6.0, 4.0, -8.0),
                (12.0, -3.0, -9.0),
                (18.0, 0.0, -8.0),
            ],
            maximum_deviation_m=0.5,
        )
        faithful = build_smooth_path(
            path,
            control_spacing_m=0.25,
            verification_resolution_m=0.01,
            deviation_utilization=0.05,
        )
        relaxed = build_smooth_path(
            path,
            control_spacing_m=0.25,
            verification_resolution_m=0.01,
            deviation_utilization=0.75,
        )
        self.assertGreater(
            relaxed.smoothing.selected_control_spacing_m,
            faithful.smoothing.selected_control_spacing_m,
        )
        self.assertGreater(
            relaxed.fidelity.symmetric_upper_bound_m,
            faithful.fidelity.symmetric_upper_bound_m,
        )
        for plan in (faithful, relaxed):
            self.assertTrue(plan.fidelity.accepted)
            self.assertLessEqual(
                plan.fidelity.symmetric_upper_bound_m,
                path.maximum_deviation_m,
            )
            self.assertEqual(plan.smoothing.requested_control_spacing_m, 0.25)
            self.assertEqual(
                plan.smoothing.control_point_count,
                len(plan.curve.control_points),
            )
            self.assertAlmostEqual(plan.smoothing.path_length_m, 23.267, places=2)
        # A wider corner is a lower-curvature corner, which is what makes the
        # retimed mission cheaper rather than merely shorter in control points.
        self.assertLess(
            relaxed.curve.derivative_norm_bound(2)
            / relaxed.curve.derivative_norm_bound(1) ** 2,
            faithful.curve.derivative_norm_bound(2)
            / faithful.curve.derivative_norm_bound(1) ** 2,
        )
        self.assertLess(
            relaxed.smoothing.heading_slope_bound_rad,
            faithful.smoothing.heading_slope_bound_rad,
        )

    def test_deviation_utilization_is_validated(self):
        path = polyline(
            [(0.0, 0.0, -8.0), (5.0, 1.0, -8.0), (10.0, 0.0, -8.0)],
            maximum_deviation_m=0.5,
        )
        for value in (0.0, -0.5, 1.5, math.nan, math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                build_smooth_path(
                    path,
                    control_spacing_m=0.25,
                    verification_resolution_m=0.01,
                    deviation_utilization=value,
                )

    def test_heading_slope_bound_is_valid_and_rejects_vanishing_tangents(self):
        plan = build_smooth_path(
            polyline(
                [
                    (0.0, 0.0, -8.0),
                    (4.0, 3.0, -8.0),
                    (8.0, -2.0, -8.0),
                ],
                maximum_deviation_m=0.4,
            ),
            control_spacing_m=0.25,
            verification_resolution_m=0.01,
        )
        bound = tangent_heading_slope_bound(plan.curve)
        parameters = np.linspace(0.0, 1.0, 4001)
        tangents = np.asarray([
            plan.curve.evaluate(float(value), 1) for value in parameters
        ])
        headings = np.unwrap(np.arctan2(tangents[:, 1], tangents[:, 0]))
        measured = float(np.max(np.abs(np.gradient(headings, parameters))))
        self.assertGreaterEqual(bound, measured)
        self.assertEqual(bound, plan.smoothing.heading_slope_bound_rad)

        vertical = build_smooth_path(
            polyline(
                [(0.0, 0.0, -4.0), (0.0, 0.0, -12.0)],
                maximum_deviation_m=0.2,
            ),
            control_spacing_m=0.5,
            verification_resolution_m=0.01,
        )
        with self.assertRaisesRegex(ValueError, "horizontal speed"):
            tangent_heading_slope_bound(vertical.curve)
        self.assertIsNone(vertical.smoothing.heading_slope_bound_rad)

    def test_scalar_spline_reuses_curve_evaluation_and_bounds(self):
        # A one-dimensional schedule must share the validated de Boor and
        # convex-hull code so later yaw planning needs no second implementation.
        degree = 5
        count = 10
        knots = (
            [0.0] * (degree + 1)
            + [index / (count - degree) for index in range(1, count - degree)]
            + [1.0] * (degree + 1)
        )
        ramp = BSplinePath(
            control_points=tuple(
                (index / (count - 1),) for index in range(count)
            ),
            knots=tuple(knots),
            degree=degree,
            periodic=False,
        )
        self.assertEqual(ramp.dimension, 1)
        self.assertEqual(ramp.evaluate(0.0), (0.0,))
        self.assertEqual(ramp.evaluate(1.0), (1.0,))
        step = 1e-6
        numerical = (
            ramp.evaluate(0.4 + step)[0] - ramp.evaluate(0.4 - step)[0]
        ) / (2.0 * step)
        self.assertAlmostEqual(numerical, ramp.evaluate(0.4, 1)[0], places=5)
        self.assertGreater(ramp.derivative_norm_bound(1), 0.0)
        linear = BSplinePath(
            control_points=((0.0,), (1.0,)),
            knots=(0.0, 0.0, 1.0, 1.0),
            degree=1,
            periodic=False,
        )
        self.assertEqual(linear.evaluate(0.5, 2), (0.0,))
        with self.assertRaisesRegex(ValueError, "D >= 1"):
            BSplinePath(
                control_points=((), ()),
                knots=(0.0, 0.0, 1.0, 1.0),
                degree=1,
                periodic=False,
            )

    def test_invalid_curve_queries_fail_closed(self):
        curve = build_smooth_path(
            polyline(
                [(0.0, 0.0, -8.0), (1.0, 0.0, -8.0)],
                maximum_deviation_m=0.1,
            ),
            control_spacing_m=0.1,
            verification_resolution_m=0.01,
        ).curve
        for parameter in (-0.1, 1.1, math.nan, math.inf):
            with self.subTest(parameter=parameter), self.assertRaises(ValueError):
                curve.evaluate(parameter)
        for derivative in (-1, 4, 1.0):
            with self.subTest(derivative=derivative), self.assertRaises(ValueError):
                curve.evaluate(0.5, derivative)


if __name__ == "__main__":
    unittest.main()
