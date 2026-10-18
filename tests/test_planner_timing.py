"""Tests for the locally time-optimal path-parameter schedule."""

from dataclasses import replace
from functools import lru_cache
import math
import unittest

import numpy as np

from mission_planner.contracts import PathIntent
from mission_planner.geometry import BSplinePath, build_smooth_path
from mission_planner.timing import (
    PathBounds,
    certified_dynamics,
    duration_requirements,
    plan_time_law,
    schedule_defects,
)


# Exact normalized derivative maxima of the seventh-order smootherstep
# q(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7 that a single global time law uses.
PEAK_Q1 = 2.1875
PEAK_Q2 = 7.513188404399301
PEAK_Q3 = 52.5
LIMITS = {
    "speed_limit_m_s": 0.9,
    "acceleration_limit_m_s2": 0.9,
    "jerk_limit_m_s3": 1.35,
}
# A small grid keeps these tests quick where the assertion does not depend on
# the planner finding its very best schedule.
COARSE_GRID = {
    "schedule_intervals": (256,),
    "pace_window_spans": (6.0,),
    "cruise_acceleration_shares": (0.3,),
    "equalization_passes": 6,
}


def polyline(coordinates, *, maximum_deviation_m=0.5, closure="open_stop"):
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
                "hard": False,
                "exact": False,
            }
            for index, coordinate in enumerate(coordinates)
        ],
    })


# Long straights around one tight corner: the case where a single global time
# law is forced to fly the whole mission at the corner's pace.
COORDINATES = (
    (0.0, 0.0, -8.0),
    (10.0, 0.0, -8.0),
    (20.0, 0.0, -8.0),
    (22.0, 2.0, -8.0),
    (22.0, 12.0, -8.0),
    (22.0, 22.0, -8.0),
)


@lru_cache(maxsize=None)
def geometry(closure="open_stop"):
    """Return the shared fixture; plans are immutable, so sharing is safe."""
    coordinates = list(COORDINATES)
    if closure == "periodic":
        coordinates.append(coordinates[0])
    return build_smooth_path(
        polyline(coordinates, closure=closure),
        control_spacing_m=0.5,
        verification_resolution_m=0.01,
    )


def global_requirement(plan):
    """Return the duration a single global smootherstep time law would need."""
    first = plan.curve.derivative_norm_bound(1)
    second = plan.curve.derivative_norm_bound(2)
    third = plan.curve.derivative_norm_bound(3)
    return max(
        first * PEAK_Q1 / LIMITS["speed_limit_m_s"],
        math.sqrt(
            (second * PEAK_Q1**2 + first * PEAK_Q2)
            / LIMITS["acceleration_limit_m_s2"]
        ),
        (
            (
                third * PEAK_Q1**3
                + 3.0 * second * PEAK_Q1 * PEAK_Q2
                + first * PEAK_Q3
            )
            / LIMITS["jerk_limit_m_s3"]
        )
        ** (1.0 / 3.0),
    )


def sampled_dynamics(plan, law, duration, count=1500):
    speed = acceleration = jerk = 0.0
    for index in range(count + 1):
        parameter, q1, q2, q3 = law.sample(index / count)
        first = np.asarray(plan.curve.evaluate(parameter, 1))
        second = np.asarray(plan.curve.evaluate(parameter, 2))
        third = np.asarray(plan.curve.evaluate(parameter, 3))
        speed = max(speed, float(np.linalg.norm(first * q1)) / duration)
        acceleration = max(
            acceleration,
            float(np.linalg.norm(second * q1**2 + first * q2)) / duration**2,
        )
        jerk = max(
            jerk,
            float(
                np.linalg.norm(
                    third * q1**3 + 3.0 * second * q1 * q2 + first * q3
                )
            )
            / duration**3,
        )
    return speed, acceleration, jerk


class PlannerTimingTests(unittest.TestCase):
    def test_schedule_is_monotone_stationary_and_spans_the_path(self):
        law = plan_time_law(geometry(), **LIMITS, **COARSE_GRID)
        self.assertEqual(schedule_defects(law.schedule), ())
        self.assertEqual(law.sample(0.0), (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(law.sample(1.0), (1.0, 0.0, 0.0, 0.0))
        samples = np.asarray([
            law.sample(value)[0] for value in np.linspace(0.0, 1.0, 2000)
        ])
        self.assertTrue(bool(np.all(np.diff(samples) >= -1e-15)))
        self.assertGreater(law.report.minimum_requirement_s, 0.0)
        self.assertIn(
            law.report.limiting_constraint,
            {"speed", "acceleration", "jerk"},
        )

    def test_local_timing_beats_a_single_global_time_law(self):
        plan = geometry()
        law = plan_time_law(plan, **LIMITS)
        # The global law pays for the tightest corner everywhere, so localizing
        # the bounds is what makes a mostly straight mission cheap.
        self.assertLess(
            law.report.minimum_requirement_s,
            0.5 * global_requirement(plan),
        )
        # No time law can beat flying the whole curve at the speed limit.
        self.assertGreater(
            law.report.minimum_requirement_s,
            law.report.straight_line_duration_s,
        )
        self.assertAlmostEqual(
            law.report.straight_line_duration_s,
            plan.smoothing.curve_length_m / LIMITS["speed_limit_m_s"],
            places=9,
        )

    def test_certified_bounds_hold_and_fit_the_limits(self):
        plan = geometry()
        law = plan_time_law(plan, **LIMITS, **COARSE_GRID)
        duration = law.report.minimum_requirement_s
        bounds = certified_dynamics(
            PathBounds.of(plan.curve), law.schedule, duration
        )
        limits = (
            LIMITS["speed_limit_m_s"],
            LIMITS["acceleration_limit_m_s2"],
            LIMITS["jerk_limit_m_s3"],
        )
        for bound, limit in zip(bounds, limits, strict=True):
            self.assertLessEqual(bound, limit)
        for sampled, bound in zip(
            sampled_dynamics(plan, law, duration), bounds, strict=True
        ):
            self.assertLessEqual(sampled, bound + 1e-9)
        # The binding constraint has to be nearly tight, or the reported
        # duration is longer than the bounds actually require.
        self.assertGreater(
            max(bound / limit for bound, limit in zip(bounds, limits, strict=True)),
            0.99,
        )

    def test_stretching_the_duration_scales_every_bound_down(self):
        plan = geometry()
        law = plan_time_law(plan, **LIMITS, **COARSE_GRID)
        bounds = PathBounds.of(plan.curve)
        duration = law.report.minimum_requirement_s
        base = certified_dynamics(bounds, law.schedule, duration)
        stretched = certified_dynamics(bounds, law.schedule, 2.0 * duration)
        for power, (first, second) in enumerate(
            zip(base, stretched, strict=True), start=1
        ):
            self.assertAlmostEqual(second * 2.0**power, first, places=9)

    def test_requirements_are_localized_to_each_schedule_span(self):
        plan = geometry()
        law = plan_time_law(plan, **LIMITS, **COARSE_GRID)
        requirements, limiting = duration_requirements(
            PathBounds.of(plan.curve), law.schedule, **LIMITS
        )
        self.assertEqual(len(requirements), len(limiting))
        # Spans over the straights must demand far less than the corner span,
        # which is the whole point of charging each span separately.
        self.assertLess(min(requirements), 0.5 * max(requirements))
        self.assertLessEqual(max(requirements), law.report.minimum_requirement_s)

    def test_schedule_defects_detect_tampering(self):
        law = plan_time_law(geometry(), **LIMITS, **COARSE_GRID)
        values = [list(point) for point in law.schedule.control_points]

        moving = list(values)
        moving[0] = [0.01]
        with self.subTest(defect="stationarity"):
            tampered = replace(
                law.schedule,
                control_points=tuple(tuple(value) for value in moving),
            )
            self.assertIn(
                "schedule must be stationary through jerk at both ends",
                schedule_defects(tampered),
            )

        reversed_values = list(values)
        middle = len(reversed_values) // 2
        reversed_values[middle] = [0.0]
        with self.subTest(defect="monotonicity"):
            tampered = replace(
                law.schedule,
                control_points=tuple(
                    tuple(value) for value in reversed_values
                ),
            )
            self.assertIn(
                "schedule control values must be nondecreasing",
                schedule_defects(tampered),
            )

        with self.subTest(defect="degree"):
            quartic = BSplinePath(
                control_points=tuple((value / 5.0,) for value in range(6)),
                knots=(0.0,) * 5 + (0.5,) + (1.0,) * 5,
                degree=4,
                periodic=False,
            )
            self.assertIn("degree must be 5", schedule_defects(quartic))

    def test_invalid_limits_and_grids_are_rejected(self):
        plan = geometry()
        for field in (
            "speed_limit_m_s",
            "acceleration_limit_m_s2",
            "jerk_limit_m_s3",
        ):
            for value in (0.0, -1.0, math.nan, math.inf):
                limits = dict(LIMITS, **{field: value})
                with (
                    self.subTest(field=field, value=value),
                    self.assertRaises(ValueError),
                ):
                    plan_time_law(plan, **limits, **COARSE_GRID)
        for intervals in ((4,), (10_000,), (1.5,), (True,), ()):
            with self.subTest(intervals=intervals), self.assertRaises(ValueError):
                plan_time_law(plan, **LIMITS, schedule_intervals=intervals)
        for windows in ((0.0,), (-1.0,), (math.inf,), (True,), ("wide",), ()):
            with self.subTest(windows=windows), self.assertRaises(ValueError):
                plan_time_law(plan, **LIMITS, pace_window_spans=windows)
        for shares in ((0.0,), (1.0,), (-0.5,), (math.nan,), (True,), ()):
            with self.subTest(shares=shares), self.assertRaises(ValueError):
                plan_time_law(
                    plan, **LIMITS, cruise_acceleration_shares=shares
                )
        for passes in (0, -1, 101, 1.5, True):
            with self.subTest(passes=passes), self.assertRaises(ValueError):
                plan_time_law(plan, **LIMITS, equalization_passes=passes)

    def test_planning_is_deterministic_and_a_closed_path_stops_at_its_seam(self):
        plan = geometry()
        first = plan_time_law(plan, **LIMITS, **COARSE_GRID)
        second = plan_time_law(plan, **LIMITS, **COARSE_GRID)
        self.assertEqual(first, second)
        periodic = plan_time_law(
            geometry(closure="periodic"), **LIMITS, **COARSE_GRID
        )
        self.assertEqual(periodic.schedule.evaluate(0.0)[0], 0.0)
        self.assertAlmostEqual(periodic.schedule.evaluate(1.0)[0], 1.0, places=9)
        with self.assertRaisesRegex(ValueError, "only a periodic path"):
            plan_time_law(plan, **LIMITS, **COARSE_GRID, loop_count=2)


if __name__ == "__main__":
    unittest.main()
