"""Analytic stop-to-stop retiming tests for smooth drawn paths."""

from dataclasses import replace
from functools import lru_cache
import math
import unittest

from mission_planner.contracts import MotionIntent, PathIntent, VehicleModel
from mission_planner.geometry import build_smooth_path
from mission_planner.retiming import retime_stop_path
from mission_planner.timing import schedule_defects


def path(*, closure="open_stop"):
    coordinates = [
        (0.0, 0.0, -8.0),
        (3.0, 1.0, -7.5),
        (6.0, -1.0, -8.5),
        (9.0, 0.0, -8.0),
    ]
    if closure == "periodic":
        coordinates.append(coordinates[0])
    return PathIntent.from_dict({
        "kind": "polyline",
        "closure": closure,
        "maximum_deviation_m": 0.75,
        "points": [
            {
                "point_id": f"point-{index}",
                "north_m": point[0],
                "east_m": point[1],
                "down_m": point[2],
                "hard": index in (0, len(coordinates) - 1),
            }
            for index, point in enumerate(coordinates)
        ],
    })


def motion(*, time_mode="minimize", duration=None, yaw_mode="fixed"):
    return MotionIntent.from_dict({
        "target_speed_m_s": 1.0,
        "maximum_speed_m_s": 1.25,
        "yaw_mode": yaw_mode,
        "fixed_yaw_deg": 30.0 if yaw_mode == "fixed" else None,
        "time_mode": time_mode,
        "requested_duration_s": duration,
    })


def vehicle():
    return VehicleModel.from_dict({
        "schema_version": 1,
        "vehicle_model_id": "px4-x500-sitl-v1",
        "configuration": "PX4 v1.17 Gazebo x500 research model",
        "mass_kg": 2.0,
        "maximum_speed_m_s": 3.0,
        "maximum_acceleration_m_s2": 1.0,
        "maximum_jerk_m_s3": 1.5,
        "maximum_yaw_rate_deg_s": 45.0,
        "maximum_yaw_acceleration_deg_s2": 60.0,
        "maximum_yaw_jerk_deg_s3": 120.0,
        "maximum_tilt_deg": 25.0,
        "maximum_body_rate_deg_s": 90.0,
        "minimum_thrust_n": 0.0,
        "maximum_thrust_n": 40.0,
        "identified_for_physical_flight": False,
    })


@lru_cache(maxsize=None)
def geometry(*, closure="open_stop"):
    """Return the shared fixture; plans are immutable, so sharing is safe."""
    return build_smooth_path(
        path(closure=closure),
        control_spacing_m=0.3,
        verification_resolution_m=0.01,
    )


class PlannerRetimingTests(unittest.TestCase):
    def test_retimed_path_is_endpoint_inclusive_stationary_and_bounded(self):
        plan = retime_stop_path(
            geometry(),
            motion(),
            vehicle(),
            sample_period_s=0.05,
            limit_scale=0.9,
        )
        self.assertEqual(plan.release_class, "SITL_ONLY")
        self.assertEqual(plan.points[0].time_s, 0.0)
        self.assertEqual(
            plan.points[-1].time_s,
            plan.summary.selected_duration_s,
        )
        self.assertTrue(all(
            right.time_s > left.time_s
            for left, right in zip(plan.points, plan.points[1:], strict=False)
        ))
        self.assertLessEqual(
            max(
                right.time_s - left.time_s
                for left, right in zip(
                    plan.points, plan.points[1:], strict=False
                )
            ),
            0.05 + 1e-12,
        )
        for endpoint in (plan.points[0], plan.points[-1]):
            self.assertEqual(
                (
                    endpoint.north_velocity_m_s,
                    endpoint.east_velocity_m_s,
                    endpoint.down_velocity_m_s,
                    endpoint.north_acceleration_m_s2,
                    endpoint.east_acceleration_m_s2,
                    endpoint.down_acceleration_m_s2,
                    endpoint.north_jerk_m_s3,
                    endpoint.east_jerk_m_s3,
                    endpoint.down_jerk_m_s3,
                ),
                (0.0,) * 9,
            )
        summary = plan.summary
        self.assertLessEqual(
            summary.conservative_speed_bound_m_s,
            summary.speed_limit_m_s * (1.0 + 1e-12),
        )
        self.assertLessEqual(
            summary.conservative_acceleration_bound_m_s2,
            summary.acceleration_limit_m_s2 * (1.0 + 1e-12),
        )
        self.assertLessEqual(
            summary.conservative_jerk_bound_m_s3,
            summary.jerk_limit_m_s3 * (1.0 + 1e-12),
        )

    def test_candidate_carries_a_valid_time_law_that_sets_the_duration(self):
        plan = retime_stop_path(geometry(), motion(), vehicle())
        self.assertEqual(schedule_defects(plan.schedule), ())
        report = plan.summary.time_law
        self.assertEqual(
            report.minimum_requirement_s,
            plan.summary.minimum_feasible_duration_s,
        )
        self.assertEqual(
            report.limiting_constraint,
            plan.summary.limiting_constraint,
        )
        self.assertEqual(report.speed_limit_m_s, plan.summary.speed_limit_m_s)
        # Every emitted parameter must come from the carried schedule, since
        # that schedule is what the verifier recomputes the trajectory from.
        duration = plan.summary.selected_duration_s
        for point in plan.points:
            self.assertAlmostEqual(
                point.path_parameter,
                plan.schedule.evaluate(point.time_s / duration)[0],
                places=12,
            )

    def test_exact_and_maximum_duration_semantics_fail_closed(self):
        baseline = retime_stop_path(geometry(), motion(), vehicle())
        minimum = baseline.summary.minimum_feasible_duration_s
        exact = retime_stop_path(
            geometry(),
            motion(time_mode="exact", duration=minimum + 5.0),
            vehicle(),
        )
        self.assertEqual(exact.summary.selected_duration_s, minimum + 5.0)
        with self.assertRaisesRegex(ValueError, "minimum feasible"):
            retime_stop_path(
                geometry(),
                motion(time_mode="exact", duration=minimum / 2.0),
                vehicle(),
            )
        with self.assertRaisesRegex(ValueError, "minimum feasible"):
            retime_stop_path(
                geometry(),
                motion(time_mode="maximum", duration=minimum / 2.0),
                vehicle(),
            )

    def test_tighter_limits_increase_duration(self):
        baseline = retime_stop_path(geometry(), motion(), vehicle())
        slower = retime_stop_path(
            geometry(),
            motion(),
            replace(
                vehicle(),
                maximum_speed_m_s=0.5,
                maximum_acceleration_m_s2=0.4,
                maximum_jerk_m_s3=0.5,
            ),
        )
        self.assertGreater(
            slower.summary.selected_duration_s,
            baseline.summary.selected_duration_s,
        )

    def test_periodic_lap_stops_at_the_seam_and_unconstrained_yaw_is_refused(self):
        plan = retime_stop_path(
            geometry(closure="periodic"),
            motion(),
            vehicle(),
            loop_count=1,
        )
        self.assertEqual(plan.summary.loop_count, 1)
        self.assertEqual(plan.points[0].path_parameter, 0.0)
        self.assertEqual(plan.points[-1].path_parameter, 1.0)
        self.assertAlmostEqual(plan.points[0].north_m, plan.points[-1].north_m, places=6)
        self.assertAlmostEqual(plan.points[0].east_m, plan.points[-1].east_m, places=6)
        with self.assertRaisesRegex(ValueError, "unconstrained yaw"):
            retime_stop_path(
                geometry(),
                motion(yaw_mode="unconstrained"),
                vehicle(),
            )
        with self.assertRaisesRegex(ValueError, "only a periodic path"):
            retime_stop_path(geometry(), motion(), vehicle(), loop_count=2)

    def test_output_is_deterministic_and_digest_sensitive(self):
        first = retime_stop_path(geometry(), motion(), vehicle())
        second = retime_stop_path(geometry(), motion(), vehicle())
        self.assertEqual(first, second)
        self.assertEqual(first.sha256, second.sha256)
        changed = retime_stop_path(
            geometry(),
            motion(time_mode="exact", duration=first.summary.selected_duration_s + 1),
            vehicle(),
        )
        self.assertNotEqual(first.sha256, changed.sha256)

    def test_invalid_retiming_controls_fail(self):
        for period in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(period=period), self.assertRaises(ValueError):
                retime_stop_path(
                    geometry(),
                    motion(),
                    vehicle(),
                    sample_period_s=period,
                )
        for scale in (0.0, -1.0, 1.1, math.nan, math.inf):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                retime_stop_path(
                    geometry(),
                    motion(),
                    vehicle(),
                    limit_scale=scale,
                )
        with self.assertRaisesRegex(ValueError, "output points"):
            retime_stop_path(
                geometry(),
                motion(),
                vehicle(),
                sample_period_s=1e-9,
            )


if __name__ == "__main__":
    unittest.main()
