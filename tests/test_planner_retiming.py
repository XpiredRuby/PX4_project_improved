"""Analytic stop-to-stop retiming tests for smooth drawn paths."""

from dataclasses import replace
import math
import unittest

from mission_planner.contracts import MotionIntent, PathIntent, VehicleModel
from mission_planner.geometry import build_smooth_path
from mission_planner.retiming import retime_stop_path


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
        "maximum_tilt_deg": 25.0,
        "maximum_body_rate_deg_s": 90.0,
        "minimum_thrust_n": 0.0,
        "maximum_thrust_n": 40.0,
        "identified_for_physical_flight": False,
    })


def geometry(*, closure="open_stop"):
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

    def test_periodic_and_nonfixed_yaw_are_explicitly_deferred(self):
        with self.assertRaisesRegex(ValueError, "entry and exit"):
            retime_stop_path(geometry(closure="periodic"), motion(), vehicle())
        with self.assertRaisesRegex(ValueError, "fixed yaw"):
            retime_stop_path(
                geometry(),
                motion(yaw_mode="tangent"),
                vehicle(),
            )

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
