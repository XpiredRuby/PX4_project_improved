"""Independent acceptance tests for baseline trajectory candidates."""

from dataclasses import replace
import unittest

from mission_planner.contracts import EnvironmentSnapshot
from mission_planner.retiming import retime_stop_path
from mission_planner.verification import verify_candidate_trajectory
from tests.test_planner_retiming import geometry, motion, vehicle


def environment(**overrides):
    raw = {
        "schema_version": 1,
        "environment_id": "empty-world-v1",
        "frame": {
            "name": "LOCAL_NED",
            "units": "m",
            "origin_id": "launch-001",
            "altitude_datum": "launch",
        },
        "max_radius_m": 40.0,
        "max_height_above_launch_m": 30.0,
        "max_drop_below_launch_m": 2.0,
        "max_wind_m_s": 5.0,
        "minimum_clearance_m": 5.0,
        "obstacle_map_sha256": None,
        "obstacle_map_verified": False,
    }
    raw.update(overrides)
    return EnvironmentSnapshot.from_dict(raw)


def by_requirement(report):
    return {check.requirement_id: check for check in report.checks}


class PlannerVerificationTests(unittest.TestCase):
    def test_valid_candidate_passes_independent_checks(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        report = verify_candidate_trajectory(
            smooth,
            candidate,
            requested_motion,
            model,
            environment(),
            maximum_deviation_m=0.75,
        )
        self.assertTrue(report.accepted)
        self.assertEqual(
            set(by_requirement(report)),
            {"INT-007", "PLN-001", "VER-003", "ENV-001", "PLN-004", "PLN-007"},
        )
        self.assertTrue(all(check.status == "PASS" for check in report.checks))
        self.assertEqual(report.candidate_trajectory_sha256, candidate.sha256)

    def test_modified_position_is_rejected(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        points = list(candidate.points)
        points[len(points) // 2] = replace(
            points[len(points) // 2],
            east_m=points[len(points) // 2].east_m + 1.0,
        )
        modified = replace(candidate, points=tuple(points))
        report = verify_candidate_trajectory(
            smooth,
            modified,
            requested_motion,
            model,
            environment(),
            maximum_deviation_m=0.75,
        )
        self.assertFalse(report.accepted)
        self.assertEqual(by_requirement(report)["VER-003"].status, "FAIL")
        self.assertNotEqual(candidate.sha256, modified.sha256)

    def test_modified_derivative_or_time_law_is_rejected(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        middle = len(candidate.points) // 2
        for field, value in (
            (
                "north_velocity_m_s",
                candidate.points[middle].north_velocity_m_s + 0.1,
            ),
            (
                "path_parameter",
                candidate.points[middle].path_parameter + 0.01,
            ),
        ):
            points = list(candidate.points)
            points[middle] = replace(points[middle], **{field: value})
            modified = replace(candidate, points=tuple(points))
            with self.subTest(field=field):
                report = verify_candidate_trajectory(
                    smooth,
                    modified,
                    requested_motion,
                    model,
                    environment(),
                    maximum_deviation_m=0.75,
                )
                self.assertFalse(report.accepted)
                self.assertEqual(
                    by_requirement(report)["VER-003"].status,
                    "FAIL",
                )

    def test_modified_schedule_is_rejected(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        values = [list(value) for value in candidate.schedule.control_points]
        middle = len(values) // 2
        cases = {
            # Moving an interior control value leaves a valid schedule that no
            # longer explains the emitted points.
            "VER-003": (middle, [min(values[middle][0] + 0.05, 1.0)]),
            # Moving an endpoint control value destroys the stationarity the
            # schedule claims, which acceptance must notice on its own.
            "PLN-001": (0, [0.01]),
        }
        for requirement, (index, value) in cases.items():
            tampered = list(values)
            tampered[index] = value
            modified = replace(
                candidate,
                schedule=replace(
                    candidate.schedule,
                    control_points=tuple(
                        tuple(entry) for entry in tampered
                    ),
                ),
            )
            with self.subTest(requirement=requirement):
                report = verify_candidate_trajectory(
                    smooth,
                    modified,
                    requested_motion,
                    model,
                    environment(),
                    maximum_deviation_m=0.75,
                )
                self.assertFalse(report.accepted)
                self.assertEqual(
                    by_requirement(report)[requirement].status,
                    "FAIL",
                )
                self.assertNotEqual(candidate.sha256, modified.sha256)

    def test_sparse_endpoint_only_controller_path_is_rejected(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        sparse = replace(
            candidate,
            points=(candidate.points[0], candidate.points[-1]),
        )
        report = verify_candidate_trajectory(
            smooth,
            sparse,
            requested_motion,
            model,
            environment(),
            maximum_deviation_m=0.75,
        )
        checks = by_requirement(report)
        self.assertFalse(report.accepted)
        self.assertEqual(checks["PLN-001"].status, "FAIL")
        self.assertEqual(checks["INT-007"].status, "FAIL")

    def test_environment_control_hull_violation_is_rejected(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        report = verify_candidate_trajectory(
            smooth,
            candidate,
            requested_motion,
            model,
            environment(max_radius_m=5.0),
            maximum_deviation_m=0.75,
        )
        self.assertFalse(report.accepted)
        check = by_requirement(report)["ENV-001"]
        self.assertEqual(check.status, "FAIL")
        self.assertLess(check.margin, 0.0)

    def test_fidelity_is_recomputed_from_mission_limit(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        report = verify_candidate_trajectory(
            smooth,
            candidate,
            requested_motion,
            model,
            environment(),
            maximum_deviation_m=0.01,
            fidelity_resolution_m=0.005,
        )
        self.assertFalse(report.accepted)
        self.assertEqual(by_requirement(report)["INT-007"].status, "FAIL")

    def test_falsified_duration_and_limit_scale_are_rejected(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        summary = replace(
            candidate.summary,
            selected_duration_s=candidate.summary.selected_duration_s / 2.0,
            limit_scale=1.0,
        )
        modified = replace(candidate, summary=summary)
        report = verify_candidate_trajectory(
            smooth,
            modified,
            requested_motion,
            model,
            environment(),
            maximum_deviation_m=0.75,
        )
        checks = by_requirement(report)
        self.assertFalse(report.accepted)
        self.assertEqual(checks["PLN-001"].status, "FAIL")
        self.assertEqual(checks["PLN-004"].status, "FAIL")
        self.assertEqual(checks["PLN-007"].status, "FAIL")

    def test_report_is_deterministic(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        first = verify_candidate_trajectory(
            smooth,
            candidate,
            requested_motion,
            model,
            environment(),
            maximum_deviation_m=0.75,
        )
        second = verify_candidate_trajectory(
            smooth,
            candidate,
            requested_motion,
            model,
            environment(),
            maximum_deviation_m=0.75,
        )
        self.assertEqual(first, second)

    def test_invalid_verifier_policy_fails(self):
        smooth = geometry()
        requested_motion = motion()
        model = vehicle()
        candidate = retime_stop_path(smooth, requested_motion, model)
        for scale in (0.0, -1.0, 1.1):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                verify_candidate_trajectory(
                    smooth,
                    candidate,
                    requested_motion,
                    model,
                    environment(),
                    maximum_deviation_m=0.75,
                    maximum_limit_scale=scale,
                )


if __name__ == "__main__":
    unittest.main()
