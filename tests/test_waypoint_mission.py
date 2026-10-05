#!/usr/bin/env python3
"""Contract and smoothness tests for local-NED waypoint missions."""
# ruff: noqa: E402

from copy import deepcopy
from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "trajectory_generator"
sys.path.insert(0, str(GENERATOR))

from plan_waypoint_mission import compile_plan, main as planner_main, parse_args
from waypoint_mission import (
    WaypointMission,
    _validate_points,
    load_mission,
    plan_mission,
)


def example_dict():
    return json.loads(
        (ROOT / "missions" / "five_stop_example.json").read_text(
            encoding="utf-8"
        )
    )


class WaypointMissionTests(unittest.TestCase):
    def test_five_named_stops_are_exact_c3_boundaries(self):
        mission = WaypointMission.from_dict(example_dict())
        planned = plan_mission(mission)

        self.assertEqual(len(planned.stop_arrival_s), 5)
        self.assertEqual(len(planned.segments), 4)
        self.assertAlmostEqual(planned.points[0].time, 0.0)
        self.assertAlmostEqual(planned.points[-1].time, planned.duration_s)
        self.assertTrue(all(
            right.time > left.time
            for left, right in zip(
                planned.points, planned.points[1:], strict=False
            )
        ))
        intervals = [
            right.time - left.time
            for left, right in zip(
                planned.points, planned.points[1:], strict=False
            )
        ]
        self.assertLessEqual(max(intervals), mission.sample_period_s + 1e-12)
        self.assertGreater(min(intervals), mission.sample_period_s * 0.95)

        by_name = {stop.name: stop for stop in mission.stops}
        by_time = {point.time: point for point in planned.points}
        for name, arrival in planned.stop_arrival_s.items():
            point = by_time[arrival]
            stop = by_name[name]
            self.assertEqual((point.x, point.y, point.z), stop.position)
            self.assertTrue(math.isclose(
                math.atan2(math.sin(point.yaw), math.cos(point.yaw)),
                math.atan2(
                    math.sin(math.radians(stop.yaw_deg)),
                    math.cos(math.radians(stop.yaw_deg)),
                ),
                abs_tol=1e-12,
            ))
            derivatives = (
                point.vx, point.vy, point.vz, point.yaw_rate,
                point.ax, point.ay, point.az, point.yaw_acceleration,
                point.jx, point.jy, point.jz, point.yaw_jerk,
            )
            self.assertEqual(derivatives, (0.0,) * len(derivatives))

    def test_analytic_segment_peaks_stay_within_every_limit(self):
        mission = WaypointMission.from_dict(example_dict())
        planned = plan_mission(mission)
        limits = mission.limits
        for segment in planned.segments:
            self.assertLessEqual(
                segment.peak_horizontal_speed_m_s,
                limits.max_horizontal_speed_m_s * 1.0000001,
            )
            self.assertLessEqual(
                segment.peak_vertical_speed_m_s,
                limits.max_vertical_speed_m_s * 1.0000001,
            )
            self.assertLessEqual(
                segment.peak_acceleration_m_s2,
                limits.max_acceleration_m_s2 * 1.0000001,
            )
            self.assertLessEqual(
                segment.peak_jerk_m_s3,
                limits.max_jerk_m_s3 * 1.0000001,
            )
            self.assertLessEqual(
                segment.peak_yaw_rate_deg_s,
                limits.max_yaw_rate_deg_s * 1.0000001,
            )
            self.assertLessEqual(
                segment.peak_yaw_acceleration_deg_s2,
                limits.max_yaw_acceleration_deg_s2 * 1.0000001,
            )
            self.assertLessEqual(
                segment.peak_yaw_jerk_deg_s3,
                limits.max_yaw_jerk_deg_s3 * 1.0000001,
            )

    def test_holds_are_stationary_and_exact_duration(self):
        mission = WaypointMission.from_dict(example_dict())
        planned = plan_mission(mission)
        first_arrival = planned.stop_arrival_s["start"]
        first_departure = first_arrival + mission.stops[0].hold_s
        hold = [
            point for point in planned.points
            if first_arrival <= point.time <= first_departure
        ]
        self.assertGreater(len(hold), 1)
        for point in hold:
            self.assertEqual((point.x, point.y, point.z), mission.stops[0].position)
            self.assertEqual(
                (point.vx, point.vy, point.vz, point.ax, point.ay, point.az,
                 point.jx, point.jy, point.jz),
                (0.0,) * 9,
            )

    def test_yaw_uses_shortest_wrapped_change(self):
        raw = example_dict()
        raw["stops"] = raw["stops"][:2]
        raw["stops"][0].update(yaw_deg=170.0, hold_s=0.0)
        raw["stops"][1].update(
            north_m=0.0,
            east_m=0.0,
            down_m=-8.0,
            yaw_deg=-170.0,
            hold_s=0.0,
        )
        planned = plan_mission(WaypointMission.from_dict(raw))
        self.assertAlmostEqual(planned.segments[0].yaw_change_deg, 20.0)
        self.assertAlmostEqual(
            planned.points[-1].yaw - planned.points[0].yaw,
            math.radians(20.0),
        )

    def test_compilation_is_byte_deterministic_and_loadable(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            first = directory / "first.csv"
            second = directory / "second.csv"
            summary = directory / "summary.json"
            one = compile_plan(
                ROOT / "missions" / "five_stop_example.json",
                first,
                summary,
            )
            two = compile_plan(
                ROOT / "missions" / "five_stop_example.json",
                second,
            )
            self.assertEqual(first.read_bytes(), second.read_bytes())
            self.assertEqual(one["trajectory_sha256"], two["trajectory_sha256"])
            self.assertEqual(
                json.loads(summary.read_text(encoding="utf-8"))["point_count"],
                one["point_count"],
            )
            self.assertEqual(len(one["source_plan_sha256"]), 64)
            self.assertEqual(
                one["stop_departure_s"]["start"],
                one["stop_arrival_s"]["start"] + 1.0,
            )

    def test_unknown_missing_nonfinite_and_unsafe_inputs_fail_closed(self):
        mutations = []
        unknown = deepcopy(example_dict())
        unknown["stops"][0]["latitude"] = 1.0
        mutations.append((unknown, "unknown fields"))
        missing = deepcopy(example_dict())
        del missing["limits"]["max_jerk_m_s3"]
        mutations.append((missing, "missing fields"))
        nonfinite = deepcopy(example_dict())
        nonfinite["stops"][0]["north_m"] = math.nan
        mutations.append((nonfinite, "finite"))
        duplicate = deepcopy(example_dict())
        duplicate["stops"][1] = dict(duplicate["stops"][0], name="duplicate")
        mutations.append((duplicate, "coincident"))
        radius = deepcopy(example_dict())
        radius["stops"][1]["north_m"] = 100.0
        mutations.append((radius, "max_radius"))
        distance = deepcopy(example_dict())
        distance["safety"]["max_total_distance_m"] = 1.0
        mutations.append((distance, "max_total_distance"))
        yaw = deepcopy(example_dict())
        yaw["stops"][0]["yaw_deg"] = 181.0
        mutations.append((yaw, "yaw_deg"))
        version = deepcopy(example_dict())
        version["schema_version"] = 1.0
        mutations.append((version, "schema_version"))
        for raw, message in mutations:
            with self.subTest(message=message), self.assertRaisesRegex(
                ValueError, message
            ):
                WaypointMission.from_dict(raw)

    def test_loader_rejects_nonstandard_json_constants(self):
        raw = example_dict()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            text = json.dumps(raw).replace("0.0", "NaN", 1)
            path.write_text(text, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-finite JSON"):
                load_mission(path)

    def test_loader_reports_missing_and_malformed_files(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            with self.assertRaisesRegex(ValueError, "cannot load mission plan"):
                load_mission(directory / "missing.json")
            malformed = directory / "malformed.json"
            malformed.write_text("{", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "cannot load mission plan"):
                load_mission(malformed)

    def test_contract_structure_and_scalar_bounds_fail_closed(self):
        variants = []
        missing = example_dict()
        del missing["frame"]
        variants.append(missing)
        frame = example_dict()
        frame["frame"] = "WGS84"
        variants.append(frame)
        boolean = example_dict()
        boolean["limits"]["max_jerk_m_s3"] = True
        variants.append(boolean)
        zero_limit = example_dict()
        zero_limit["limits"]["max_jerk_m_s3"] = 0.0
        variants.append(zero_limit)
        unknown_limit = example_dict()
        unknown_limit["limits"]["unknown"] = 1.0
        variants.append(unknown_limit)
        missing_safety = example_dict()
        del missing_safety["safety"]["max_duration_s"]
        variants.append(missing_safety)
        zero_safety = example_dict()
        zero_safety["safety"]["max_duration_s"] = 0.0
        variants.append(zero_safety)
        bad_name = example_dict()
        bad_name["stops"][0]["name"] = " "
        variants.append(bad_name)
        missing_stop = example_dict()
        del missing_stop["stops"][0]["north_m"]
        variants.append(missing_stop)
        negative_hold = example_dict()
        negative_hold["stops"][0]["hold_s"] = -1.0
        variants.append(negative_hold)
        duplicate_names = example_dict()
        duplicate_names["stops"][1]["name"] = "start"
        variants.append(duplicate_names)
        vertical = example_dict()
        vertical["stops"][1]["down_m"] = -20.0
        variants.append(vertical)
        for sample_period in (0.001, 0.2):
            sample = example_dict()
            sample["sample_period_s"] = sample_period
            variants.append(sample)
        for count in (1, 65):
            count_variant = example_dict()
            count_variant["stops"] = [
                dict(count_variant["stops"][0], name=f"stop-{index}")
                for index in range(count)
            ]
            variants.append(count_variant)
        for raw in ([], {"schema_version": 1}, *variants):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                WaypointMission.from_dict(raw)

    def test_defensive_sample_audit_rejects_nonfinite_and_excess_motion(self):
        mission = WaypointMission.from_dict(example_dict())
        point = deepcopy(plan_mission(mission).points[0])
        point.vx = mission.limits.max_horizontal_speed_m_s * 2.0
        with self.assertRaisesRegex(RuntimeError, "motion limits"):
            _validate_points([point], mission.limits)
        point.vx = math.nan
        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            _validate_points([point], mission.limits)

    def test_cli_parser_and_entrypoint_write_requested_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            output = directory / "mission.csv"
            summary = directory / "mission.json"
            args = parse_args([
                "--plan", str(ROOT / "missions" / "five_stop_example.json"),
                "--output", str(output),
                "--summary", str(summary),
            ])
            self.assertEqual(args.output, output)
            with redirect_stdout(io.StringIO()) as stdout:
                planner_main([
                    "--plan",
                    str(ROOT / "missions" / "five_stop_example.json"),
                    "--output",
                    str(output),
                    "--summary",
                    str(summary),
                ])
            self.assertTrue(output.is_file())
            self.assertTrue(summary.is_file())
            self.assertEqual(json.loads(stdout.getvalue())["point_count"], 2520)

    def test_duration_budget_is_enforced_after_retiming(self):
        raw = example_dict()
        raw["safety"]["max_duration_s"] = 1.0
        mission = WaypointMission.from_dict(raw)
        with self.assertRaisesRegex(ValueError, "planned duration"):
            plan_mission(mission)


if __name__ == "__main__":
    unittest.main()
