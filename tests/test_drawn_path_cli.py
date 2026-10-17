"""End-to-end tests for the initial drawn-path compiler."""

from contextlib import redirect_stdout
import csv
import io
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from mission_planner.export import FIELDS
from mission_planner.plan_drawn_path import (
    compile_drawn_path,
    main,
    parse_args,
)


ROOT = Path(__file__).resolve().parents[1]
INTENT = ROOT / "missions" / "drawn_path_example.json"
VEHICLE = ROOT / "config" / "planner" / "x500_sitl_vehicle_model.json"
ENVIRONMENT = ROOT / "config" / "planner" / "empty_world_environment.json"


def compile_into(directory: Path, prefix: str):
    return compile_drawn_path(
        intent_path=INTENT,
        vehicle_path=VEHICLE,
        environment_path=ENVIRONMENT,
        trajectory_path=directory / f"{prefix}.csv",
        report_path=directory / f"{prefix}.report.json",
        summary_path=directory / f"{prefix}.summary.json",
        control_spacing_m=0.5,
        verification_resolution_m=0.02,
        sample_period_s=0.1,
        limit_scale=0.9,
        controller_cruise_height_m=17.0,
    )


class DrawnPathCompilerTests(unittest.TestCase):
    def test_compiler_is_deterministic_and_controller_compatible(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            first = compile_into(directory, "first")
            second = compile_into(directory, "second")
            self.assertEqual(
                (directory / "first.csv").read_bytes(),
                (directory / "second.csv").read_bytes(),
            )
            self.assertEqual(
                (directory / "first.report.json").read_bytes(),
                (directory / "second.report.json").read_bytes(),
            )
            self.assertEqual(
                first["candidate_trajectory_sha256"],
                second["candidate_trajectory_sha256"],
            )
            self.assertEqual(first["release_class"], "SITL_ONLY")
            self.assertIn("SITL_ONLY", first["limitations"])
            report = json.loads(
                (directory / "first.report.json").read_text(encoding="utf-8")
            )
            self.assertTrue(report["accepted"])
            with (directory / "first.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(tuple(rows[0]), FIELDS)
            self.assertEqual(len(rows), first["point_count"])
            self.assertEqual(float(rows[0]["time"]), 0.0)
            self.assertEqual(
                float(rows[-1]["time"]),
                first["retiming"]["selected_duration_s"],
            )
            module_name = "controller_trajectory_drawn_path_test"
            specification = importlib.util.spec_from_file_location(
                module_name,
                ROOT / "controller" / "trajectory.py",
            )
            self.assertIsNotNone(specification)
            self.assertIsNotNone(specification.loader)
            module = importlib.util.module_from_spec(specification)
            sys.modules[module_name] = module
            try:
                specification.loader.exec_module(module)
                loaded = module.Trajectory(directory / "first.csv")
            finally:
                sys.modules.pop(module_name, None)
            self.assertEqual(len(loaded.points), first["point_count"])
            self.assertEqual(
                loaded.duration,
                first["retiming"]["selected_duration_s"],
            )

    def test_mismatched_contracts_fail_before_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            intent = json.loads(INTENT.read_text(encoding="utf-8"))
            intent["vehicle_model_id"] = "wrong-model"
            invalid = directory / "invalid.json"
            invalid.write_text(json.dumps(intent), encoding="utf-8")
            trajectory = directory / "trajectory.csv"
            report = directory / "report.json"
            summary = directory / "summary.json"
            with self.assertRaisesRegex(ValueError, "identifiers do not match"):
                compile_drawn_path(
                    intent_path=invalid,
                    vehicle_path=VEHICLE,
                    environment_path=ENVIRONMENT,
                    trajectory_path=trajectory,
                    report_path=report,
                    summary_path=summary,
                    control_spacing_m=0.5,
                    verification_resolution_m=0.02,
                    sample_period_s=0.1,
                    limit_scale=0.9,
                    controller_cruise_height_m=17.0,
                )
            self.assertFalse(trajectory.exists())
            self.assertFalse(report.exists())
            self.assertFalse(summary.exists())

    def test_controller_origin_mismatch_fails_before_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            intent = json.loads(INTENT.read_text(encoding="utf-8"))
            intent["path"]["points"][0]["north_m"] = 1.0
            invalid = directory / "invalid.json"
            invalid.write_text(json.dumps(intent), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "controller rebasing"):
                compile_drawn_path(
                    intent_path=invalid,
                    vehicle_path=VEHICLE,
                    environment_path=ENVIRONMENT,
                    trajectory_path=directory / "trajectory.csv",
                    report_path=directory / "report.json",
                    summary_path=directory / "summary.json",
                    control_spacing_m=0.5,
                    verification_resolution_m=0.02,
                    sample_period_s=0.1,
                    limit_scale=0.9,
                    controller_cruise_height_m=17.0,
                )
            self.assertFalse((directory / "trajectory.csv").exists())

    def test_output_paths_must_not_alias(self):
        with tempfile.TemporaryDirectory() as directory:
            shared = Path(directory) / "shared.output"
            with self.assertRaisesRegex(ValueError, "must be distinct"):
                compile_drawn_path(
                    intent_path=INTENT,
                    vehicle_path=VEHICLE,
                    environment_path=ENVIRONMENT,
                    trajectory_path=shared,
                    report_path=shared,
                    summary_path=Path(directory) / "summary.json",
                    control_spacing_m=0.5,
                    verification_resolution_m=0.02,
                    sample_period_s=0.1,
                    limit_scale=0.9,
                    controller_cruise_height_m=17.0,
                )

    def test_cli_writes_requested_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            arguments = [
                "--intent", str(INTENT),
                "--vehicle", str(VEHICLE),
                "--environment", str(ENVIRONMENT),
                "--trajectory", str(directory / "trajectory.csv"),
                "--report", str(directory / "report.json"),
                "--summary", str(directory / "summary.json"),
                "--control-spacing-m", "0.5",
                "--verification-resolution-m", "0.02",
                "--sample-period-s", "0.1",
            ]
            parsed = parse_args(arguments)
            self.assertEqual(parsed.intent, INTENT)
            with redirect_stdout(io.StringIO()) as output:
                main(arguments)
            printed = json.loads(output.getvalue())
            self.assertEqual(printed["release_class"], "SITL_ONLY")
            self.assertTrue((directory / "trajectory.csv").is_file())
            self.assertTrue((directory / "report.json").is_file())
            self.assertTrue((directory / "summary.json").is_file())

    def test_publication_failure_restores_previous_output_set(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            trajectory = directory / "trajectory.csv"
            report = directory / "report.json"
            summary = directory / "summary.json"
            previous = {
                trajectory: b"old trajectory\n",
                report: b'{"old": "report"}\n',
                summary: b'{"old": "summary"}\n',
            }
            for path, content in previous.items():
                path.write_bytes(content)
            original_replace = Path.replace

            def fail_trajectory_publication(source, target):
                if (
                    source.name.startswith("trajectory.csv.pending-")
                    and target == trajectory
                ):
                    raise OSError("injected trajectory publication failure")
                return original_replace(source, target)

            with patch.object(
                Path,
                "replace",
                autospec=True,
                side_effect=fail_trajectory_publication,
            ), self.assertRaisesRegex(OSError, "injected"):
                compile_drawn_path(
                    intent_path=INTENT,
                    vehicle_path=VEHICLE,
                    environment_path=ENVIRONMENT,
                    trajectory_path=trajectory,
                    report_path=report,
                    summary_path=summary,
                    control_spacing_m=0.5,
                    verification_resolution_m=0.02,
                    sample_period_s=0.1,
                    limit_scale=0.9,
                    controller_cruise_height_m=17.0,
                )
            for path, content in previous.items():
                self.assertEqual(path.read_bytes(), content)
            self.assertFalse(list(directory.glob("*.pending-*")))
            self.assertFalse(list(directory.glob("*.backup-*")))


if __name__ == "__main__":
    unittest.main()
