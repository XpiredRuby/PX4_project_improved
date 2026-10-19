"""The browser can rehearse only a retained, exactly confirmed SITL plan."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mission_planner.editor import preview
from mission_planner.studio_jobs import StudioJobs

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from planner_sitl_campaign import prepare_case  # noqa: E402


class StudioJobTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        document = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        document["path"]["points"] = [document["path"]["points"][0], document["path"]["points"][-1]]
        cls.result = preview({"source": "drawing", "document": document})

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        autopilot = self.root / "autopilot"
        binary = autopilot / "build/px4_sitl_default/bin/px4"
        binary.parent.mkdir(parents=True)
        binary.touch()
        self.jobs = StudioJobs(self.root / "studio", autopilot, gui=False)
        self.identifier = self.jobs.retain(self.result)
        self.confirmation = {"plan_id": self.identifier, "confirm_interpretation": True,
                             "controller_trajectory_sha256": self.result["summary"]["controller_trajectory_sha256"]}

    def test_confirmation_binds_real_compiler_artifacts_without_recompiling(self):
        process = SimpleNamespace(poll=lambda: None, pid=123)
        with patch("mission_planner.studio_jobs.subprocess.Popen", return_value=process) as spawn:
            status = self.jobs.start(self.confirmation)
        self.addCleanup(self.jobs.log.close)
        self.assertEqual(status["stage"], "STARTING")
        self.assertFalse(status["flight_approval"])
        command = spawn.call_args.args[0]
        self.assertIn("--reviewed-plan", command)
        output = self.root / "trial"
        output.mkdir()
        case = prepare_case("reviewed-plan", output, self.jobs.folder(self.identifier))
        self.assertEqual(Path(case["trajectory"]).read_text(), self.result["trajectory_csv"])
        self.assertEqual(case["cruise_height_m"], self.result["summary"]["controller_cruise_height_m"])

    def test_paths_commands_missing_confirmation_and_mismatched_hash_cannot_start(self):
        invalid = [dict(self.confirmation, plan_id="../outside"),
                   dict(self.confirmation, command="arm"),
                   dict(self.confirmation, confirm_interpretation=1),
                   dict(self.confirmation, controller_trajectory_sha256="0" * 64)]
        for payload in invalid:
            with self.subTest(payload=payload), patch("mission_planner.studio_jobs.subprocess.Popen") as spawn:
                with self.assertRaises(ValueError):
                    self.jobs.start(payload)
                spawn.assert_not_called()

    def test_modified_retained_csv_and_report_refuse_execution(self):
        folder = self.jobs.folder(self.identifier)
        (folder / "trajectory.csv").write_text("altered")
        with self.assertRaisesRegex(ValueError, "does not match"):
            self.jobs.start(self.confirmation)
        (folder / "trajectory.csv").write_text(self.result["trajectory_csv"])
        self.jobs.process = Mock(poll=lambda: None)
        with self.assertRaisesRegex(ValueError, "already running"):
            self.jobs.start(self.confirmation)
        self.jobs.process = None
        with patch("mission_planner.studio_jobs.subprocess.Popen", return_value=SimpleNamespace(poll=lambda: None)):
            self.jobs.start(self.confirmation)
        self.addCleanup(self.jobs.log.close)
        report = json.loads((folder / "verification.json").read_text())
        report["accepted"] = False
        (folder / "verification.json").write_text(json.dumps(report))
        output = self.root / "trial"
        output.mkdir()
        with self.assertRaisesRegex(ValueError, "verification or intent"):
            prepare_case("reviewed-plan", output, folder)

    def test_exited_worker_never_becomes_a_success(self):
        with patch("mission_planner.studio_jobs.subprocess.Popen", return_value=SimpleNamespace(poll=lambda: 1)):
            status = self.jobs.start(self.confirmation)
        self.assertEqual(status["stage"], "FAILED")
        self.assertFalse(status["flight_approval"])


if __name__ == "__main__":
    unittest.main()
