"""The browser can rehearse only a retained, exactly confirmed SITL plan."""
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mission_planner.editor import preview
from mission_planner.studio_jobs import StudioJobs, recorded_phase

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from planner_sitl_campaign import prepare_case  # noqa: E402


class StudioJobTests(unittest.TestCase):
    def test_live_phase_uses_a_complete_telemetry_row_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            folder = run / "reviewed-plan/batch/reviewed-plan"
            folder.mkdir(parents=True)
            path = folder / "research_log_TEST.csv"
            path.write_text("time,phase,x\n0,TAKEOFF,0\n1,TRAJECTORY,1\n2,RETURN_")
            self.assertEqual(recorded_phase(run), "TRAJECTORY")
            path.write_text("time,phase,x\n")
            self.assertIsNone(recorded_phase(run))

    @classmethod
    def setUpClass(cls):
        document = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        document["path"]["points"] = [document["path"]["points"][0], document["path"]["points"][-1]]
        # Browser JSON serializes whole numbers as integers. The retained
        # document must use the compiler's normalized numeric representation.
        for point in document["path"]["points"]:
            for field in ("north_m", "east_m", "down_m"):
                point[field] = int(point[field])
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

    def test_simultaneous_confirmations_can_start_only_one_worker(self):
        other = dict(self.confirmation, plan_id=self.jobs.retain(self.result))
        barrier = threading.Barrier(2)

        def start(payload):
            barrier.wait(timeout=2)
            try:
                self.jobs.start(payload)
                return True
            except ValueError:
                return False

        def spawn(*args, **kwargs):
            time.sleep(.05)  # Expose the check-before-spawn race without the job lock.
            return SimpleNamespace(poll=lambda: None, pid=123)

        with patch("mission_planner.studio_jobs.subprocess.Popen", side_effect=spawn) as worker:
            with ThreadPoolExecutor(max_workers=2) as pool:
                outcomes = list(pool.map(start, [self.confirmation, other]))
        self.addCleanup(self.jobs.log.close)
        self.assertEqual(sum(outcomes), 1)
        self.assertEqual(worker.call_count, 1)

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

    def test_cancellation_is_exact_artifact_bound_and_never_kills_worker(self):
        process = Mock(pid=123)
        process.poll.return_value = None
        with patch("mission_planner.studio_jobs.subprocess.Popen", return_value=process):
            self.jobs.start(self.confirmation)
        self.addCleanup(self.jobs.log.close)
        payload = {"plan_id": self.identifier,
                   "controller_trajectory_sha256": self.confirmation["controller_trajectory_sha256"]}
        for wrong in (dict(payload, plan_id="../outside"),
                      dict(payload, controller_trajectory_sha256="0"*64),
                      dict(payload, force_kill=True), {}):
            with self.subTest(payload=wrong), self.assertRaises(ValueError):
                self.jobs.cancel(wrong)
        with patch.object(process, "terminate", side_effect=AssertionError("never kill"), create=True) as kill:
            status = self.jobs.cancel(payload)
            self.assertEqual(status["stage"], "STARTING")
            self.assertTrue(status["cancel_requested"])
            self.assertTrue(self.jobs.cancel(payload)["cancel_requested"])
            kill.assert_not_called()
        marker = json.loads((self.jobs.folder(self.identifier) / "CANCEL_REQUEST.json").read_text())
        self.assertEqual(marker["release_class"], "SITL_ONLY")
        self.assertEqual(marker["plan_id"], self.identifier)
        self.assertTrue(marker["requested"])
        self.assertFalse(status["flight_approval"])

    def test_no_cancel_before_start_or_after_process_exit(self):
        payload = {"plan_id": self.identifier,
                   "controller_trajectory_sha256": self.confirmation["controller_trajectory_sha256"]}
        with self.assertRaisesRegex(ValueError, "not running"):
            self.jobs.cancel(payload)
        with patch("mission_planner.studio_jobs.subprocess.Popen",
                   return_value=SimpleNamespace(poll=lambda: 1)):
            self.jobs.start(self.confirmation)
        with self.assertRaisesRegex(ValueError, "not running"):
            self.jobs.cancel(payload)
        self.assertFalse((self.jobs.folder(self.identifier) / "CANCEL_REQUEST.json").exists())

    def test_unregistered_aircraft_cannot_silently_use_the_x500_simulator(self):
        from mission_planner.aircraft import assess_profile
        profile = json.loads((ROOT / "config/planner/aircraft/hex.json").read_text())
        setup = assess_profile(profile)
        document = json.loads(json.dumps(self.result["intent"]))
        document.update(vehicle_model_id=profile["profile_id"],
                        environment_id="profile-env-" + setup["profile_sha256"][:16])
        result = preview(dict(source="drawing", document=document, aircraft_profile=profile))
        self.assertFalse(self.jobs.compatible(result))
        identifier = self.jobs.retain(result)
        confirmation = dict(self.confirmation, plan_id=identifier,
                            controller_trajectory_sha256=result["summary"]["controller_trajectory_sha256"])
        with patch("mission_planner.studio_jobs.subprocess.Popen") as spawn:
            with self.assertRaisesRegex(ValueError, "matching registered Gazebo"):
                self.jobs.start(confirmation)
            spawn.assert_not_called()


if __name__ == "__main__":
    unittest.main()
