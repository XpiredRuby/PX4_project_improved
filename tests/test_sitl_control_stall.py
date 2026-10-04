"""Check software fault injection and strict evidence classification."""
# ruff: noqa: E402
import math
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "analysis")]
from sitl_control_faults import install_control_stall, install_storage_stall
from analyze_fault_run import build_control_stall_audit, build_storage_stall_audit, build_nominal_run_audit
import test_analysis_pipeline


class SoftwareStallTests(unittest.TestCase):
    def test_nominal_audit_cannot_pass_an_incomplete_but_safely_landed_mission(self):
        frame = test_analysis_pipeline.AnalysisPipelineTests().successful_run_frame()
        manifest = {"outcome": "SUCCESS", "cleanup_error": None,
                    "final_state": {"armed": False, "landed_state": 1}}
        self.assertTrue(build_nominal_run_audit(frame, manifest)["overall_passed"])
        for change in ({"outcome": "ABORTED_TO_LAND"}, {"cleanup_error": "storage stalled"},
                       {"final_state": {"armed": True, "landed_state": 1}}):
            with self.subTest(change=change):
                self.assertFalse(build_nominal_run_audit(frame, dict(manifest, **change))["overall_passed"])
    def test_storage_stall_blocks_only_evidence_stream_once(self):
        import io

        class Logger:
            def __init__(self):
                self._stream = io.StringIO()

            def _run(self):
                for _ in range(4):
                    self._stream.write("row\n")
                self._stream.flush()

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "storage.json"
            install_storage_stall(Logger, 4., 2, path)
            logger = Logger()
            with patch("sitl_control_faults.time.sleep") as sleep:
                logger._run()
                sleep.assert_called_once_with(4.)
            self.assertEqual(logger._stream.stream.getvalue(), "row\n" * 4)
            self.assertTrue(json.loads(path.read_text())["completed"])

    def test_invalid_storage_stall_settings_are_rejected(self):
        for delay, rows in ((0., 100), (6., 100), (math.nan, 100), (4., 1), (4., 2.5), (4., True)):
            with self.subTest(delay=delay, rows=rows), self.assertRaises(ValueError):
                install_storage_stall(object, delay, rows, "unused.json")

    def test_storage_stall_audit_requires_exercised_delay_and_live_control(self):
        frame = test_analysis_pipeline.AnalysisPipelineTests().successful_run_frame()
        frame["monotonic_time"] = [10., 10.1, 10.2, 10.3, 10.4, 10.5]
        manifest = {"outcome": "SUCCESS", "cleanup_error": None,
                    "final_state": {"armed": False, "landed_state": 1}}
        injection = {"started": True, "completed": True, "delay_s": .2,
                     "started_monotonic": 10., "ended_monotonic": 10.3}
        self.assertTrue(build_storage_stall_audit(frame, manifest, injection)["overall_passed"])
        self.assertFalse(build_storage_stall_audit(frame, manifest, {})["overall_passed"])
        self.assertFalse(build_storage_stall_audit(frame, dict(manifest, outcome="PX4_FAILSAFE"), injection)["overall_passed"])
        frame.loc[:3, ["cmd_vx", "cmd_vy", "cmd_vz"]] = math.nan
        self.assertFalse(build_storage_stall_audit(frame, manifest, injection)["overall_passed"])

    def test_control_stall_is_once_only_and_does_not_pause_other_phases(self):
        class Worker:
            phase = "TAKEOFF"
            phase_clock_s = 3.

            def _build_log_row(self, **kwargs):
                return kwargs

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "injection.json"
            install_control_stall(Worker, 2., 2., path)
            worker = Worker()
            with patch("sitl_control_faults.time.sleep") as sleep:
                self.assertEqual(worker._build_log_row(count=1), {"count": 1})
                sleep.assert_not_called()
                worker.phase = "TRAJECTORY"
                worker._build_log_row(count=2)
                worker._build_log_row(count=3)
                sleep.assert_called_once_with(2.)
            self.assertTrue(path.exists())

    def test_invalid_stall_settings_are_rejected(self):
        for duration, clock in ((0., 2.), (6., 2.), (math.nan, 2.), (2., -1.), (2., math.inf)):
            with self.subTest(duration=duration, clock=clock), self.assertRaises(ValueError):
                install_control_stall(object, duration, clock, "unused.json")

    def evidence(self):
        frame = test_analysis_pipeline.AnalysisPipelineTests().successful_run_frame()
        frame.loc[2:, "phase"] = "PX4_FAILSAFE"
        frame["setpoint_send_count"] = [1, 2, 5, 5, 5, 5]
        frame["setpoint_failsafe_brakes"] = [0, 0, 1, 1, 1, 1]
        manifest = {"outcome": "PX4_FAILSAFE", "cleanup_error": None,
                    "cleanup_status": "px4_failsafe_land_and_disarm_confirmed",
                    "reason": "Controller failed (setpoint_error): Control command expired",
                    "final_state": {"armed": False, "landed_state": 1,
                                    "failure_action": "PX4_FAILSAFE"}}
        return frame, manifest, {"started": True, "completed": True}

    def test_stall_abort_requires_safe_touchdown_and_no_later_sends(self):
        frame, manifest, injection = self.evidence()
        audit = build_control_stall_audit(frame, manifest, injection)
        self.assertTrue(audit["overall_passed"])
        json.dumps(audit, allow_nan=False)
        frame.loc[5, "setpoint_send_count"] = 6
        self.assertFalse(build_control_stall_audit(frame, manifest, injection)["overall_passed"])
        frame.loc[5, "setpoint_send_count"] = 5
        frame.loc[5, "x"] = 2.
        self.assertFalse(build_control_stall_audit(frame, manifest, injection)["overall_passed"])

    def test_stall_abort_cannot_pass_without_exercised_fault_and_shutdown(self):
        frame, manifest, injection = self.evidence()
        for change in ({"reason": "unrelated exception"}, {"cleanup_error": "failed"},
                       {"outcome": "SUCCESS"}):
            with self.subTest(change=change):
                self.assertFalse(build_control_stall_audit(frame, dict(manifest, **change), injection)["overall_passed"])
        self.assertFalse(build_control_stall_audit(frame, manifest, {})["overall_passed"])


if __name__ == "__main__":
    unittest.main()
