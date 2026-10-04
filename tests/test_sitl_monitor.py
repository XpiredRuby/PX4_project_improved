import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from sitl_monitor import monitor_child, trial_phase_clock, allow_next_trial, quality_rejection_confirmed  # noqa: E402


class SitlMonitorTests(unittest.TestCase):
    def test_align_trigger_uses_elapsed_phase_time(self):
        row = {"phase": "ALIGN", "phase_clock_s": 0., "phase_elapsed_s": .3}
        self.assertEqual(trial_phase_clock(row), .3)
        row["phase"] = "TRAJECTORY"
        self.assertEqual(trial_phase_clock(row), 0.)

    def test_failed_audit_or_injection_stops_next_trial(self):
        self.assertFalse(allow_next_trial({}, {"audit_passed": False}))
        self.assertFalse(allow_next_trial({}, {"audit_passed": True, "scenario_errors": ["failed"]}))
        self.assertTrue(allow_next_trial({}, {"audit_passed": True}))
        self.assertFalse(allow_next_trial({"expected": "quality_rejection"}, {"audit_passed": True}))
        self.assertTrue(allow_next_trial({"expected": "quality_rejection"}, {"quality_rejection_confirmed": True}))

    def test_quality_rejection_requires_native_shutdown_evidence(self):
        manifest = {"outcome": "UNSAFE_TOUCHDOWN", "cleanup_status": "landed_with_quality_violation",
                    "cleanup_error": None, "reason": "UnsafeTouchdown: touchdown tilt exceeded 10 degrees"}
        rows = [{"elapsed_s": i * .05, "armed": False, "landed_state": 1,
                 "actuator_age_s": .01, "extended_state_age_s": .1,
                 **{f"actuator_output_{j}": 0. for j in range(4)}} for i in range(21)]
        self.assertTrue(quality_rejection_confirmed(manifest, rows))
        rows[-1]["actuator_output_2"] = .2
        self.assertFalse(quality_rejection_confirmed(manifest, rows))
        rows[-1]["actuator_output_2"] = 0.
        manifest["reason"] = "touchdown telemetry unavailable"
        self.assertFalse(quality_rejection_confirmed(manifest, rows))

    @patch("sitl_monitor.time.sleep")
    def test_injection_failure_waits_until_runner_cleanup_finishes(self, sleep):
        child = Mock(returncode=0)
        child.poll.side_effect = [None, None, None, 0]
        inject = Mock(side_effect=TimeoutError("parameter reply missing"))
        record = Mock()
        code, errors = monitor_child(child, inject, record)
        self.assertEqual(code, 0)
        self.assertEqual(errors[0]["type"], "TimeoutError")
        inject.assert_called_once()
        record.assert_called_once()
        self.assertEqual(child.poll.call_count, 4)
        child.kill.assert_not_called()
        child.terminate.assert_not_called()

    @patch("sitl_monitor.time.sleep")
    def test_recording_failure_also_retains_the_parent(self, sleep):
        child = Mock(returncode=1)
        child.poll.side_effect = [None, None, 1]
        code, errors = monitor_child(
            child, Mock(side_effect=RuntimeError("injection failed")),
            Mock(side_effect=OSError("disk full")))
        self.assertEqual(code, 1)
        self.assertEqual(len(errors), 2)
        self.assertEqual(child.poll.call_count, 3)
        child.terminate.assert_not_called()

    @patch("sitl_monitor.time.sleep")
    @patch("sitl_monitor.time.monotonic", side_effect=[0, 2])
    def test_deadline_stops_injections_but_waits_for_cleanup(self, clock, sleep):
        child = Mock(returncode=0)
        child.poll.side_effect = [None, None, 0]
        inject = Mock()
        code, errors = monitor_child(child, inject, Mock(), timeout_s=1)
        self.assertEqual(code, 0)
        self.assertEqual(errors[0]["type"], "TimeoutError")
        inject.assert_not_called()
        self.assertEqual(child.poll.call_count, 3)


if __name__ == "__main__":
    unittest.main()
