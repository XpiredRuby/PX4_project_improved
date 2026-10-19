"""Synthetic cancellation lifecycle proof; no simulator or aircraft is flown."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.planner_sitl_campaign import cancellation_verified
from scripts.sitl_studio_cancel import StudioCancelAuthority


class StudioCancellationTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.plan = Path(tmp.name) / "retained"
        self.run = self.plan / "rehearsal/reviewed-plan"
        (self.run / "batch/reviewed-plan").mkdir(parents=True)
        (self.run / "plan").mkdir()
        self.marker = self.plan / "CANCEL_REQUEST.json"
        self.marker.write_text(json.dumps({"release_class": "SITL_ONLY",
                                            "requested": True}))
        self.manifest = self.run / "batch/reviewed-plan/run_manifest_TEST.json"
        self.manifest.write_text(json.dumps({
            "outcome": "PX4_FAILSAFE",
            "reason": "studio simulation cancellation requested",
            "cleanup_status": "px4_failsafe_land_and_disarm_confirmed",
            "cleanup_error": None,
            "final_state": {"armed": False, "landed_state": 1}}))
        self.ground = self.run / "final_ground.json"
        self.ground.write_text(json.dumps({
            "armed": False, "landed_state": 1,
            "continuous_confirmation_s": 2.1, "actuators": [0.]*4}))

    def test_cancelled_only_after_complete_ground_shutdown(self):
        self.assertTrue(cancellation_verified(self.run, self.marker))
        original = json.loads(self.ground.read_text())
        for changes in ({"armed": True}, {"landed_state": 2},
                        {"continuous_confirmation_s": 0.3},
                        {"actuators": [0, .2, 0, 0]}):
            with self.subTest(changes=changes):
                self.ground.write_text(json.dumps({**original, **changes}))
                self.assertFalse(cancellation_verified(self.run, self.marker))
        self.ground.write_text(json.dumps(original))
        m = json.loads(self.manifest.read_text())
        for changes in ({"reason": "unrelated failure"}, {"cleanup_error": "failed"},
                        {"outcome": "SUCCESS"}, {"final_state": {"armed": True, "landed_state": 1}}):
            with self.subTest(changes=changes):
                self.manifest.write_text(json.dumps({**m, **changes}))
                self.assertFalse(cancellation_verified(self.run, self.marker))
        self.manifest.write_text(json.dumps(m))
        self.marker.unlink()
        self.assertFalse(cancellation_verified(self.run, self.marker))

    def test_only_owned_sitl_guard_accepts_marker_and_never_hardware(self):
        summary = {"release_class": "SITL_ONLY", "controller_cruise_height_m": 20.}
        (self.run / "plan/summary.json").write_text(json.dumps(summary))
        with patch.object(StudioCancelAuthority, "validate"):
            guard = StudioCancelAuthority(self.run, self.marker)
            hardware = SimpleNamespace(connection_string="serial:/dev/ttyUSB0")
            with self.assertRaisesRegex(RuntimeError, "only accepts loopback"):
                guard.bind_controller(hardware)
            studio = SimpleNamespace(connection_string="udp:127.0.0.1:14540")
            with self.assertRaisesRegex(RuntimeError, "cancellation requested"):
                guard.bind_controller(studio)
            self.marker.unlink()
            guard.bind_controller(studio)
            studio.release_guard()
            self.marker.write_text(json.dumps({"release_class": "SITL_ONLY", "requested": True}))
            with self.assertRaisesRegex(RuntimeError, "cancellation requested"):
                studio.release_guard()


if __name__ == "__main__":
    unittest.main()
