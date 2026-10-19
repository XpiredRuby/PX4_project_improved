"""Real SITL packet replay with controlled timing faults; no simulated EKF claim."""
# ruff: noqa: E402
import base64
import copy
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controller"))
from pymavlink.dialects.v20 import common
from PID_position_new import PositionController


class CapturedReplayTests(unittest.TestCase):
    def replay(self, mutation=None):
        controller = PositionController()
        controller.master = SimpleNamespace(target_system=1, target_component=1)
        events = json.loads((Path(__file__).parent / "fixtures" /
                             "captured_sitl_packets.json").read_text())["events"]
        parser = common.MAVLink(None)
        frozen = None
        offset = 1000.
        for event in events:
            message = parser.parse_buffer(base64.b64decode(event["packet_b64"]))[0]
            message = copy.copy(message)
            elapsed = event["elapsed_s"]
            if message.get_type() == "GPS_RAW_INT":
                if mutation == "freeze" and elapsed >= 1.:
                    if frozen is None:
                        frozen = message.time_usec
                    message.time_usec = frozen
                if mutation == "delay" and elapsed >= 1.:
                    # Receipt keeps advancing while sensor time advances slowly.
                    message.time_usec -= int((elapsed - 1.) * .7 * 1e6)
                if mutation == "uncertainty":
                    message.h_acc = 0xFFFFFFFF
            receipt_elapsed = elapsed * 2. if mutation == "slow_clock" else elapsed
            with patch("VehicleState.time.monotonic", return_value=offset + receipt_elapsed):
                controller.handle_mavlink_message(message)
        last_elapsed = events[-1]["elapsed_s"] * (2. if mutation == "slow_clock" else 1.)
        return controller, controller._snapshot(offset + last_elapsed)

    def test_real_wire_packets_are_parsed_and_have_valid_navigation(self):
        controller, snapshot = self.replay()
        self.assertEqual(controller._navigation_health_reasons(snapshot), [])
        self.assertGreater(controller.state.message_counts["GPS_RAW_INT"], 10)

    def test_repeated_sensor_timestamps_detected_despite_fresh_arrivals(self):
        controller, snapshot = self.replay("freeze")
        self.assertLess(snapshot["gps_age_s"], .3)
        self.assertGreater(snapshot["gps_source_age_s"], 1.5)
        self.assertIn("GPS measurement timestamp stopped advancing",
                      controller._navigation_health_reasons(snapshot))

    def test_growing_delay_detected_with_increasing_sensor_timestamps(self):
        controller, snapshot = self.replay("delay")
        self.assertFalse(snapshot["gps_source_regressed"])
        self.assertLess(snapshot["gps_source_age_s"], .3)
        self.assertGreater(snapshot["gps_source_delay_s"], 1.5)
        self.assertIn("GPS measurement delivery delay increased beyond limit",
                      controller._navigation_health_reasons(snapshot))

    def test_unknown_uncertainty_is_unhealthy(self):
        controller, snapshot = self.replay("uncertainty")
        self.assertTrue(any("horizontal accuracy" in reason
                            for reason in controller._navigation_health_reasons(snapshot)))

    def test_scaled_simulator_clock_does_not_look_like_growing_gps_delay(self):
        _, snapshot = self.replay("slow_clock")
        self.assertLess(snapshot["gps_source_delay_s"], .5)
