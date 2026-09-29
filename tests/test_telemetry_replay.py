#!/usr/bin/env python3
# ruff: noqa: E402
"""Replay a captured-shape telemetry sequence through the real dispatcher."""

import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "controller"
sys.path.insert(0, str(CONTROLLER))

from PID_position_new import PositionController


class ReplayMessage(SimpleNamespace):
    def get_type(self):
        return self.message_type

    def get_srcSystem(self):
        return self.source_system

    def get_srcComponent(self):
        return self.source_component


class TelemetryReplayTests(unittest.TestCase):
    def test_sequence_filters_foreign_data_and_latches_time_regression(self):
        fixture = json.loads(
            (ROOT / "tests" / "fixtures" / "telemetry_sequence.json")
            .read_text(encoding="utf-8")
        )
        controller = PositionController()
        controller.master = SimpleNamespace(
            target_system=1,
            target_component=1,
        )

        accepted = []
        for event in fixture["events"]:
            message = ReplayMessage(
                message_type=event["type"],
                source_system=event["source_system"],
                source_component=event["source_component"],
                **event["fields"],
            )
            accepted.append(controller.handle_mavlink_message(message))

        self.assertEqual(accepted, [True, True, True, True, False, True, True])
        self.assertAlmostEqual(controller.state.x, 1.2)
        self.assertAlmostEqual(controller.state.y, -2.1)
        self.assertAlmostEqual(controller.state.z, -15.1)
        self.assertTrue(controller.state.position_source_regressed)
        self.assertEqual(
            controller.state.message_counts["LOCAL_POSITION_NED"],
            3,
        )
        self.assertEqual(controller.state.gps_fix_type, 3)
        self.assertEqual(controller.state.gps_satellites_visible, 12)
        self.assertAlmostEqual(controller.state.gps_horizontal_accuracy_m, 0.8)
        self.assertAlmostEqual(controller.state.gps_vertical_accuracy_m, 1.5)
        self.assertEqual(controller.state.landed_state, 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
