#!/usr/bin/env python3
# ruff: noqa: E402
"""MAVLink runner tests using deterministic protocol doubles."""

import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))

import offboard_runner
from mission_state import FailureAction, MissionOutcome
from pymavlink import mavutil
from px4_policy import read_px4_parameters


class FakeMav:
    def __init__(self):
        self.parameter_requests = []
        self.mode_requests = []

    def param_request_read_send(self, system, component, name, index):
        self.parameter_requests.append((system, component, name, index))

    def set_mode_send(self, system, flags, custom_mode):
        self.mode_requests.append((system, flags, custom_mode))


class FakeParameterMessage:
    def __init__(self, system, component, name, value):
        self._system = system
        self._component = component
        self.param_id = name
        self.param_value = value

    def get_srcSystem(self):
        return self._system

    def get_srcComponent(self):
        return self._component


class FakeMaster:
    def __init__(self, messages=()):
        self.target_system = 1
        self.target_component = 1
        self.mav = FakeMav()
        self._messages = list(messages)

    def recv_match(self, **_kwargs):
        return self._messages.pop(0) if self._messages else None


class FakeController:
    def __init__(self, master=None):
        self.master = master or FakeMaster()
        self.mav_send_lock = threading.Lock()
        self.native_samples = 0

    def log_native_landing_sample(self):
        self.native_samples += 1


class TickClock:
    def __init__(self, step=0.05):
        self.value = -step
        self.step = step

    def __call__(self):
        self.value += self.step
        return self.value


class RunnerProtocolTests(unittest.TestCase):
    def test_failure_outcome_distinguishes_prearm_land_and_px4_failsafe(self):
        controller = FakeController()
        controller.failure_action_snapshot = lambda: FailureAction.LAND
        self.assertEqual(
            offboard_runner.classify_failure(controller, False),
            MissionOutcome.PREARM_REJECTED,
        )
        self.assertEqual(
            offboard_runner.classify_failure(controller, True),
            MissionOutcome.ABORTED_TO_LAND,
        )
        controller.failure_action_snapshot = (
            lambda: FailureAction.PX4_FAILSAFE
        )
        self.assertEqual(
            offboard_runner.classify_failure(controller, True),
            MissionOutcome.PX4_FAILSAFE,
        )

    def test_parameter_reader_filters_other_vehicle_and_matches_bytes_id(self):
        master = FakeMaster([
            FakeParameterMessage(9, 1, b"COM_OF_LOSS_T", 99.0),
            FakeParameterMessage(1, 1, b"COM_OF_LOSS_T\x00", 0.5),
        ])
        controller = FakeController(master)

        result = read_px4_parameters(
            controller,
            ("COM_OF_LOSS_T",),
            attempts=1,
            timeout_s=0.05,
        )

        self.assertEqual(result, {"COM_OF_LOSS_T": 0.5})
        self.assertEqual(
            master.mav.parameter_requests,
            [(1, 1, b"COM_OF_LOSS_T", -1)],
        )

    def test_mode_requests_encode_px4_offboard_and_land(self):
        controller = FakeController()

        offboard_runner.request_mode(controller, "OFFBOARD")
        offboard_runner.request_mode(controller, "LAND")

        requests = controller.master.mav.mode_requests
        self.assertEqual(requests[0][2], 6 << 16)
        self.assertEqual(requests[1][2], (6 << 24) | (4 << 16))

    def test_ensure_mode_retries_then_times_out(self):
        controller = FakeController()
        controller.control_dt = 0.05

        with (
            patch.object(offboard_runner.time, "monotonic", TickClock()),
            patch.object(offboard_runner.time, "sleep", return_value=None),
            patch.object(
                offboard_runner,
                "heartbeat_snapshot",
                return_value=(4, 0, True),
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "did not enter OFFBOARD"):
                offboard_runner.ensure_mode(
                    controller,
                    "OFFBOARD",
                    timeout=0.3,
                    keep_streaming=False,
                )

        self.assertGreaterEqual(len(controller.master.mav.mode_requests), 1)

    def test_ensure_mode_accepts_confirmed_offboard(self):
        controller = FakeController()
        controller.control_dt = 0.05

        with patch.object(
            offboard_runner,
            "heartbeat_snapshot",
            return_value=(6, 0, True),
        ):
            offboard_runner.ensure_mode(
                controller,
                "OFFBOARD",
                timeout=1.0,
                keep_streaming=False,
            )

        self.assertEqual(len(controller.master.mav.mode_requests), 1)

    def test_command_rejection_and_ack_timeout_are_explicit(self):
        command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        controller = FakeController()

        with patch.object(
            offboard_runner,
            "command_ack_snapshot",
            return_value=(2, mavutil.mavlink.MAV_RESULT_DENIED, 0),
        ):
            with self.assertRaisesRegex(RuntimeError, "rejected command"):
                offboard_runner.wait_for_command_ack(
                    controller,
                    command,
                    after_sequence=1,
                    timeout=1.0,
                )

        with (
            patch.object(offboard_runner.time, "monotonic", TickClock()),
            patch.object(offboard_runner.time, "sleep", return_value=None),
            patch.object(
                offboard_runner,
                "command_ack_snapshot",
                return_value=(1, -1, -1),
            ),
        ):
            with self.assertRaisesRegex(TimeoutError, "No COMMAND_ACK"):
                offboard_runner.wait_for_command_ack(
                    controller,
                    command,
                    after_sequence=1,
                    timeout=0.3,
                )

    def test_native_landing_requires_new_on_ground_report_and_disarm(self):
        controller = FakeController()
        heartbeats = iter([(4, 6, True), (4, 6, False)])
        landing_states = iter([(2, 0.1, 11), (1, 0.1, 12)])

        with (
            patch.object(
                offboard_runner,
                "heartbeat_snapshot",
                side_effect=lambda _controller: next(heartbeats),
            ),
            patch.object(
                offboard_runner,
                "landing_snapshot",
                side_effect=lambda _controller: next(landing_states),
            ),
            patch.object(offboard_runner.time, "sleep", return_value=None),
        ):
            offboard_runner.wait_for_native_landing(
                controller,
                after_sequence=10,
                timeout=1.0,
            )

        self.assertEqual(controller.native_samples, 2)

    def test_disarm_without_fresh_on_ground_report_is_rejected(self):
        controller = FakeController()

        with (
            patch.object(
                offboard_runner,
                "heartbeat_snapshot",
                return_value=(4, 6, False),
            ),
            patch.object(
                offboard_runner,
                "landing_snapshot",
                return_value=(1, 2.0, 10),
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "fresh PX4 ON_GROUND"):
                offboard_runner.wait_for_native_landing(
                    controller,
                    after_sequence=10,
                    timeout=1.0,
                )


if __name__ == "__main__":
    unittest.main(verbosity=2)
