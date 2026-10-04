#!/usr/bin/env python3
# ruff: noqa: E402
"""MAVLink runner tests using deterministic protocol doubles."""

import math
import struct
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))

import offboard_runner
from mission_state import FailureAction, MissionOutcome
from mission_config import MissionConfig
from pymavlink import mavutil
from px4_policy import decode_px4_parameter_value, read_px4_parameters


class FakeMav:
    def __init__(self):
        self.parameter_requests = []
        self.mode_requests = []

    def param_request_read_send(self, system, component, name, index):
        self.parameter_requests.append((system, component, name, index))

    def set_mode_send(self, system, flags, custom_mode):
        self.mode_requests.append((system, flags, custom_mode))


class FakeParameterMessage:
    def __init__(
        self,
        system,
        component,
        name,
        value,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    ):
        self._system = system
        self._component = component
        self.param_id = name
        self.param_value = value
        self.param_type = param_type

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
        self.config = MissionConfig()
        self.mav_send_lock = threading.Lock()
        self.native_samples = 0
        self.native_phases = []
        self.failsafe_handoff_started = False

    def log_native_landing_sample(self, phase="PX4_LAND"):
        self.native_samples += 1
        self.native_phases.append(phase)

    def begin_failsafe_handoff(self):
        self.failsafe_handoff_started = True

    def _snapshot(self, _now):
        return {"vx": 0., "vy": 0., "vz": 0., "roll": 0., "pitch": 0.,
                "position_age_s": .05, "attitude_age_s": .05}


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

    def test_parameter_reader_decodes_px4_bytewise_int32(self):
        packed_value = struct.unpack(">f", struct.pack(">i", 7))[0]
        master = FakeMaster([
            FakeParameterMessage(
                1,
                1,
                b"EKF2_GPS_CTRL",
                packed_value,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
        ])

        result = read_px4_parameters(
            FakeController(master),
            ("EKF2_GPS_CTRL",),
            attempts=1,
            timeout_s=0.05,
        )

        self.assertEqual(result, {"EKF2_GPS_CTRL": 7})

    def test_parameter_decoder_rejects_unsupported_64_bit_type(self):
        with self.assertRaisesRegex(RuntimeError, "Unsupported MAVLink"):
            decode_px4_parameter_value(
                0.0,
                mavutil.mavlink.MAV_PARAM_TYPE_INT64,
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
            patch.object(
                offboard_runner,
                "propulsion_snapshot",
                return_value=((0.0, 0.0, 0.0, 0.0), 0.1, 5),
            ),
            patch.object(offboard_runner.time, "sleep", return_value=None),
        ):
            offboard_runner.wait_for_native_landing(
                controller,
                after_sequence=10,
                timeout=1.0,
                post_disarm_confirm_s=0.0,
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

    def test_native_landing_waits_for_zero_propulsion(self):
        controller = FakeController()
        clock = TickClock(step=0.05)
        propulsion = iter([
            ((0.12, 0.12, 0.12, 0.12), 0.1, 4),
            ((0.0, 0.0, 0.0, 0.0), 0.1, 5),
            ((0.0, 0.0, 0.0, 0.0), 0.1, 6),
            ((0.0, 0.0, 0.0, 0.0), 0.1, 7),
        ])

        with (
            patch.object(offboard_runner.time, "monotonic", clock),
            patch.object(offboard_runner.time, "sleep", return_value=None),
            patch.object(
                offboard_runner,
                "heartbeat_snapshot",
                return_value=(4, 6, False),
            ),
            patch.object(
                offboard_runner,
                "landing_snapshot",
                return_value=(1, 0.1, 12),
            ),
            patch.object(
                offboard_runner,
                "propulsion_snapshot",
                side_effect=lambda _controller: next(propulsion),
            ),
        ):
            offboard_runner.wait_for_native_landing(
                controller,
                after_sequence=10,
                timeout=2.0,
                post_disarm_confirm_s=0.1,
            )

        self.assertGreaterEqual(controller.native_samples, 3)

    def test_unsafe_contact_is_reported_only_after_zero_propulsion_confirmation(self):
        controller = FakeController()
        controller._snapshot = lambda _now: {
            "vx": .8, "vy": 0., "vz": 0., "roll": 0., "pitch": 0.,
            "position_age_s": .05, "attitude_age_s": .05,
        }
        propulsion = iter([((.12,) * 4, .1, 4), ((0.,) * 4, .1, 5),
                           ((0.,) * 4, .1, 6), ((0.,) * 4, .1, 7)])
        with (
            patch.object(offboard_runner.time, "monotonic", TickClock(.05)),
            patch.object(offboard_runner.time, "sleep", return_value=None),
            patch.object(offboard_runner, "heartbeat_snapshot", return_value=(4, 6, False)),
            patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12)),
            patch.object(offboard_runner, "propulsion_snapshot",
                         side_effect=lambda _controller: next(propulsion)),
        ):
            with self.assertRaises(offboard_runner.UnsafeTouchdown):
                offboard_runner.wait_for_native_landing(controller, 10, 2., .1)
        self.assertGreaterEqual(controller.native_samples, 3)

    def test_contact_violation_between_first_and_final_samples_is_retained(self):
        for source in ("logged", "current"):
            with self.subTest(source=source):
                controller = FakeController()
                good = controller._snapshot(0.)

                def sample(_now, good=good, source=source, controller=controller):
                    pose = dict(good)
                    if source == "current" and controller.native_samples == 2:
                        pose["roll"] = math.radians(10.01)
                    return pose

                def log_sample(phase="PX4_LAND", good=good, source=source, controller=controller):
                    controller.native_samples += 1
                    pose = dict(good, landed_state=1, extended_state_seq=12,
                                extended_state_age_s=.05)
                    if source == "logged" and controller.native_samples == 2:
                        pose["roll"] = math.radians(10.01)
                    return pose

                controller._snapshot = sample
                controller.log_native_landing_sample = log_sample
                with (
                    patch.object(offboard_runner.time, "monotonic", TickClock(.01)),
                    patch.object(offboard_runner.time, "sleep", return_value=None),
                    patch.object(offboard_runner, "heartbeat_snapshot", return_value=(4, 6, False)),
                    patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12)),
                    patch.object(offboard_runner, "propulsion_snapshot",
                                 side_effect=lambda _c, controller=controller: ((.12,) * 4 if controller.native_samples == 1
                                                         else (0.,) * 4, .1, 12)),
                ):
                    with self.assertRaisesRegex(offboard_runner.UnsafeTouchdown, "tilt"):
                        offboard_runner.wait_for_native_landing(controller, 10, 2., .3)
                self.assertGreaterEqual(controller.native_samples, 3)

    def test_contact_position_violation_survives_until_safe_motor_shutdown(self):
        controller = FakeController()
        controller.native_landing_reference_xy = (0., 0.)

        def pose(_now):
            return {"x": 2. if controller.native_samples == 1 else 0., "y": 0.,
                    "vx": 0., "vy": 0., "vz": 0., "roll": 0., "pitch": 0.,
                    "position_age_s": .05, "attitude_age_s": .05}

        controller._snapshot = pose
        with (
            patch.object(offboard_runner.time, "monotonic", TickClock(.05)),
            patch.object(offboard_runner.time, "sleep", return_value=None),
            patch.object(offboard_runner, "heartbeat_snapshot", return_value=(4, 6, False)),
            patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12)),
            patch.object(offboard_runner, "propulsion_snapshot",
                         side_effect=lambda _c: ((.12,) * 4 if controller.native_samples == 1
                                                 else (0.,) * 4, .1, 12)),
        ):
            with self.assertRaisesRegex(offboard_runner.UnsafeTouchdown, "position exceeded"):
                offboard_runner.wait_for_native_landing(controller, 10, 2., .2)
        self.assertGreaterEqual(controller.native_samples, 3)

    def test_invalid_contact_estimate_cannot_establish_position_quality(self):
        controller = FakeController()
        controller.native_landing_reference_xy = (0., 0.)
        original = controller._snapshot
        controller._snapshot = lambda now: dict(original(now), x=0., y=0.)
        controller._local_estimate_valid = lambda pose: False
        with (
            patch.object(offboard_runner.time, "monotonic", TickClock(.05)),
            patch.object(offboard_runner.time, "sleep", return_value=None),
            patch.object(offboard_runner, "heartbeat_snapshot", return_value=(4, 6, False)),
            patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12)),
            patch.object(offboard_runner, "propulsion_snapshot", return_value=((0.,) * 4, .1, 12)),
        ):
            with self.assertRaisesRegex(offboard_runner.UnsafeTouchdown, "estimate invalid"):
                offboard_runner.wait_for_native_landing(controller, 10, 2., .1)

    def test_failsafe_cleanup_waits_for_land_disarm_and_zero_propulsion(self):
        controller = FakeController()
        heartbeats = iter([
            (6, 0, True),
            (4, 6, True),
            (4, 6, True),
            (4, 6, False),
        ])
        landing_states = iter([
            (2, 0.1, 11),
            (1, 0.1, 12),
        ])

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
            patch.object(
                offboard_runner,
                "propulsion_snapshot",
                return_value=((0.0, 0.0, 0.0, 0.0), 0.1, 8),
            ),
            patch.object(offboard_runner.time, "sleep", return_value=None),
        ):
            offboard_runner.wait_for_failsafe_landing(
                controller,
                after_sequence=10,
                landing_timeout=2.0,
                takeover_timeout=1.0,
                post_disarm_confirm_s=0.0,
            )

        self.assertTrue(controller.failsafe_handoff_started)
        self.assertGreaterEqual(controller.native_samples, 3)
        self.assertTrue(
            all(phase == "PX4_FAILSAFE" for phase in controller.native_phases)
        )

    def test_failsafe_cleanup_rejects_unexpected_takeover_mode(self):
        controller = FakeController()

        with patch.object(
            offboard_runner,
            "heartbeat_snapshot",
            return_value=(4, 3, True),
        ), patch.object(
            offboard_runner.time,
            "monotonic",
            TickClock(),
        ), patch.object(offboard_runner.time, "sleep", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "settle into LAND"):
                offboard_runner.wait_for_failsafe_landing(
                    controller,
                    after_sequence=10,
                    landing_timeout=2.0,
                    takeover_timeout=1.0,
                )


if __name__ == "__main__":
    unittest.main(verbosity=2)
