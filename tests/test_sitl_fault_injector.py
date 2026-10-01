#!/usr/bin/env python3
# ruff: noqa: E402
"""Unit tests for the guarded PX4 SITL failure injector."""

import argparse
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from sitl_fault_injector import (
    AirborneGate,
    decode_parameter_value,
    encode_parameter_value,
    validate_local_connection,
    wait_while_armed,
)


class TickClock:
    def __init__(self, step=0.25):
        self.value = -step
        self.step = step

    def __call__(self):
        self.value += self.step
        return self.value


class FakeMaster:
    def __init__(self, base_modes):
        self.target_system = 1
        # pymavlink defaults to broadcast requests even after a heartbeat.
        self.target_component = 0
        self.messages = [
            SimpleNamespace(
                base_mode=base_mode,
                get_srcSystem=lambda: 1,
                get_srcComponent=lambda: 1,
            ) for base_mode in base_modes
        ]

    def recv_match(self, **_kwargs):
        return self.messages.pop(0) if self.messages else None


class SitlFaultInjectorTests(unittest.TestCase):
    def test_connection_must_be_local_udp_input(self):
        self.assertEqual(
            validate_local_connection("udpin:0.0.0.0:14550"),
            "udpin:0.0.0.0:14550",
        )
        for value in (
            "udp:127.0.0.1:14550",
            "udpin:192.168.1.10:14550",
            "udpin:0.0.0.0:not-a-port",
            "udpin:0.0.0.0:70000",
        ):
            with self.subTest(value=value):
                with self.assertRaises(argparse.ArgumentTypeError):
                    validate_local_connection(value)

    def test_int32_parameter_round_trip_matches_px4_bytewise_encoding(self):
        parameter_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        for value in (0, 1, 7, -1):
            with self.subTest(value=value):
                encoded = encode_parameter_value(value, parameter_type)
                decoded = decode_parameter_value(encoded, parameter_type)
                self.assertEqual(decoded, float(value))

    def test_airborne_gate_requires_arm_and_real_climb(self):
        gate = AirborneGate(8.0)
        disarmed = 0
        armed = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        gate.observe_heartbeat(disarmed)
        self.assertFalse(gate.observe_position(0.10))
        self.assertFalse(gate.observe_position(0.00))
        gate.observe_heartbeat(armed)
        self.assertFalse(gate.observe_position(-7.5))
        self.assertTrue(gate.observe_position(-8.1))

    def test_airborne_gate_rejects_nonfinite_position(self):
        gate = AirborneGate(1.0)
        gate.observe_heartbeat(
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        self.assertFalse(gate.observe_position(float("nan")))
        self.assertFalse(gate.observe_position(float("inf")))

    def test_scheduled_fault_delay_requires_fresh_armed_heartbeat(self):
        armed = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        with patch("sitl_fault_injector.time.monotonic", TickClock(0.2)):
            wait_while_armed(FakeMaster([armed] * 8), 1.0)

        with patch("sitl_fault_injector.time.monotonic", TickClock(0.2)):
            with self.assertRaisesRegex(RuntimeError, "disarmed"):
                wait_while_armed(FakeMaster([armed, 0]), 1.0)

        with patch("sitl_fault_injector.time.monotonic", TickClock(0.6)):
            with self.assertRaisesRegex(RuntimeError, "fresh armed heartbeat"):
                wait_while_armed(FakeMaster([]), 1.0)

    def test_fault_delay_ignores_foreign_heartbeats(self):
        armed = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        for field in ("get_srcSystem", "get_srcComponent"):
            with self.subTest(field=field):
                master = FakeMaster([0, armed, armed, armed])
                setattr(master.messages[0], field, lambda: 253)
                with patch("sitl_fault_injector.time.monotonic", TickClock(0.2)):
                    wait_while_armed(master, 1.0)

                master = FakeMaster([armed] * 8)
                for message in master.messages:
                    setattr(message, field, lambda: 253)
                with patch("sitl_fault_injector.time.monotonic", TickClock(0.2)):
                    with self.assertRaisesRegex(RuntimeError, "fresh armed"):
                        wait_while_armed(master, 1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
