#!/usr/bin/env python3
# ruff: noqa: E402
"""Unit tests for the guarded PX4 SITL failure injector."""

import argparse
import sys
import unittest
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from sitl_fault_injector import (
    AirborneGate,
    decode_parameter_value,
    encode_parameter_value,
    validate_local_connection,
)


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


if __name__ == "__main__":
    unittest.main(verbosity=2)
