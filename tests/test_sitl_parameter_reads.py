"""Parameter transport checks: loss, source identity, and bounded deadlines."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import sitl_fault_injector as injector  # noqa: E402


class Clock:
    value = 0.

    def now(self):
        self.value += .01
        return self.value


class Transport:
    target_system = 1
    target_component = 0

    def __init__(self, deliver_after=2, messages=None):
        self.requests = []
        self.deliver_after = deliver_after
        self.messages = messages if messages is not None else [self.message()]
        self.mav = SimpleNamespace(param_request_read_send=self.request)

    def request(self, *args):
        self.requests.append(args)

    @staticmethod
    def message(component=1):
        return SimpleNamespace(param_id="SIM_BAT_MIN_PCT", param_value=50., param_type=9,
                               get_srcSystem=lambda: 1, get_srcComponent=lambda: component)

    def recv_match(self, **_kwargs):
        if len(self.requests) >= self.deliver_after and self.messages:
            return self.messages.pop(0)
        return None


class ParameterReadTests(unittest.TestCase):
    def test_lost_request_is_retried_without_extending_total_deadline(self):
        clock, master = Clock(), Transport()
        with patch.object(injector.time, "monotonic", clock.now):
            self.assertEqual(injector.read_parameter(master, "SIM_BAT_MIN_PCT", 1.), (50., 9))
        self.assertEqual(len(master.requests), 2)
        self.assertEqual(master.requests[0][1], 1)
        self.assertLessEqual(clock.value, 1.05)

    def test_wrong_component_reply_cannot_satisfy_parameter_read(self):
        master = Transport(1, [Transport.message(42), Transport.message(1)])
        with patch.object(injector.time, "monotonic", Clock().now):
            self.assertEqual(injector.read_parameter(master, "SIM_BAT_MIN_PCT", 1.), (50., 9))
        self.assertEqual(master.messages, [])

    def test_unresponsive_transport_stops_at_original_deadline(self):
        clock, master = Clock(), Transport(messages=[])
        with patch.object(injector.time, "monotonic", clock.now):
            with self.assertRaises(TimeoutError):
                injector.read_parameter(master, "SIM_BAT_MIN_PCT", 1.)
        self.assertLessEqual(clock.value, 1.1)
        self.assertLessEqual(len(master.requests), 2)
        for invalid in (0., math.nan, math.inf):
            with self.assertRaises(ValueError):
                injector.read_parameter(master, "SIM_BAT_MIN_PCT", invalid)


if __name__ == "__main__":
    unittest.main()
