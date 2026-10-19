"""Ground-evidence gate regressions for the isolated simulator harness."""

import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import planner_sitl_campaign as campaign  # noqa: E402


class Clock:
    def __init__(self):
        self.value = 0.

    def now(self):
        self.value += .025
        return self.value


def master(armed=False, landed=1, output=0., component=1):
    def packet(kind, **values):
        return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: 1,
                               get_srcComponent=lambda: component, **values)
    stream = iter([None] + [value for _ in range(100) for value in [
        packet("HEARTBEAT", base_mode=128 if armed else 0),
        packet("EXTENDED_SYS_STATE", landed_state=landed),
        packet("ACTUATOR_OUTPUT_STATUS", actuator=[output] * 4),
    ]])
    return SimpleNamespace(target_system=1, mav=SimpleNamespace(command_long_send=Mock()),
                           recv_match=lambda **kwargs: next(stream, None))


class GroundGateTests(unittest.TestCase):
    def test_continuous_fresh_ground_and_zero_output_are_required(self):
        with patch.object(campaign.time, "monotonic", Clock().now):
            evidence = campaign.ground_evidence(master())
        self.assertFalse(evidence["armed"])
        self.assertEqual(evidence["landed_state"], 1)
        self.assertGreaterEqual(evidence["continuous_confirmation_s"], 2.)

    def test_armed_airborne_nonzero_or_wrong_component_cannot_pass(self):
        for changes in ({"armed": True}, {"landed": 2}, {"output": .2},
                        {"output": float("nan")}, {"component": 42}):
            with self.subTest(changes=changes), patch.object(campaign.time, "monotonic", Clock().now):
                with self.assertRaisesRegex(RuntimeError, "evidence missing"):
                    campaign.ground_evidence(master(**changes), timeout_s=3.)


if __name__ == "__main__":
    unittest.main()
