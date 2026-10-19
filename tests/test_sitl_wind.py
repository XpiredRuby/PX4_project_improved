import math
import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import sitl_wind as wind  # noqa: E402


class WindTests(unittest.TestCase):
    def test_zero_requires_actual_wind_response(self):
        self.assertFalse(wind.wind_seed_matches("", (0., 0., 0.)))
        self.assertFalse(wind.wind_seed_matches("linear_velocity {}", (0., 0., 0.)))
        self.assertTrue(wind.wind_seed_matches("linear_velocity {} enable_wind: true", (0., 0., 0.)))
        self.assertFalse(wind.wind_seed_matches("linear_velocity {x: 1} enable_wind: true", (0., 0., 0.)))

    def test_lost_publication_is_retried_and_observed(self):
        clock = [0.]
        publications = []
        def run(command, **kwargs):
            if command[1] == "topic":
                publications.append(command)
            value = 6 if len(publications) >= 2 else 0
            return SimpleNamespace(stdout=f"linear_velocity {{x: {value}}} enable_wind: true")
        def sleep(duration):
            clock[0] += duration
        with patch.object(wind.time, "monotonic", lambda: clock[0]), \
                patch.object(wind.time, "sleep", sleep), patch.object(wind.subprocess, "run", run):
            self.assertIn("x: 6", wind.set_wind("test", (6., 0., 0.), 1.))
        self.assertEqual(len(publications), 2)
        self.assertLessEqual(clock[0], 1.)

    def test_missing_service_stops_at_deadline(self):
        clock = [0.]
        def sleep(duration):
            clock[0] += duration
        with patch.object(wind.time, "monotonic", lambda: clock[0]), \
                patch.object(wind.time, "sleep", sleep), \
                patch.object(wind.subprocess, "run", side_effect=subprocess.TimeoutExpired("gz", .5)):
            with self.assertRaisesRegex(RuntimeError, "not confirmed"):
                wind.set_wind("test", (0., 0., 0.), 1.)
        self.assertLessEqual(clock[0], 1.01)
        for invalid in (0, math.nan, math.inf):
            with self.assertRaises(ValueError):
                wind.set_wind("test", (0., 0., 0.), invalid)

    def test_cli_startup_time_fits_bounded_confirmation(self):
        clock = [0.]
        def run(command, **kwargs):
            duration = .9 if command[1] == "topic" else .3
            if kwargs["timeout"] < duration:
                clock[0] += kwargs["timeout"]
                raise subprocess.TimeoutExpired("gz", kwargs["timeout"])
            clock[0] += duration
            return SimpleNamespace(stdout="linear_velocity {} enable_wind: true")
        with patch.object(wind.time, "monotonic", lambda: clock[0]), \
                patch.object(wind.subprocess, "run", run):
            self.assertIn("enable_wind: true", wind.set_wind("test", (0., 0., 0.)))
        self.assertLessEqual(clock[0], 5.)


if __name__ == "__main__":
    unittest.main()
