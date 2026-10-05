"""Actual subprocess regressions for inherited-output and timeout hangs."""

import os
from pathlib import Path
import signal
import sys
import time
import unittest

from px4_connector_server import _run_command, OUTPUT_LIMIT


class CommandRunnerTests(unittest.TestCase):
    def test_input_is_closed_and_output_and_status_are_preserved(self):
        result = _run_command([
            sys.executable, "-c",
            "import sys;print(repr(sys.stdin.read()));"
            "print('failure detail',file=sys.stderr);sys.exit(7)",
        ], timeout_seconds=2)
        self.assertEqual(result["exit_code"], 7)
        self.assertEqual(result["stdout"].strip(), "''")
        self.assertIn("failure detail", result["stderr"])

    def test_child_inheriting_output_does_not_delay_parent_result(self):
        start = time.monotonic()
        result = _run_command([
            sys.executable, "-u", "-c",
            "import subprocess,sys;"
            "p=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)']);"
            "print(p.pid,flush=True)",
        ], timeout_seconds=2)
        child_pid = int(result["stdout"].strip())
        try:
            self.assertEqual(result["exit_code"], 0)
            self.assertLess(time.monotonic() - start, 2)
        finally:
            try:
                os.kill(child_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def test_timeout_retains_partial_output_without_pipe_drain(self):
        start = time.monotonic()
        result = _run_command([
            sys.executable, "-u", "-c",
            "import subprocess,sys,time;"
            "subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)']);"
            "print('partial evidence',flush=True);time.sleep(30)",
        ], timeout_seconds=1)
        self.assertEqual(result["exit_code"], -2)
        self.assertIn("partial evidence", result["stdout"])
        self.assertIn("timed out after 1 seconds", result["stderr"])
        self.assertLess(time.monotonic() - start, 8)

    def test_large_output_returns_bounded_tail(self):
        result = _run_command([
            sys.executable, "-c",
            f"print('x'*{OUTPUT_LIMIT + 1000}+'TAIL')",
        ], timeout_seconds=2)
        self.assertEqual(result["exit_code"], 0)
        self.assertEqual(len(result["stdout"]), OUTPUT_LIMIT)
        self.assertTrue(result["stdout"].rstrip().endswith("TAIL"))

    def test_launch_failure_returns_an_error(self):
        missing = str(Path(__file__).parent / "nonexistent_executable")
        result = _run_command([missing], timeout_seconds=1)
        self.assertEqual(result["exit_code"], -3)
        self.assertTrue(result["stderr"])


if __name__ == "__main__":
    unittest.main()
