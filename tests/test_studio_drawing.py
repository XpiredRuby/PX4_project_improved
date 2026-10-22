"""Exercise the actual browser drawing model and its served configuration."""
from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from mission_planner.editor import main
from mission_planner.explain import PlanningError


ROOT = Path(__file__).resolve().parents[1]


class DrawingLogicTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("node"), "Node is required for browser logic tests")
    def test_real_browser_draft_model(self):
        result = subprocess.run(
            [shutil.which("node"), "--test", str(ROOT / "tests/studio_drawing.test.cjs")],
            capture_output=True, text=True, timeout=20,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_worker_preserves_structured_duration_refusal(self):
        with tempfile.TemporaryDirectory() as directory:
            request = Path(directory) / "request.json"
            request.write_text("{}")
            output = StringIO()
            error = PlanningError("requested time too short", limiting_constraint="duration",
                                  minimum_feasible_duration_s=42.)
            with patch("mission_planner.editor.preview", side_effect=error), redirect_stdout(output):
                main(["--compile-request", str(request)])
            refusal = json.loads(output.getvalue())
            self.assertFalse(refusal["accepted"])
            self.assertEqual(refusal["limiting_constraint"], "duration")
            self.assertEqual(refusal["minimum_feasible_duration_s"], 42.)
            self.assertEqual(refusal["error"], "requested time too short")
