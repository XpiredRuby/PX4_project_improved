"""A long compiler must not monopolize simulator status and cancellation."""
from concurrent.futures import ThreadPoolExecutor
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch

from mission_planner.editor import PreviewHandler


class StudioResponsivenessTests(unittest.TestCase):
    def test_status_and_cancel_remain_available_while_one_plan_compiles(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), PreviewHandler)
        server.jobs = Mock()
        server.jobs.status.return_value = {"stage": "RUNNING"}
        server.jobs.cancel.return_value = {"stage": "RUNNING", "cancel_requested": True}
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        entered, release = threading.Event(), threading.Event()

        def request(method, path, payload=None):
            connection = HTTPConnection("127.0.0.1", server.server_port, timeout=3)
            try:
                connection.request(method, path, json.dumps(payload) if payload is not None else None,
                                   {"Content-Type": "application/json"})
                response = connection.getresponse()
                return response.status, json.loads(response.read())
            finally:
                connection.close()

        def compiler(*args, **kwargs):
            entered.set()
            if not release.wait(5):
                raise TimeoutError("test compiler was not released")
            return SimpleNamespace(returncode=0, stdout=b'{"accepted":false,"error":"test refusal"}')

        try:
            with patch("mission_planner.editor.subprocess.run", side_effect=compiler), ThreadPoolExecutor() as pool:
                pending = pool.submit(request, "POST", "/api/preview", {})
                self.assertTrue(entered.wait(2))
                try:
                    self.assertEqual(request("GET", "/api/rehearsal/active")[0], 200)
                    code, value = request("POST", "/api/rehearsal/cancel", {})
                    self.assertEqual(code, 202)
                    self.assertTrue(value["cancel_requested"])
                    code, value = request("POST", "/api/preview", {})
                    self.assertEqual(code, 409)
                    self.assertIn("compiling", value["error"])
                finally:
                    release.set()
                self.assertEqual(pending.result()[0], 422)
                # Releasing the slot after a refused plan permits the next one.
                self.assertEqual(request("POST", "/api/preview", {})[0], 422)
        finally:
            release.set()
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
