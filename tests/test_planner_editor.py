"""Offline editor integration and command-authority boundary tests."""

from copy import deepcopy
from http.client import HTTPConnection
from http.server import HTTPServer
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

from mission_planner.editor import PreviewHandler, preview


ROOT = Path(__file__).resolve().parents[1]


class EditorTests(unittest.TestCase):
    def test_both_inputs_use_the_compiler_and_export_actual_certificate(self):
        drawn = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        drawn["path"]["points"] = [drawn["path"]["points"][0], drawn["path"]["points"][-1]]
        interview = {
            "schema_version": 1, "mission_id": "studio-test",
            "vehicle_model_id": "px4-x500-sitl-v1", "environment_id": "empty-world-v1",
            "pattern": "waypoints", "altitude_m": 17., "maximum_deviation_m": .5,
            "maximum_speed_m_s": 1.25, "target_speed_m_s": 1.,
            "time_mode": "minimize", "yaw_mode": "fixed", "fixed_yaw_deg": 0.,
            "waypoints": [{"north_m": 0., "east_m": 0.}, {"north_m": 18., "east_m": 0.}],
        }
        for source, document in (("drawing", drawn), ("interview", interview)):
            with self.subTest(source=source):
                result = preview({"source": source, "document": document})
                self.assertTrue(result["accepted"])
                self.assertTrue(result["verification"]["accepted"])
                self.assertEqual(result["release_class"], "SITL_ONLY")
                self.assertFalse(result["flight_approval"])
                self.assertFalse(result["campaign_ran"])
                self.assertEqual(result["simulation_status"], "NOT_RUN")
                self.assertAlmostEqual(result["samples"][0]["time"], 0.)
                self.assertAlmostEqual(result["samples"][-1]["x"], 18.)
                self.assertIn("planner_contract_version", result["trajectory_csv"])
                self.assertLessEqual(len(result["samples"]), 1002)

    def test_editor_does_not_accept_arbitrary_paths_or_authority_requests(self):
        for payload in (
            {"source": "drawing", "document": {}, "vehicle_path": "/tmp/model.json"},
            {"source": "hardware", "document": {}},
            {"source": "drawing", "document": {}, "arm": True},
        ):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                preview(payload)
        raw = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        raw["release_class"] = "FLIGHT_CANDIDATE"
        with self.assertRaises(ValueError):
            preview({"source": "drawing", "document": raw})

    def test_size_limit_precedes_compiler(self):
        raw = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        original = deepcopy(raw["path"]["points"][0])
        raw["path"]["points"] = [dict(original, point_id=f"p{i}", north_m=float(i))
                                  for i in range(257)]
        with self.assertRaisesRegex(ValueError, "at most 256"):
            preview({"source": "drawing", "document": raw})


class HTTPBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.server = HTTPServer(("127.0.0.1", 0), PreviewHandler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def request(self, method, path, body=None, headers=None):
        connection = HTTPConnection("127.0.0.1", self.server.server_port, timeout=5)
        try:
            connection.request(method, path, body, headers or {})
            response = connection.getresponse()
            return response.status, response.read()
        finally:
            connection.close()

    def test_page_available_but_files_and_launch_endpoints_absent(self):
        status, body = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"Gazebo has not run", body)
        self.assertIn(b"Plan and verify", body)
        for path in ("/../config/planner/x500_sitl_vehicle_model.json", "/api/arm", "/api/launch"):
            with self.subTest(path=path):
                status, _ = self.request("GET", path)
                self.assertEqual(status, 404)

    def test_cross_origin_and_rebinding_requests_cannot_run_worker(self):
        for headers in (
            {"Host": "attacker.example"},
            {"Origin": "https://attacker.example"},
        ):
            with self.subTest(headers=headers), patch("mission_planner.editor.subprocess.run") as worker:
                status, _ = self.request("POST", "/api/preview", "{}", headers)
                self.assertEqual(status, 403)
                worker.assert_not_called()

    def test_invalid_content_and_oversize_requests_fail_before_worker(self):
        for body, headers in (
            ("{}", {"Content-Type": "text/plain"}),
            ("x" * 65537, {"Content-Type": "application/json"}),
        ):
            with self.subTest(headers=headers), patch("mission_planner.editor.subprocess.run") as worker:
                status, _ = self.request("POST", "/api/preview", body, headers)
                self.assertEqual(status, 400)
                worker.assert_not_called()

    def test_real_http_worker_returns_verified_plan_and_refuses_bad_input(self):
        document = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        document["path"]["points"] = [document["path"]["points"][0],
                                      document["path"]["points"][-1]]
        payload = json.dumps({"source": "drawing", "document": document})
        # The compiler is a real subprocess, not a stub; allow its normal cost.
        connection = HTTPConnection("127.0.0.1", self.server.server_port, timeout=125)
        self.addCleanup(connection.close)
        connection.request("POST", "/api/preview", payload,
                           {"Content-Type": "application/json"})
        response = connection.getresponse()
        result = json.loads(response.read())
        self.assertEqual(response.status, 200)
        self.assertTrue(result["accepted"])
        self.assertTrue(result["verification"]["accepted"])
        self.assertFalse(result["campaign_ran"])
        connection.request("POST", "/api/preview", '{"source":"drawing","document":{}}',
                           {"Content-Type": "application/json"})
        response = connection.getresponse()
        result = json.loads(response.read())
        self.assertEqual(response.status, 422)
        self.assertFalse(result["accepted"])


if __name__ == "__main__":
    unittest.main()
