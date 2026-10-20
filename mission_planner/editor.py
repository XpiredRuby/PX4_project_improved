"""Loopback drawing/interview studio with an optional explicit local SITL backend.

Run ``python -m mission_planner.editor`` and open http://127.0.0.1:8765.
Each preview is compiled by a time-bounded worker using the existing verifier.
Outputs remain SITL_ONLY. Downloaded files are not simulation evidence.
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading

from .aircraft import assess_profile, catalog, import_px4_parameters
from .contracts import load_contract
from .preview import preview, MAX_INPUT_POINTS, MAX_RESPONSE_BYTES
from .explain import PlanningError


ROOT = Path(__file__).resolve().parents[1]
MAX_REQUEST_BYTES = 65_536
WORKER_TIMEOUT_S = 120
PREVIEW_SLOT = threading.BoundedSemaphore(1)


class PreviewHandler(BaseHTTPRequestHandler):
    server_version = "PX4OfflinePreview/1"

    def _reply(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy",
                         "default-src 'self'; script-src 'self' 'unsafe-inline'; "
                         "style-src 'self' 'unsafe-inline'; connect-src 'self'; "
                         "object-src 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: int, value: dict) -> None:
        self._reply(status, json.dumps(value, allow_nan=False).encode(),
                    "application/json; charset=utf-8")

    def _local_request(self) -> bool:
        port = self.server.server_port
        allowed = {f"127.0.0.1:{port}", f"localhost:{port}"}
        host = self.headers.get("Host", "")
        origin = self.headers.get("Origin")
        return host in allowed and origin in (None, f"http://{host}")

    def do_GET(self):
        if not self._local_request():
            self._json(403, {"error": "local origin required"})
        elif self.path == "/":
            self._reply(200, (ROOT / "mission_planner/editor.html").read_bytes(),
                        "text/html; charset=utf-8")
        elif self.path == "/draft.js":
            self._reply(200, (ROOT / "mission_planner/draft.js").read_bytes(),
                        "text/javascript; charset=utf-8")
        elif self.path == "/api/editor-settings":
            environment = load_contract(ROOT / "config/planner/empty_world_environment.json", dict)
            self._json(200, {"max_input_points": MAX_INPUT_POINTS,
                             "max_radius_m": environment["max_radius_m"],
                             "max_height_m": environment["max_height_above_launch_m"],
                             "connected_heading_modes": ["fixed"]})
        elif self.path == "/api/capabilities":
            self._json(200, {"sitl_enabled": getattr(self.server, "jobs", None) is not None,
                             "hardware_enabled": False})
        elif self.path == "/api/aircraft/catalog":
            self._json(200, catalog())
        elif self.path in {"/api/aircraft/example/" + name for name in ("quad", "hex", "octo", "x500")}:
            self._json(200, load_contract(ROOT / "config/planner/aircraft" / (self.path.rsplit("/", 1)[-1] + ".json"), dict))
        elif self.path == "/api/aircraft/binding":
            jobs = getattr(self.server, "jobs", None)
            binding = getattr(jobs, "binding", None)
            self._json(200, {"profile": binding["profile"] if binding else None})
        elif self.path.startswith("/api/rehearsal/") and getattr(self.server, "jobs", None) is not None:
            try:
                self._json(200, self.server.jobs.status(self.path.removeprefix("/api/rehearsal/")))
            except (ValueError, OSError) as exc:
                self._json(404, {"error": str(exc)})
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        if not self._local_request():
            self._json(403, {"error": "local origin required"})
            return
        if self.path not in ("/api/preview", "/api/rehearsal", "/api/rehearsal/cancel", "/api/aircraft/assess", "/api/aircraft/parameters"):
            self._json(404, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= MAX_REQUEST_BYTES:
                raise ValueError("request must contain 1–65536 bytes")
            if self.headers.get_content_type() != "application/json":
                raise ValueError("application/json required")
            self.connection.settimeout(10)
            body = self.rfile.read(length)
            if len(body) != length:
                raise ValueError("incomplete request")
            if self.path.startswith("/api/aircraft/"):
                with tempfile.TemporaryDirectory() as directory:
                    request = Path(directory) / "request.json"
                    request.write_bytes(body)
                    payload = load_contract(request, dict)
                value = (assess_profile(payload) if self.path.endswith("/assess")
                         else import_px4_parameters(payload.get("text")))
                self._json(200, value)
                return
            if self.path in ("/api/rehearsal", "/api/rehearsal/cancel"):
                if getattr(self.server, "jobs", None) is None:
                    self._json(409, {"error": "local SITL backend is not configured"})
                    return
                # Apply the same strict JSON loader to authority-bearing input.
                with tempfile.TemporaryDirectory() as directory:
                    request = Path(directory) / "request.json"
                    request.write_bytes(body)
                    payload = load_contract(request, dict)
                result = (self.server.jobs.start(payload) if self.path == "/api/rehearsal"
                          else self.server.jobs.cancel(payload))
                self._json(202, result)
                return
            with tempfile.TemporaryDirectory(prefix="px4-preview-request-") as directory:
                request = Path(directory) / "request.json"
                request.write_bytes(body)
                # Planning must not block status polling or a cancellation.
                # A second expensive compiler is refused rather than queued.
                if not PREVIEW_SLOT.acquire(blocking=False):
                    self._json(409, {"accepted": False,
                                     "error": "Another plan is compiling; wait for it and retry."})
                    return
                try:
                    result = subprocess.run(
                        [sys.executable, "-m", "mission_planner.editor",
                         "--compile-request", str(request)],
                        cwd=ROOT, capture_output=True, timeout=WORKER_TIMEOUT_S,
                        check=False,
                    )
                finally:
                    PREVIEW_SLOT.release()
            if result.returncode:
                raise ValueError("preview worker failed; no accepted output")
            if len(result.stdout) > MAX_RESPONSE_BYTES:
                raise ValueError("preview exceeds response budget; use the CLI")
            value = json.loads(result.stdout)
            if value.get("accepted") and getattr(self.server, "jobs", None) is not None:
                value["simulation_compatible"] = self.server.jobs.compatible(value)
                value["plan_id"] = self.server.jobs.retain(value)
            self._json(200 if value.get("accepted") else 422, value)
        except subprocess.TimeoutExpired:
            self._json(422, {"accepted": False, "error": "preview exceeded 120 s; use fewer points or the CLI"})
        except (ValueError, OSError) as exc:
            self._json(400, {"accepted": False, "error": str(exc)})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--sitl-autopilot", type=Path, help="enable isolated local PX4/Gazebo rehearsals")
    parser.add_argument("--runs", type=Path, help="retained studio plans and simulation evidence")
    parser.add_argument("--headless", action="store_true", help="disable the optional Gazebo GUI")
    parser.add_argument("--sim-binding", type=Path, help="trusted local profile-to-simulator registration")
    parser.add_argument("--compile-request", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.compile_request is not None:
        try:
            result = preview(load_contract(args.compile_request, lambda value: value))
        except PlanningError as exc:
            result = {**exc.to_dict(), "campaign_ran": False, "error": str(exc)}
        except (ValueError, OSError, KeyError) as exc:
            result = {"accepted": False, "release_class": "SITL_ONLY",
                      "campaign_ran": False, "error": str(exc)}
        print(json.dumps(result, allow_nan=False))
        return
    with ThreadingHTTPServer(("127.0.0.1", args.port), PreviewHandler) as server:
        if args.sitl_autopilot:
            if args.runs is None:
                parser.error("--sitl-autopilot requires --runs")
            from .studio_jobs import StudioJobs
            binding = load_contract(args.sim_binding, dict) if args.sim_binding else None
            server.jobs = StudioJobs(args.runs, args.sitl_autopilot, gui=not args.headless, binding=binding)
        elif args.sim_binding:
            parser.error("--sim-binding requires --sitl-autopilot")
        print(f"Offline planner: http://127.0.0.1:{server.server_port}", flush=True)
        print("SITL_ONLY. Local rehearsals require confirmation. Hardware API is disabled.", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
