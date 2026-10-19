"""Loopback-only drawing/interview preview. No simulator or flight commands.

Run ``python -m mission_planner.editor`` and open http://127.0.0.1:8765.
Each preview is compiled by a time-bounded worker using the existing verifier.
Outputs remain SITL_ONLY. Downloaded files are not simulation evidence.
"""

from __future__ import annotations

import argparse
import csv
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

from .contracts import MissionIntent, load_contract
from .plan_drawn_path import compile_drawn_path
from .plan_interview import intent_from_interview


ROOT = Path(__file__).resolve().parents[1]
MAX_REQUEST_BYTES = 65_536
MAX_RESPONSE_BYTES = 24 * 1024 * 1024
MAX_INPUT_POINTS = 256
WORKER_TIMEOUT_S = 120


def preview(payload: dict) -> dict:
    """Compile one explicit document without importing the flight runner."""
    if not isinstance(payload, dict) or set(payload) != {"source", "document"}:
        raise ValueError("preview requires exactly source and document")
    source = payload["source"]
    raw = payload["document"]
    if source == "drawing":
        intent = raw
        loops = 1
    elif source == "interview":
        intent, loops = intent_from_interview(raw)
    else:
        raise ValueError("source must be drawing or interview")
    contract = MissionIntent.from_dict(intent)
    if len(contract.path.points) > MAX_INPUT_POINTS:
        raise ValueError(f"preview accepts at most {MAX_INPUT_POINTS} input points")
    altitude = -contract.path.points[0].down_m
    if not math.isfinite(altitude) or altitude <= 0.:
        raise ValueError("preview requires a positive launch-relative cruise height")
    with tempfile.TemporaryDirectory(prefix="px4-preview-") as directory:
        work = Path(directory)
        intent_path = work / "intent.json"
        intent_path.write_text(json.dumps(intent, allow_nan=False), encoding="utf-8")
        trajectory = work / "trajectory.csv"
        report = work / "verification.json"
        summary = compile_drawn_path(
            intent_path=intent_path,
            vehicle_path=ROOT / "config/planner/x500_sitl_vehicle_model.json",
            environment_path=ROOT / "config/planner/empty_world_environment.json",
            trajectory_path=trajectory,
            report_path=report,
            summary_path=work / "summary.json",
            control_spacing_m=.5,
            verification_resolution_m=.02,
            sample_period_s=.05,
            limit_scale=.9,
            controller_cruise_height_m=altitude,
            deviation_utilization=.75,
            loop_count=loops,
        )
        if trajectory.stat().st_size > MAX_RESPONSE_BYTES // 2:
            raise ValueError("trajectory exceeds the preview download budget; use the CLI")
        csv_text = trajectory.read_text(encoding="utf-8")
        rows = list(csv.DictReader(csv_text.splitlines()))
        stride = max(1, math.ceil((len(rows) - 1) / 1000))
        indices = sorted({*range(0, len(rows), stride), len(rows) - 1})
        return {
            "accepted": True,
            "release_class": "SITL_ONLY",
            "campaign_ran": False,
            "simulation_status": "NOT_RUN",
            "flight_approval": False,
            "intent": intent,
            "summary": summary,
            "verification": json.loads(report.read_text(encoding="utf-8")),
            "samples": [
                {key: float(rows[i][key]) for key in
                 ("time", "x", "y", "z", "yaw", "vx", "vy", "vz", "ax", "ay", "az")}
                for i in indices
            ],
            "display_samples_decimated": stride > 1,
            "trajectory_csv": csv_text,
            "preview_settings": {
                "control_spacing_m": .5, "verification_resolution_m": .02,
                "sample_period_s": .05, "limit_scale": .9,
                "deviation_utilization": .75,
            },
        }


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
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        if not self._local_request():
            self._json(403, {"error": "local origin required"})
            return
        if self.path != "/api/preview":
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
            with tempfile.TemporaryDirectory(prefix="px4-preview-request-") as directory:
                request = Path(directory) / "request.json"
                request.write_bytes(body)
                result = subprocess.run(
                    [sys.executable, "-m", "mission_planner.editor",
                     "--compile-request", str(request)],
                    cwd=ROOT, capture_output=True, timeout=WORKER_TIMEOUT_S,
                    check=False,
                )
            if result.returncode:
                raise ValueError("preview worker failed; no accepted output")
            if len(result.stdout) > MAX_RESPONSE_BYTES:
                raise ValueError("preview exceeds response budget; use the CLI")
            value = json.loads(result.stdout)
            self._json(200 if value.get("accepted") else 422, value)
        except subprocess.TimeoutExpired:
            self._json(422, {"accepted": False, "error": "preview exceeded 120 s; use fewer points or the CLI"})
        except (ValueError, OSError) as exc:
            self._json(400, {"accepted": False, "error": str(exc)})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--compile-request", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.compile_request is not None:
        try:
            result = preview(load_contract(args.compile_request, lambda value: value))
        except (ValueError, OSError, KeyError) as exc:
            result = {"accepted": False, "release_class": "SITL_ONLY",
                      "campaign_ran": False, "error": str(exc)}
        print(json.dumps(result, allow_nan=False))
        return
    with HTTPServer(("127.0.0.1", args.port), PreviewHandler) as server:
        print(f"Offline planner: http://127.0.0.1:{server.server_port}", flush=True)
        print("SITL_ONLY preview. Simulation has not run. No flight commands.", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
