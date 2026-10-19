"""Loopback drawing/interview studio with an optional explicit local SITL backend.

Run ``python -m mission_planner.editor`` and open http://127.0.0.1:8765.
Each preview is compiled by a time-bounded worker using the existing verifier.
Outputs remain SITL_ONLY. Downloaded files are not simulation evidence.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import csv
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

from .contracts import EnvironmentSnapshot, MissionIntent, VehicleModel, load_contract
from .plan_drawn_path import compile_drawn_path
from .plan_interview import intent_from_interview
from .aircraft import assess_profile, catalog, import_px4_parameters, mission_budget
from .contracts import canonical_sha256


ROOT = Path(__file__).resolve().parents[1]
MAX_REQUEST_BYTES = 65_536
MAX_RESPONSE_BYTES = 24 * 1024 * 1024
MAX_INPUT_POINTS = 256
WORKER_TIMEOUT_S = 120


def preview(payload: dict) -> dict:
    """Compile one explicit document without importing the flight runner."""
    if (not isinstance(payload, dict) or not {"source", "document"} <= set(payload)
            or set(payload) - {"source", "document", "aircraft_profile"}):
        raise ValueError("preview requires source, document and optionally aircraft_profile")
    source = payload["source"]
    raw = payload["document"]
    if source == "drawing":
        intent = deepcopy(raw)
        loops = 1
    elif source == "interview":
        intent, loops = intent_from_interview(raw)
    else:
        raise ValueError("source must be drawing or interview")
    setup, error_budget = None, None
    if "aircraft_profile" in payload:
        setup = assess_profile(payload["aircraft_profile"])
        if not setup["ready"]:
            return {"accepted": False, "release_class": "SITL_ONLY", "campaign_ran": False,
                    "error": "Complete the aircraft setup questions and resolve compatibility errors.",
                    "aircraft_setup": setup}
        profile = setup["profile"]
        total = _intent_deviation(intent)
        geometric = total - profile["position_uncertainty_m"] - profile["tracking_error_m"]
        if geometric <= 0:
            raise ValueError("path tolerance is consumed by the position and tracking allowances; increase tolerance or improve navigation/tracking")
        error_budget = dict(total_tolerance_m=total, planning_tolerance_m=geometric,
                            position_uncertainty_m=profile["position_uncertainty_m"],
                            tracking_error_m=profile["tracking_error_m"])
        intent["path"]["maximum_deviation_m"] = geometric
        if any(p["down_m"] != intent["path"]["points"][0]["down_m"] for p in intent["path"]["points"]):
            cap = min(profile["maximum_climb_m_s"], profile["maximum_descent_m_s"])
            intent["motion"]["maximum_speed_m_s"] = min(intent["motion"]["maximum_speed_m_s"], cap)
            intent["motion"]["target_speed_m_s"] = min(intent["motion"]["target_speed_m_s"], cap)
            error_budget["vertical_route_speed_cap_m_s"] = cap
    contract = MissionIntent.from_dict(intent)
    if len(contract.path.points) > MAX_INPUT_POINTS:
        raise ValueError(f"preview accepts at most {MAX_INPUT_POINTS} input points")
    altitude = -contract.path.points[0].down_m
    if not math.isfinite(altitude) or altitude <= 0.:
        raise ValueError("preview requires a positive launch-relative cruise height")
    with tempfile.TemporaryDirectory(prefix="px4-preview-") as directory:
        work = Path(directory)
        vehicle_path = ROOT / "config/planner/x500_sitl_vehicle_model.json"
        environment_path = ROOT / "config/planner/empty_world_environment.json"
        if setup is not None:
            vehicle_path, environment_path = work / "vehicle.json", work / "environment.json"
            vehicle_path.write_text(json.dumps(setup["vehicle_model"], allow_nan=False))
            environment = load_contract(ROOT / "config/planner/empty_world_environment.json", dict)
            environment.update(environment_id="profile-env-" + setup["profile_sha256"][:16],
                               max_radius_m=profile["max_radius_m"],
                               max_height_above_launch_m=profile["max_height_m"],
                               max_wind_m_s=profile["expected_wind_m_s"])
            environment_path.write_text(json.dumps(environment, allow_nan=False))
        intent_path = work / "intent.json"
        intent_path.write_text(json.dumps(intent, allow_nan=False), encoding="utf-8")
        trajectory = work / "trajectory.csv"
        report = work / "verification.json"
        summary = compile_drawn_path(
            intent_path=intent_path,
            vehicle_path=vehicle_path,
            environment_path=environment_path,
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
        budget = None
        if setup is not None:
            if any(abs(float(r["vz"])) > min(profile["maximum_climb_m_s"], profile["maximum_descent_m_s"]) + 1e-8 for r in rows):
                raise ValueError("vertical route speed exceeds the aircraft climb/descent envelope")
            budget = mission_budget(setup, summary["retiming"]["selected_duration_s"],
                                    altitude, -float(rows[-1]["z"]))
            summary.update(aircraft_profile_sha256=setup["profile_sha256"],
                           mission_budget_sha256=canonical_sha256(budget),
                           path_error_budget_sha256=canonical_sha256(error_budget))
        stride = max(1, math.ceil((len(rows) - 1) / 1000))
        indices = sorted({*range(0, len(rows), stride), len(rows) - 1})
        return {
            "accepted": True,
            "release_class": "SITL_ONLY",
            "campaign_ran": False,
            "simulation_status": "NOT_RUN",
            "flight_approval": False,
            "intent": contract.to_dict(),
            "vehicle_model": load_contract(vehicle_path, VehicleModel.from_dict).to_dict(),
            "environment": load_contract(environment_path, EnvironmentSnapshot.from_dict).to_dict(),
            "aircraft_setup": setup,
            "mission_budget": budget,
            "path_error_budget": error_budget,
            "simulation_compatible": setup is None,
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


def _intent_deviation(intent):
    # Parse before arithmetic so strings, bools and non-finite input are refused.
    return MissionIntent.from_dict(intent).path.maximum_deviation_m


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
        if self.path not in ("/api/preview", "/api/rehearsal", "/api/aircraft/assess", "/api/aircraft/parameters"):
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
            if self.path == "/api/rehearsal":
                if getattr(self.server, "jobs", None) is None:
                    self._json(409, {"error": "local SITL backend is not configured"})
                    return
                # Apply the same strict JSON loader to authority-bearing input.
                with tempfile.TemporaryDirectory() as directory:
                    request = Path(directory) / "request.json"
                    request.write_bytes(body)
                    payload = load_contract(request, dict)
                self._json(202, self.server.jobs.start(payload))
                return
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
        except (ValueError, OSError, KeyError) as exc:
            result = {"accepted": False, "release_class": "SITL_ONLY",
                      "campaign_ran": False, "error": str(exc)}
        print(json.dumps(result, allow_nan=False))
        return
    with HTTPServer(("127.0.0.1", args.port), PreviewHandler) as server:
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
