"""Retained, artifact-bound jobs for the optional local SITL studio.

Only server-created plans may be selected. Client paths, commands, endpoints
and aircraft settings are never accepted. Physical flight has no API here.
"""
from __future__ import annotations

from datetime import datetime, timedelta, timezone
from functools import wraps
import csv
import hashlib
import json
from pathlib import Path
import re
import secrets
import subprocess
import sys
import threading

from .acknowledgement import issue_sitl_acknowledgement
from .contracts import canonical_sha256, load_contract
from .simulation_binding import validate_binding


ROOT = Path(__file__).resolve().parents[1]


def synchronized(method):
    """Serialize job ownership across concurrent local HTTP requests."""
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return call


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".pending")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def recorded_phase(run):
    """Read a complete recent telemetry row for display, never for control."""
    files = list((run / "reviewed-plan/batch/reviewed-plan").glob("research_log_*.csv"))
    if len(files) != 1:
        return None
    try:
        with files[0].open("rb") as stream:
            header = next(csv.reader([stream.readline().decode()]))
            stream.seek(0, 2)
            end = stream.tell()
            stream.seek(max(0, end-16384))
            tail = stream.read().decode(errors="replace")
        # Drop the possibly partial first and last rows. Logging may be active.
        lines = tail.split("\n")[1:-1]
        index = header.index("phase")
        for row in reversed(list(csv.reader(lines))):
            if len(row) == len(header):
                return row[index]
    except (OSError, ValueError, StopIteration, csv.Error):
        pass
    return None


class StudioJobs:
    def __init__(self, directory: Path, autopilot: Path, *, gui=True, binding=None):
        self._lock = threading.RLock()
        self.directory = directory.resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.autopilot = autopilot.resolve()
        if not (self.autopilot / "build/px4_sitl_default/bin/px4").is_file():
            raise ValueError("SITL studio requires an existing compiled PX4 tree")
        self.gui = gui
        self.binding = validate_binding(binding, self.autopilot) if binding is not None else None
        self.process = None
        self.current = None
        self.log = None

    def compatible(self, result):
        if self.binding is None:
            default = load_contract(ROOT / "config/planner/x500_sitl_vehicle_model.json", dict)
            return (result.get("aircraft_setup") is None
                    and result["summary"]["vehicle_model_sha256"] == canonical_sha256(default))
        return ((result.get("aircraft_setup") or {}).get("profile_sha256") == self.binding["profile_sha256"]
                and result["summary"]["vehicle_model_sha256"] == self.binding["vehicle_model_sha256"])

    def folder(self, identifier):
        if not isinstance(identifier, str) or not re.fullmatch(r"[0-9a-f]{32}", identifier):
            raise ValueError("invalid plan identifier")
        folder = self.directory / identifier
        if not folder.is_dir():
            raise ValueError("plan is unavailable; plan and verify again")
        return folder

    @synchronized
    def retain(self, result):
        if result.get("accepted") is not True or result.get("release_class") != "SITL_ONLY":
            raise ValueError("only an accepted SITL plan may be retained")
        csv_text = result["trajectory_csv"]
        digest = hashlib.sha256(csv_text.encode()).hexdigest()
        if digest != result["summary"]["controller_trajectory_sha256"]:
            raise ValueError("accepted trajectory digest mismatch")
        identifier = secrets.token_hex(16)
        folder = self.directory / identifier
        folder.mkdir()
        (folder / "trajectory.csv").write_bytes(csv_text.encode())
        for filename, key in (("summary.json", "summary"), ("intent.json", "intent"),
                              ("verification.json", "verification"),
                              ("vehicle_model.json", "vehicle_model"), ("environment.json", "environment")):
            write_json(folder / filename, result[key])
        if result.get("aircraft_setup") is not None:
            write_json(folder / "aircraft_profile.json", result["aircraft_setup"]["profile"])
            write_json(folder / "mission_budget.json", result["mission_budget"])
            write_json(folder / "path_error_budget.json", result["path_error_budget"])
        if self.binding is not None and self.compatible(result):
            write_json(folder / "simulation_binding.json", self.binding)
        write_json(folder / "job.json", {"stage": "READY", "plan_id": identifier,
                                        "controller_trajectory_sha256": digest})
        return identifier

    @synchronized
    def status(self, identifier):
        folder = self.folder(identifier)
        status = json.loads((folder / "job.json").read_text())
        run = folder / "rehearsal"
        if (run / "status.json").is_file():
            status.update(json.loads((run / "status.json").read_text()))
        if self.current == identifier and self.process is not None and self.process.poll() is not None:
            if status["stage"] in ("STARTING", "RUNNING"):
                status.update(stage="FAILED", error="SITL worker exited without a complete assessment")
            if self.log is not None:
                self.log.close()
                self.log = None
        assessment = run / "reviewed-plan/assessment.json"
        if assessment.is_file():
            status["assessment"] = json.loads(assessment.read_text())
        if status["stage"] == "FAILED" and (folder / "worker.log").is_file():
            status["diagnostics"] = (folder / "worker.log").read_text(errors="replace")[-6000:]
        status.update(release_class="SITL_ONLY", flight_approval=False,
                      cancel_requested=(folder / "CANCEL_REQUEST.json").is_file())
        status["phase"] = recorded_phase(run)
        return status

    @synchronized
    def cancel(self, payload):
        """Request controller-managed landing; never kill the simulator process.

        No physical endpoint is exposed. Even a failed mission must retain
        landing, disarm and propulsion evidence before a cancelled verdict.
        """
        if (not isinstance(payload, dict)
                or set(payload) != {"plan_id", "controller_trajectory_sha256"}):
            raise ValueError("SITL cancellation needs the exact active plan and trajectory digest")
        identifier = payload["plan_id"]
        folder = self.folder(identifier)
        summary = load_contract(folder / "summary.json", dict)
        if (payload["controller_trajectory_sha256"] != summary["controller_trajectory_sha256"]
                or hashlib.sha256((folder / "trajectory.csv").read_bytes()).hexdigest()
                    != summary["controller_trajectory_sha256"]):
            raise ValueError("cancellation does not match the confirmed trajectory")
        status = self.status(identifier)
        if (self.current != identifier or self.process is None
                or self.process.poll() is not None
                or status["stage"] not in ("STARTING", "RUNNING")):
            raise ValueError("this rehearsal is not running; inspect its retained assessment")
        marker = folder / "CANCEL_REQUEST.json"
        if not marker.exists():
            write_json(marker, {"release_class": "SITL_ONLY", "requested": True,
                                "plan_id": identifier,
                                "controller_trajectory_sha256": summary["controller_trajectory_sha256"],
                                "requested_at": datetime.now(timezone.utc).isoformat()})
        return self.status(identifier)

    @synchronized
    def start(self, payload):
        if (not isinstance(payload, dict)
                or set(payload) != {"plan_id", "controller_trajectory_sha256", "confirm_interpretation"}
                or payload["confirm_interpretation"] is not True):
            raise ValueError("confirm the exact displayed plan before its SITL rehearsal")
        identifier = payload["plan_id"]
        folder = self.folder(identifier)
        summary = json.loads((folder / "summary.json").read_text())
        if self.binding is None:
            default = load_contract(ROOT / "config/planner/x500_sitl_vehicle_model.json", dict)
            if ("aircraft_profile_sha256" in summary
                    or summary["vehicle_model_sha256"] != canonical_sha256(default)):
                raise ValueError("this aircraft has no matching registered Gazebo model; simulation refused")
        else:
            validate_binding(self.binding, self.autopilot)
            if (summary.get("aircraft_profile_sha256") != self.binding["profile_sha256"]
                    or summary["vehicle_model_sha256"] != self.binding["vehicle_model_sha256"]):
                raise ValueError("confirmed aircraft does not match the registered simulator")
        actual = hashlib.sha256((folder / "trajectory.csv").read_bytes()).hexdigest()
        if payload["controller_trajectory_sha256"] != actual or summary["controller_trajectory_sha256"] != actual:
            raise ValueError("confirmation does not match the retained trajectory")
        if (folder / "rehearsal").exists():
            raise ValueError("this plan already has a rehearsal; inspect its result or create a new plan")
        if self.process is not None and self.process.poll() is None:
            raise ValueError("a studio rehearsal is already running")
        intent = json.loads((folder / "intent.json").read_text())
        if intent["motion"]["yaw_mode"] != "fixed":
            raise ValueError("connected rehearsal currently requires fixed heading; tangent launch alignment is not yet qualified")
        ack = issue_sitl_acknowledgement(
            **{key: summary[key] for key in ("controller_trajectory_sha256",
                                           "candidate_trajectory_sha256", "verification_report_sha256")},
            expires_at=(datetime.now(timezone.utc) + timedelta(hours=2)).isoformat(),
            acknowledged_by="local-browser-review",
        )
        write_json(folder / "sitl_acknowledgement.json", ack)
        write_json(folder / "confirmation.json", {**payload, "confirmed_at": datetime.now(timezone.utc).isoformat()})
        arguments = [sys.executable, "-u", str(ROOT / "scripts/planner_sitl_campaign.py"),
                     "--confirm-sitl", "--autopilot", str(self.autopilot),
                     "--output", str(folder / "rehearsal"), "--reviewed-plan", str(folder),
                     "--cases", "reviewed-plan"]
        if self.gui:
            arguments.append("--gui")
        self.log = (folder / "worker.log").open("w")
        try:
            self.process = subprocess.Popen(arguments, cwd=ROOT, stdin=subprocess.DEVNULL,
                                            stdout=self.log, stderr=subprocess.STDOUT)
        except OSError:
            self.log.close()
            self.log = None
            raise
        self.current = identifier
        write_json(folder / "job.json", {"stage": "STARTING", "plan_id": identifier,
                                        "controller_trajectory_sha256": actual})
        return self.status(identifier)
