"""Retained, artifact-bound jobs for the optional local SITL studio.

Only server-created plans may be selected. Client paths, commands, endpoints
and aircraft settings are never accepted. Physical flight has no API here.
"""
from __future__ import annotations

from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import re
import secrets
import subprocess
import sys

from .acknowledgement import issue_sitl_acknowledgement


ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".pending")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


class StudioJobs:
    def __init__(self, directory: Path, autopilot: Path, *, gui=True):
        self.directory = directory.resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.autopilot = autopilot.resolve()
        if not (self.autopilot / "build/px4_sitl_default/bin/px4").is_file():
            raise ValueError("SITL studio requires an existing compiled PX4 tree")
        self.gui = gui
        self.process = None
        self.current = None
        self.log = None

    def folder(self, identifier):
        if not isinstance(identifier, str) or not re.fullmatch(r"[0-9a-f]{32}", identifier):
            raise ValueError("invalid plan identifier")
        folder = self.directory / identifier
        if not folder.is_dir():
            raise ValueError("plan is unavailable; plan and verify again")
        return folder

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
        write_json(folder / "job.json", {"stage": "READY", "plan_id": identifier,
                                        "controller_trajectory_sha256": digest})
        return identifier

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
        status.update(release_class="SITL_ONLY", flight_approval=False)
        return status

    def start(self, payload):
        if (not isinstance(payload, dict)
                or set(payload) != {"plan_id", "controller_trajectory_sha256", "confirm_interpretation"}
                or payload["confirm_interpretation"] is not True):
            raise ValueError("confirm the exact displayed plan before its SITL rehearsal")
        identifier = payload["plan_id"]
        folder = self.folder(identifier)
        summary = json.loads((folder / "summary.json").read_text())
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
