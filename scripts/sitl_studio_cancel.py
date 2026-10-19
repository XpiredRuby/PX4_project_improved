"""SITL-only cancellation guard for a confirmed browser rehearsal.

Cancellation is a request to the controller's existing native PX4 LAND
cleanup, never an OS kill of an airborne simulator or a physical flight API.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path

from mission_planner.acknowledgement import load_sitl_acknowledgement
from mission_planner.contracts import load_contract


class StudioCancelAuthority:
    """Internal SITL lifecycle hook; deliberately has no physical authority."""

    def __init__(self, run_folder, cancel_marker):
        self.root = Path(run_folder).resolve()
        self.marker = Path(cancel_marker).resolve()
        if (self.marker.name != "CANCEL_REQUEST.json"
                or self.marker.parent != self.root.parents[1]
                or self.root.name != "reviewed-plan"):
            raise ValueError("SITL cancellation marker must belong to the reviewed plan")
        summary_file = self.root / "plan/summary.json"
        summary = load_contract(summary_file, dict)
        if summary.get("release_class") != "SITL_ONLY":
            raise ValueError("studio cancellation is available for SITL_ONLY plans")
        self.release = {"release_class": "SITL_ONLY",
                        "expected_cruise_height_m": summary["controller_cruise_height_m"]}
        self.expected_digest = hashlib.sha256(summary_file.read_bytes()).hexdigest()
        self.validate()

    def validate(self):
        owned = load_contract(self.root / "owned_process.json", dict)
        pid = owned.get("pid")
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise RuntimeError("no identified local SITL simulator")
        command = Path(f"/proc/{pid}/cmdline").read_bytes()
        if (b"/build/px4_sitl_default/bin/px4\x00" not in command
                or str(self.root / "runtime").encode() not in command
                or os.getpgid(pid) != owned.get("pgid")):
            raise RuntimeError("studio guard needs its exact owned SITL simulator")
        summary = self.root / "plan/summary.json"
        if hashlib.sha256(summary.read_bytes()).hexdigest() != self.expected_digest:
            raise RuntimeError("confirmed studio artifact changed")
        load_sitl_acknowledgement(self.root / "plan/sitl_acknowledgement.json",
                                  summary, self.root / "plan/trajectory.csv")

    def check(self):
        if self.marker.exists():
            request = load_contract(self.marker, dict)
            if request.get("requested") is True and request.get("release_class") == "SITL_ONLY":
                raise RuntimeError("studio simulation cancellation requested; yielding to native PX4 LAND cleanup")
            raise RuntimeError("invalid studio cancellation marker; yielding to native PX4 LAND cleanup")

    def bind_controller(self, controller):
        if controller.connection_string != "udp:127.0.0.1:14540":
            raise RuntimeError("studio cancellation guard only accepts loopback SITL")
        self.validate()
        self.check()
        controller.release_guard = self.check

    def connected(self, controller, audited_parameters):
        self.validate()
        self.check()

    def before_arm(self, controller):
        self.validate()
        self.check()
