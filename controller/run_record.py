"""Machine-readable mission outcome and reproducibility records."""

from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import os
from pathlib import Path
import platform
import subprocess

from mission_state import MissionOutcome


DEPENDENCIES = ("matplotlib", "numpy", "pandas", "pymavlink")


def utc_now():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def git_commit(repository_root):
    environment_sha = os.environ.get("GITHUB_SHA")
    if environment_sha:
        return environment_sha
    try:
        result = subprocess.run(
            ["git", "-C", str(repository_root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def dependency_versions():
    versions = {}
    for name in DEPENDENCIES:
        try:
            versions[name] = version(name)
        except PackageNotFoundError:
            versions[name] = None
    return versions


def source_hashes(controller_dir):
    hashes = {}
    paths = sorted(controller_dir.glob("*.py"))
    paths.append(controller_dir / "trajectory.csv")
    for path in paths:
        hashes[path.name] = (
            hashlib.sha256(path.read_bytes()).hexdigest()
            if path.is_file()
            else None
        )
    return hashes


class RunRecord:
    """Persist a run manifest without affecting flight-control decisions."""

    def __init__(self, output_dir=None, run_id=None):
        controller_dir = Path(__file__).resolve().parent
        repository_root = controller_dir.parent
        now = datetime.now(timezone.utc)
        self.run_id = run_id or now.strftime("%Y%m%dT%H%M%S%fZ")
        self.path = Path(output_dir or Path.cwd()) / (
            f"run_manifest_{self.run_id}.json"
        )
        self.data = {
            "schema_version": 1,
            "run_id": self.run_id,
            "started_at_utc": now.isoformat().replace("+00:00", "Z"),
            "completed_at_utc": None,
            "outcome": "RUNNING",
            "reason": "",
            "cleanup_status": "not_required",
            "cleanup_error": None,
            "git_commit": git_commit(repository_root),
            "python_version": platform.python_version(),
            "dependencies": dependency_versions(),
            "source_sha256": source_hashes(controller_dir),
            "trajectory_randomized": False,
            "random_seed": None,
            "controller_config": None,
            "px4_parameters": None,
            "final_state": None,
        }
        self.write()

    def attach_context(self, controller, px4_parameters):
        self.data["controller_config"] = asdict(controller.config)
        self.data["controller_runtime"] = {
            "connection_string": controller.connection_string,
            "control_rate_hz": controller.control_rate,
            "max_horizontal_speed_m_s": controller.max_horizontal_speed,
            "max_vertical_speed_m_s": controller.max_vertical_speed,
            "setpoint_watchdog_timeout_s": (
                controller.setpoint_watchdog_timeout
            ),
        }
        self.data["px4_parameters"] = {
            name: float(value)
            for name, value in sorted(px4_parameters.items())
        }
        self.write()

    def finalize(
        self,
        outcome,
        reason,
        final_state,
        cleanup_status="not_required",
        cleanup_error=None,
    ):
        outcome = MissionOutcome(outcome)
        self.data.update(
            completed_at_utc=utc_now(),
            outcome=outcome.value,
            reason=str(reason),
            cleanup_status=str(cleanup_status),
            cleanup_error=(
                None if cleanup_error is None else str(cleanup_error)
            ),
            final_state=final_state,
        )
        self.write()

    def write(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(self.data, indent=2, sort_keys=True, allow_nan=False)
            + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, self.path)
