"""Data-quality gates applied before any flight is used for fitting or scoring.

Every rejection is recorded with a reason so the scored set is auditable.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .nanobench import Flight, load_flight


@dataclass(frozen=True)
class Gate:
    arena_half_width_m: float = 3.0  # |x|,|y| limit for setpoint and truth
    max_altitude_m: float = 3.0
    max_setpoint_step_m: float = (
        1.5  # commanded steps to a trajectory start are ~1 m; larger is corruption
    )
    max_repaired_samples: int = (
        10  # more one-sample spikes than this means the log is not trustworthy
    )
    min_duration_s: float = 3.0


def content_hash(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


DEFAULT_GATE = Gate()


def check(flight: Flight, gate: Gate = DEFAULT_GATE) -> list[str]:
    reasons = []
    for label, xyz in (("setpoint", flight.setpoint), ("truth", flight.position)):
        if not np.all(np.isfinite(xyz)):
            reasons.append(f"{label} has non-finite values")
            continue
        if np.max(np.abs(xyz[:, :2])) > gate.arena_half_width_m:
            reasons.append(
                f"{label} leaves arena: max |xy| = {np.max(np.abs(xyz[:, :2])):.2f} m"
            )
        if np.max(xyz[:, 2]) > gate.max_altitude_m or np.min(xyz[:, 2]) < -0.2:
            reasons.append(
                f"{label} altitude out of range [{np.min(xyz[:, 2]):.2f}, {np.max(xyz[:, 2]):.2f}] m"
            )
    step = (
        np.max(np.linalg.norm(np.diff(flight.setpoint, axis=0), axis=1))
        if len(flight.t) > 1
        else 0.0
    )
    if step > gate.max_setpoint_step_m:
        reasons.append(f"setpoint jumps {step:.2f} m in one sample")
    if flight.repaired_samples > gate.max_repaired_samples:
        reasons.append(
            f"{flight.repaired_samples} setpoint spikes repaired (limit {gate.max_repaired_samples})"
        )
    if flight.duration_s < gate.min_duration_s:
        reasons.append(f"active segment only {flight.duration_s:.1f} s")
    return reasons


def load_clean(dataset_dir: Path, gate: Gate = DEFAULT_GATE):
    """Return (accepted flights, rejection log). Duplicates keep the first file by name."""
    accepted, log, seen = [], [], {}
    for path in sorted(Path(dataset_dir).glob("*.csv")):
        digest = content_hash(path)
        if digest in seen:
            log.append(
                {
                    "file": path.name,
                    "reason": f"byte-identical duplicate of {seen[digest]}",
                }
            )
            continue
        seen[digest] = path.name
        try:
            flight = load_flight(path)
        except ValueError as exc:
            log.append({"file": path.name, "reason": str(exc)})
            continue
        reasons = check(flight, gate)
        if reasons:
            log.append({"file": path.name, "reason": "; ".join(reasons)})
            continue
        if flight.repaired_samples:
            log.append(
                {
                    "file": path.name,
                    "accepted": True,
                    "reason": f"{flight.repaired_samples} one-sample setpoint spike(s) repaired",
                }
            )
        accepted.append(flight)
    return accepted, log
