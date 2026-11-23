"""Loader for the NanoBench Crazyflie 2.1 dataset (Ullah & Baca, arXiv:2603.09908).

Truth is Vicon position/velocity (px..vz). The onboard EKF is fed Vicon pose during
collection, so it is NOT independent and is never used as truth here.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

SETPOINT = ["sp_ctrltarget_x", "sp_ctrltarget_y", "sp_ctrltarget_z"]
POSITION = ["px", "py", "pz"]
VELOCITY = ["vx", "vy", "vz"]


@dataclass(frozen=True)
class Flight:
    name: str
    trajectory_id: str
    trajectory_type: str
    repetition: int
    t: np.ndarray  # s from start of the active segment, uniform grid
    setpoint: np.ndarray  # (N, 3) m
    position: np.ndarray  # (N, 3) m, Vicon truth
    velocity: np.ndarray  # (N, 3) m/s, Vicon truth
    repaired_samples: int = (
        0  # isolated one-sample setpoint spikes replaced by neighbour mean
    )

    @property
    def dt(self) -> float:
        return float(np.median(np.diff(self.t)))

    @property
    def duration_s(self) -> float:
        return float(self.t[-1] - self.t[0])


def _read_meta(path: Path) -> dict:
    meta: dict = {}
    for line in path.read_text().splitlines():
        if line and not line.startswith(" ") and ":" in line:
            key, _, value = line.partition(":")
            meta[key.strip()] = value.strip()
    return meta


AIRBORNE_TRUTH_Z_M = 0.20
AIRBORNE_SETPOINT_Z_M = 0.30


def repair_spikes(
    setpoint: np.ndarray, threshold_m: float = 0.3
) -> tuple[np.ndarray, int]:
    """Replace isolated one-sample spikes (off-path sample whose neighbours agree)."""
    sp = setpoint.copy()
    repaired = 0
    for i in range(1, len(sp) - 1):
        out = (
            np.linalg.norm(sp[i] - sp[i - 1]) > threshold_m
            and np.linalg.norm(sp[i] - sp[i + 1]) > threshold_m
        )
        if out and np.linalg.norm(sp[i + 1] - sp[i - 1]) < threshold_m:
            sp[i] = 0.5 * (sp[i - 1] + sp[i + 1])
            repaired += 1
    return sp, repaired


def active_segment(setpoint: np.ndarray, truth_z: np.ndarray | None = None) -> slice:
    """Airborne tracking segment: commanded altitude >= 0.3 m and (if given) truth altitude >= 0.2 m.

    Excludes pre-arm idle, ground contact during takeoff, and the landing tail.
    """
    airborne = setpoint[:, 2] >= AIRBORNE_SETPOINT_Z_M
    if truth_z is not None:
        airborne &= truth_z >= AIRBORNE_TRUTH_Z_M
    active = np.flatnonzero(airborne)
    if active.size < 2:
        raise ValueError("flight has no airborne setpoint segment")
    return slice(int(active[0]), int(active[-1]) + 1)


def load_flight(csv_path: Path) -> Flight:
    csv_path = Path(csv_path)
    frame = pd.read_csv(csv_path, usecols=["t", *POSITION, *VELOCITY, *SETPOINT])
    if frame.isna().any().any():
        frame = frame.interpolate(limit_direction="both")
    setpoint, repaired = repair_spikes(frame[SETPOINT].to_numpy(float))
    seg = active_segment(setpoint, frame["pz"].to_numpy(float))
    t = frame["t"].to_numpy(float)[seg]
    meta_path = csv_path.with_name(csv_path.stem + "_metadata.yaml")
    meta = _read_meta(meta_path) if meta_path.exists() else {}
    stem = csv_path.stem
    trajectory_id = meta.get("trajectory_id", stem.rsplit("_rep", 1)[0])
    rep_text = meta.get("repetition", stem.rsplit("_rep", 1)[-1])
    return Flight(
        name=stem,
        trajectory_id=trajectory_id,
        trajectory_type=meta.get("trajectory_type", trajectory_id),
        repetition=int(rep_text) if str(rep_text).isdigit() else 0,
        t=t - t[0],
        setpoint=setpoint[seg],
        position=frame[POSITION].to_numpy(float)[seg],
        velocity=frame[VELOCITY].to_numpy(float)[seg],
        repaired_samples=repaired,
    )


def load_dataset(dataset_dir: Path, min_duration_s: float = 3.0) -> list[Flight]:
    flights = []
    for path in sorted(Path(dataset_dir).glob("*.csv")):
        try:
            flight = load_flight(path)
        except ValueError:
            continue
        if flight.duration_s >= min_duration_s:
            flights.append(flight)
    return flights
