"""Public PX4 Flight Review logs: index filtering, download, and ULog extraction.

Truth caveat: Flight Review logs carry only the vehicle's own GNSS-aided EKF2 estimate.
Everything scored against them is "relative to the onboard estimate", never "relative to truth".
"""

from __future__ import annotations

import gzip
import hashlib
import json
import urllib.request
from collections import defaultdict
from pathlib import Path

import numpy as np

INDEX_URL = "https://cdn.logs.px4.io/dbinfo.json"
X500_AIRFRAME = "Holybro X500 V2"
NAV_OFFBOARD = 14
NAV_MISSION = 3


def fetch_index(dest: Path) -> Path:
    """Download the compressed Flight Review index (kept gzip-compressed on disk)."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(INDEX_URL, timeout=600) as response:
        dest.write_bytes(response.read())
    return dest


def load_index(path: Path) -> list[dict]:
    raw = Path(path).read_bytes()
    if raw[:2] == b"\x1f\x8b":
        raw = gzip.decompress(raw)
    return json.loads(raw)


def mode_seconds(entry: dict, nav_state: int) -> float:
    for key, value in entry.get("flight_mode_durations") or []:
        if key == nav_state:
            return float(value)
    return 0.0


def select_candidates(
    entries,
    airframe=X500_AIRFRAME,
    min_offboard_s=30.0,
    per_vehicle=3,
    min_version=(1, 14),
):
    """Real-hardware X500 V2 logs with enough Offboard time, capped per vehicle for diversity."""

    def version_ok(entry):
        text = str(entry.get("ver_sw_release") or "")
        if not text.startswith("v1."):
            return False
        try:
            major, minor = (int(p) for p in text[1:].split(".")[:2])
        except ValueError:
            return False
        return (major, minor) >= min_version

    pool = [
        e
        for e in entries
        if e.get("airframe_name") == airframe
        and "SITL" not in str(e.get("sys_hw", ""))
        and version_ok(e)
        and mode_seconds(e, NAV_OFFBOARD) >= min_offboard_s
        and e.get("download_url")
    ]
    pool.sort(key=lambda e: -mode_seconds(e, NAV_OFFBOARD))
    used = defaultdict(int)
    picked = []
    for entry in pool:
        vehicle = entry.get("vehicle_uuid") or entry["log_id"]
        if used[vehicle] >= per_vehicle:
            continue
        used[vehicle] += 1
        picked.append(
            {
                "log_id": entry["log_id"],
                "vehicle": vehicle,
                "version": entry.get("ver_sw_release"),
                "hardware": entry.get("sys_hw"),
                "offboard_s": mode_seconds(entry, NAV_OFFBOARD),
                "duration_s": entry.get("duration_s"),
                "download_url": entry["download_url"],
            }
        )
    return picked


def download(candidate: dict, out_dir: Path, timeout: float = 300.0) -> Path | None:
    out = Path(out_dir) / f"{candidate['log_id']}.ulg"
    if out.exists() and out.stat().st_size > 0:
        return out
    url = candidate["download_url"]
    if url.startswith("/"):
        url = "https://review.px4.io" + url
    tmp = out.with_suffix(".part")
    try:
        with (
            urllib.request.urlopen(url, timeout=timeout) as response,
            open(tmp, "wb") as fh,
        ):
            while chunk := response.read(1 << 20):
                fh.write(chunk)
    except Exception:  # noqa: BLE001 - public mirror is best-effort; failures are logged by caller
        tmp.unlink(missing_ok=True)
        return None
    if tmp.read_bytes()[:7] != b"ULog\x01\x12\x35":
        tmp.unlink(missing_ok=True)
        return None
    tmp.rename(out)
    return out


TOPICS = [
    "vehicle_local_position",
    "trajectory_setpoint",
    "vehicle_status",
    "esc_status",
    "actuator_motors",
    "battery_status",
    "vehicle_attitude",
    "hover_thrust_estimate",
    "vehicle_land_detected",
]


def _field(dataset, name):
    return np.asarray(dataset.data[name], dtype=float) if name in dataset.data else None


def extract(ulg_path: Path) -> dict:
    """Extract the signals needed for identification and replay into plain arrays (seconds, NED metres)."""
    from pyulog import ULog

    ulog = ULog(str(ulg_path), TOPICS)
    data = {}
    for d in ulog.data_list:
        if d.multi_id == 0 and d.name not in data:
            data[d.name] = d
    out: dict = {
        "params": {
            k: float(v)
            for k, v in ulog.initial_parameters.items()
            if isinstance(v, (int, float))
        }
    }
    t0 = ulog.start_timestamp

    def time(d):
        return (np.asarray(d.data["timestamp"], dtype=float) - t0) * 1e-6

    lp = data.get("vehicle_local_position")
    if lp is None:
        raise ValueError("no vehicle_local_position")
    out["lpos_t"] = time(lp)
    out["lpos"] = np.column_stack(
        [_field(lp, k) for k in ("x", "y", "z", "vx", "vy", "vz")]
    )
    if (valid := _field(lp, "xy_valid")) is not None:
        out["lpos_xy_valid"] = valid
    sp = data.get("trajectory_setpoint")
    if sp is not None:
        cols = []
        for base in ("position", "velocity", "acceleration"):
            for i in range(3):
                v = _field(sp, f"{base}[{i}]")
                cols.append(
                    v if v is not None else np.full(len(sp.data["timestamp"]), np.nan)
                )
        out["sp_t"], out["sp"] = time(sp), np.column_stack(cols)
    st = data.get("vehicle_status")
    if st is not None:
        out["nav_t"], out["nav"] = time(st), _field(st, "nav_state")
        if (armed := _field(st, "arming_state")) is not None:
            out["arming"] = armed
    esc = data.get("esc_status")
    if esc is not None:
        rpm = [_field(esc, f"esc[{i}].esc_rpm") for i in range(8)]
        rpm = [r for r in rpm if r is not None]
        if rpm:
            out["esc_t"], out["esc_rpm"] = time(esc), np.column_stack(rpm)
    am = data.get("actuator_motors")
    if am is not None:
        ctrl = [_field(am, f"control[{i}]") for i in range(4)]
        if all(c is not None for c in ctrl):
            out["mot_t"], out["mot"] = time(am), np.column_stack(ctrl)
    bat = data.get("battery_status")
    if bat is not None:
        out["bat_t"] = time(bat)
        out["bat"] = np.column_stack(
            [_field(bat, "voltage_v"), _field(bat, "current_a")]
        )
    att = data.get("vehicle_attitude")
    if att is not None:
        out["att_t"] = time(att)
        out["att_q"] = np.column_stack([_field(att, f"q[{i}]") for i in range(4)])
    hte = data.get("hover_thrust_estimate")
    if hte is not None and (h := _field(hte, "hover_thrust")) is not None:
        out["hover_t"], out["hover"] = time(hte), h
    return out


def segments(
    nav_t: np.ndarray,
    nav: np.ndarray,
    end_t: float,
    state: int = NAV_OFFBOARD,
    min_s: float = 10.0,
):
    """(start, stop) times of continuous spans in the requested navigation state."""
    spans, start = [], None
    for i, value in enumerate(nav):
        if value == state and start is None:
            start = nav_t[i]
        elif value != state and start is not None:
            spans.append((start, nav_t[i]))
            start = None
    if start is not None:
        spans.append((start, end_t))
    return [(a, b) for a, b in spans if b - a >= min_s]


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()
