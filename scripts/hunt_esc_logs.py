"""Find public X500 V2 logs that carry ESC RPM telemetry (needed for propulsion identification).

The Flight Review index does not list logged topics, so this downloads candidates (Pixhawk 6X-class
first, where DShot RPM telemetry is common), keeps the extracted arrays only when esc_status has
usable RPM at >= 20 Hz, and deletes every other downloaded file.
Usage: python scripts/hunt_esc_logs.py WORK_DIR [--max-downloads N]
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from fidelity import px4_logs  # noqa: E402

PRIORITY_HW = ("PX4_FMU_V6X", "PX4_FMU_V6XRT", "ARK_FMU_V6X")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("work_dir", type=Path)
    ap.add_argument("--max-downloads", type=int, default=400)
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args()
    work = args.work_dir
    entries = px4_logs.load_index(work / "dbinfo.json.gz")
    done = set(json.loads((work / "manifest.json").read_text()))
    pool = [
        e
        for e in entries
        if e.get("airframe_name") == px4_logs.X500_AIRFRAME
        and str(e.get("ver_sw_release", "")).startswith(
            ("v1.15", "v1.16", "v1.17", "v1.18")
        )
        and (e.get("duration_s") or 0) >= 120
        and e["log_id"] not in done
        and e.get("download_url")
    ]
    pool.sort(
        key=lambda e: (
            str(e.get("sys_hw")) not in PRIORITY_HW,
            -(e.get("duration_s") or 0),
        )
    )
    seen_vehicle: dict[str, int] = {}
    picked = []
    for e in pool:
        v = e.get("vehicle_uuid") or e["log_id"]
        if seen_vehicle.get(v, 0) >= 4:
            continue
        seen_vehicle[v] = seen_vehicle.get(v, 0) + 1
        picked.append(
            {
                "log_id": e["log_id"],
                "vehicle": v,
                "version": e.get("ver_sw_release"),
                "hardware": e.get("sys_hw"),
                "download_url": e["download_url"],
            }
        )
    picked = picked[: args.max_downloads]
    out_dir = work / "esc"
    out_dir.mkdir(exist_ok=True)
    tmp_dir = work / "esc_tmp"
    tmp_dir.mkdir(exist_ok=True)
    found = {}

    def job(c):
        if shutil.disk_usage(work).free < 5 * 1024**3:
            return c, "disk floor"
        path = px4_logs.download(c, tmp_dir)
        if path is None:
            return c, "download failed"
        try:
            d = px4_logs.extract(path)
            ok = (
                "esc_rpm" in d
                and len(d["esc_t"]) > 100
                and 1.0 / np.median(np.diff(d["esc_t"])) >= 20
            )
            ok = ok and float(np.median(np.max(d["esc_rpm"][:, :4], axis=1))) > 1000
            if ok:
                params = d.pop("params")
                np.savez_compressed(out_dir / f"{c['log_id']}.npz", **d)
                (out_dir / f"{c['log_id']}.params.json").write_text(json.dumps(params))
            return c, "esc" if ok else "no esc rpm"
        except Exception as exc:  # noqa: BLE001
            return c, f"extract failed: {exc}"[:120]
        finally:
            path.unlink(missing_ok=True)

    with ThreadPoolExecutor(args.workers) as ex:
        for i, (c, status) in enumerate(ex.map(job, picked)):
            if status == "esc":
                found[c["log_id"]] = {
                    k: c[k] for k in ("vehicle", "version", "hardware")
                }
                (work / "esc_manifest.json").write_text(json.dumps(found, indent=1))
            if i % 20 == 0 or status == "esc":
                print(i, c["log_id"][:8], status, "found", len(found), flush=True)
    (work / "esc_manifest.json").write_text(json.dumps(found, indent=1))
    print("done found", len(found), "of", len(picked), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
