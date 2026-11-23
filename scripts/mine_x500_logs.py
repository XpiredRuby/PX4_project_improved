"""Mine public Holybro X500 V2 Offboard flights from PX4 Flight Review.

Steps (each resumable): index -> select -> download -> extract.
Usage: python scripts/mine_x500_logs.py WORK_DIR [--max-logs N] [--workers K]
Outputs in WORK_DIR: dbinfo.json.gz, candidates.json, ulg/*.ulg, npz/*.npz, manifest.json
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

MIN_FREE_BYTES = 5 * 1024**3


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("work_dir", type=Path)
    ap.add_argument("--max-logs", type=int, default=120)
    ap.add_argument("--workers", type=int, default=6)
    args = ap.parse_args()
    work = args.work_dir
    work.mkdir(parents=True, exist_ok=True)
    index = work / "dbinfo.json.gz"
    if not index.exists():
        print("fetching index", flush=True)
        px4_logs.fetch_index(index)
    cand_path = work / "candidates.json"
    if not cand_path.exists():
        entries = px4_logs.load_index(index)
        cands = px4_logs.select_candidates(entries)
        cand_path.write_text(json.dumps(cands, indent=1))
        del entries
    cands = json.loads(cand_path.read_text())[: args.max_logs]
    print(f"candidates {len(cands)}", flush=True)
    (work / "ulg").mkdir(exist_ok=True)
    (work / "npz").mkdir(exist_ok=True)

    def fetch(c):
        if shutil.disk_usage(work).free < MIN_FREE_BYTES:
            return c["log_id"], None, "disk floor reached"
        path = px4_logs.download(c, work / "ulg")
        return c["log_id"], path, None if path else "download failed or not a ULog"

    manifest = {}
    man_path = work / "manifest.json"
    if man_path.exists():
        manifest = json.loads(man_path.read_text())
    with ThreadPoolExecutor(args.workers) as pool:
        for log_id, path, err in pool.map(fetch, cands):
            entry = manifest.setdefault(log_id, {})
            meta = next(c for c in cands if c["log_id"] == log_id)
            entry.update(
                {k: meta[k] for k in ("vehicle", "version", "hardware", "offboard_s")}
            )
            if err:
                entry["status"] = err
                continue
            npz = work / "npz" / f"{log_id}.npz"
            if not npz.exists():
                try:
                    data = px4_logs.extract(path)
                except Exception as exc:  # noqa: BLE001 - user-uploaded logs vary; record and continue
                    entry["status"] = f"extract failed: {exc}"[:200]
                    continue
                params = data.pop("params")
                np.savez_compressed(npz, **data)
                (work / "npz" / f"{log_id}.params.json").write_text(json.dumps(params))
            entry["status"] = "ok"
            entry["sha256"] = entry.get("sha256") or px4_logs.file_sha256(path)
            man_path.write_text(json.dumps(manifest, indent=1))
            print(log_id[:8], "ok", flush=True)
    man_path.write_text(json.dumps(manifest, indent=1))
    ok = sum(1 for e in manifest.values() if e.get("status") == "ok")
    print(f"done ok={ok} of {len(manifest)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
