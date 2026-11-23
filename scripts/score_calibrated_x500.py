"""Pick and score a calibrated Gazebo x500 against real X500 V2 flights (held out by aircraft).

Inputs are sitl_replay.py outputs flown with the real aircraft's own tuning: the stock model
(--stock DIR, config log_tuning) and calibrated variants (--variant DIR ...). The drag scale is
chosen on half of the aircraft and reported on the other half.
Usage: python scripts/score_calibrated_x500.py --logs ~/validation-data/x500 --stock ~/validation-data/replay \
           --variant ~/validation-data/replay-cal-drag1 ... --out validation/2026-11-23-x500-public-logs
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from fidelity import metrics  # noqa: E402

spec = importlib.util.spec_from_file_location(
    "sitl_replay", ROOT / "scripts/sitl_replay.py"
)
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def errors(path: Path, windows: dict, config: str = "log_tuning") -> dict:
    lead = int(replay.LEAD_S * replay.RATE_HZ)
    out = {}
    for rec in map(json.loads, (path / "replay.jsonl").read_text().splitlines()):
        w = windows.get((rec["log_id"], rec["t0"]))
        if w is None or rec["config"] != config:
            continue
        sim, real = np.array(rec["sim_pos"]), np.array(w["real_pos"])
        out[(rec["log_id"], rec["t0"])] = (
            w["vehicle"],
            np.linalg.norm(
                (sim[lead:] - sim[lead]) - (real[lead:] - real[lead]), axis=1
            ),
        )
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", type=Path, required=True)
    ap.add_argument("--stock", type=Path, required=True)
    ap.add_argument("--variant", type=Path, nargs="+", required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    windows = {
        (w["log_id"], w["t0"]): w for w in replay.select_windows(args.logs, 10_000)
    }
    runs = {"stock": errors(args.stock, windows)}
    labels = {}
    for d in args.variant:
        first = json.loads((d / "replay.jsonl").read_text().splitlines()[0])
        labels[d.name] = first.get("variant", d.name)
        runs[d.name] = errors(d, windows)
    common = set.intersection(*(set(r) for r in runs.values()))
    vehicles = sorted({runs["stock"][k][0] for k in common})
    cal_vehicles = set(vehicles[::2])
    cal = [k for k in common if runs["stock"][k][0] in cal_vehicles]
    hold = [k for k in common if runs["stock"][k][0] not in cal_vehicles]

    def summary(name, keys):
        return metrics.summarize(np.concatenate([runs[name][k][1] for k in keys]))

    table = {
        name: {
            "calibration_half": summary(name, cal),
            "holdout_half": summary(name, hold),
            "all": summary(name, sorted(common)),
        }
        for name in runs
    }
    best = min(labels, key=lambda n: table[n]["calibration_half"]["p95_m"])
    result = {
        "windows": len(common),
        "vehicles": len(vehicles),
        "calibration_windows": len(cal),
        "holdout_windows": len(hold),
        "variants": labels,
        "selected_on_calibration_half": best,
        "selected_variant": labels[best],
        "table": table,
        "holdout_change_vs_stock_p95": table[best]["holdout_half"]["p95_m"]
        / table["stock"]["holdout_half"]["p95_m"]
        - 1,
        "holdout_change_vs_stock_rmse": table[best]["holdout_half"]["rmse_m"]
        / table["stock"]["holdout_half"]["rmse_m"]
        - 1,
        "note": "All runs use each real aircraft's own PX4 tuning; referent is the real aircraft's EKF2 estimate.",
    }
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "calibrated_model.json").write_text(
        json.dumps(result, indent=1, default=float) + "\n"
    )
    ev_path = ROOT / "config/fidelity/x500_evidence.json"
    if ev_path.exists():
        ev = json.loads(ev_path.read_text())
        ev["headline"] = [
            h
            for h in ev["headline"]
            if not h["label"].startswith("Calibrated x500_cal")
        ]
        st, cb = table["stock"]["holdout_half"], table[best]["holdout_half"]
        ev["headline"].append(
            {
                "label": "Calibrated x500_cal, held-out median",
                "value": f"{st['p50_m']:.2f} m → {cb['p50_m']:.2f} m",
                "note": f"p95 {st['p95_m']:.2f} → {cb['p95_m']:.2f} m; tuning matters more than physics",
            }
        )
        ev_path.write_text(json.dumps(ev, indent=1) + "\n")
    for name in runs:
        h = table[name]["holdout_half"]
        print(
            f"{name:24s} holdout rmse {h['rmse_m']:.2f} p50 {h['p50_m']:.2f} p95 {h['p95_m']:.2f}"
        )
    print(
        "selected",
        labels[best],
        "holdout p95 change",
        round(100 * result["holdout_change_vs_stock_p95"], 1),
        "%",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
