"""Score the closed-loop tracking model against NanoBench real flights (held-out only).

Protocol A (held-out repetitions): for each trajectory id with >= 2 repetitions, the
highest-numbered repetition is test, the rest are train.
Protocol B (held-out shapes): train on simple shapes, test on shapes never seen in training.

Usage: python scripts/score_nanobench.py <nanobench>/datasets/dataset <out_dir>
"""

from __future__ import annotations

import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from fidelity import metrics, tracking_model  # noqa: E402
from fidelity.quality import load_clean  # noqa: E402

TRAIN_SHAPES = {"circle", "oval", "figure8", "figure_eight", "linear_ramp", "multisine"}
EXCLUDE_TYPES = {"battery_drain"}  # hover only; no tracking content


def split_by_repetition(flights):
    groups = defaultdict(list)
    for f in flights:
        groups[f.trajectory_id].append(f)
    train, test = [], []
    for group in groups.values():
        group.sort(key=lambda f: f.repetition)
        if len(group) >= 2:
            train.extend(group[:-1])
            test.append(group[-1])
        else:
            train.extend(group)
    return train, test


def split_by_shape(flights):
    def is_train(f):
        key = f.trajectory_type.lower()
        return key in TRAIN_SHAPES or any(
            s in f.trajectory_id.lower()
            for s in ("circle", "oval", "figure8", "ramp", "multisine")
        )

    train = [f for f in flights if is_train(f)]
    test = [f for f in flights if not is_train(f)]
    return train, test


def evaluate(name, train, test):
    assert not {f.name for f in train} & {f.name for f in test}, "train/test leakage"
    model = tracking_model.fit(train)
    train_err = np.concatenate(
        [
            metrics.error_norm(tracking_model.simulate(model, f), f.position)
            for f in train
        ]
    )
    bound = metrics.envelope(train_err, 0.95)
    per_flight, model_err, base_err = [], [], []
    for f in test:
        e_model = metrics.error_norm(tracking_model.simulate(model, f), f.position)
        e_base = metrics.error_norm(tracking_model.plan_is_flight(f), f.position)
        model_err.append(e_model)
        base_err.append(e_base)
        per_flight.append(
            {
                "flight": f.name,
                "duration_s": round(f.duration_s, 2),
                "peak_speed_m_s": round(
                    float(np.max(np.linalg.norm(f.velocity, axis=1))), 3
                ),
                "model_rmse_m": round(metrics.summarize(e_model)["rmse_m"], 4),
                "baseline_rmse_m": round(metrics.summarize(e_base)["rmse_m"], 4),
                "coverage_at_bound": round(metrics.coverage(e_model, bound), 3),
            }
        )
    model_err = np.concatenate(model_err)
    base_err = np.concatenate(base_err)
    return {
        "protocol": name,
        "train_flights": len(train),
        "test_flights": len(test),
        "model": model.to_dict(),
        "envelope_95_from_train_m": round(bound, 4),
        "test_coverage_at_envelope": round(metrics.coverage(model_err, bound), 4),
        "test_model": {
            k: round(v, 4) if isinstance(v, float) else v
            for k, v in metrics.summarize(model_err).items()
        },
        "test_baseline_plan_is_flight": {
            k: round(v, 4) if isinstance(v, float) else v
            for k, v in metrics.summarize(base_err).items()
        },
        "per_flight": per_flight,
    }


def main(dataset_dir: str, out_dir: str) -> int:
    accepted, quality_log = load_clean(Path(dataset_dir))
    flights = [
        f
        for f in accepted
        if f.trajectory_type.lower() not in EXCLUDE_TYPES
        and "battery" not in f.trajectory_id.lower()
    ]
    if not flights:
        print("no flights loaded", file=sys.stderr)
        return 2
    results = {
        "dataset": "NanoBench (Crazyflie 2.1, Vicon truth), arXiv:2603.09908",
        "flights_loaded": len(flights),
        "total_minutes": round(sum(f.duration_s for f in flights) / 60, 2),
        "truth": "Vicon position (px,py,pz). Onboard EKF not used: it is fed Vicon pose.",
        "files_rejected": sum(1 for r in quality_log if not r.get("accepted")),
        "quality_log": quality_log,
        "protocols": [
            evaluate("A_heldout_repetitions", *split_by_repetition(flights)),
            evaluate("B_heldout_shapes", *split_by_shape(flights)),
        ],
    }
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "nanobench_fidelity.json").write_text(json.dumps(results, indent=2))
    for p in results["protocols"]:
        print(
            p["protocol"],
            "train",
            p["train_flights"],
            "test",
            p["test_flights"],
            "| model",
            p["test_model"],
            "| baseline",
            p["test_baseline_plan_is_flight"],
            "| bound",
            p["envelope_95_from_train_m"],
            "coverage",
            p["test_coverage_at_envelope"],
        )
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__)
        raise SystemExit(2)
    raise SystemExit(main(sys.argv[1], sys.argv[2]))
