"""Evaluate a first-order position-response baseline on whole NanoBench flights."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

REVISION = "934a1ab92c458cad99c9278a5cb63bf68af6e56c"
TRAIN = ("B2_circle_medium_rep1", "B2_circle_medium_rep2")
TEST = ("B3_figure8_medium_rep1", "B10_lissajous_slow_rep1", "B10_lissajous_fast_rep1")


def read_flight(folder, name):
    path = folder / (name + ".csv")
    data = pd.read_csv(path)
    time = data["t"].to_numpy()
    measured = data[["px", "py", "pz"]].to_numpy()
    command = data[["sp_ctrltarget_x", "sp_ctrltarget_y", "sp_ctrltarget_z"]].to_numpy()
    if (len(time) < 2 or not np.isfinite(np.column_stack([time, measured, command])).all()
            or np.any(np.diff(time) <= 0) or np.diff(time).max() > .25):
        raise ValueError(f"{name} has missing samples or discontinuous timestamps")
    return time, measured, command, hashlib.sha256(path.read_bytes()).hexdigest()


def simulate(flight, tau):
    time, measured, reference, _ = flight
    predicted = np.empty_like(measured)
    predicted[0] = measured[0]
    gains = -np.expm1(-np.diff(time) / tau)
    for i, gain in enumerate(gains):
        predicted[i + 1] = predicted[i] + gain * (reference[i] - predicted[i])
    return predicted


def errors(residual):
    distance = np.linalg.norm(residual, axis=1)
    return {"rmse_m": float(np.sqrt(np.mean(distance ** 2))),
            "p95_m": float(np.quantile(distance, .95, method="inverted_cdf")),
            "max_m": float(distance.max()), "bias_xyz_m": residual.mean(axis=0).tolist()}


def evaluate(folder):
    # Select one time constant by equally weighting the training flights.
    training = [read_flight(folder, name) for name in TRAIN]
    candidates = np.geomspace(.02, 2, 41)
    def loss(tau):
        return np.mean([np.mean(np.sum((simulate(f, tau) - f[1]) ** 2, axis=1)) for f in training])
    tau = float(min(candidates, key=loss))
    runs = []
    for name in (*TRAIN, *TEST):
        flight = read_flight(folder, name)
        time, measured, reference, digest = flight
        runs.append({"flight": name, "split": "train" if name in TRAIN else "held_out",
                     "sha256": digest, "samples": len(time), "duration_s": float(time[-1] - time[0]),
                     "predicted_minus_measured": errors(simulate(flight, tau) - measured),
                     "reference_minus_measured": errors(reference - measured)})
    return {"source": "https://github.com/syediu/nanobench", "revision": REVISION,
            "aircraft": "Crazyflie 2.1", "measurement": "Vicon world XYZ, metres",
            "model": "First-order closed-loop position lag; initialized once per flight",
            "tau_s": tau, "training_maneuver": "medium circle", "runs": runs}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_directory", type=Path)
    print(json.dumps(evaluate(parser.parse_args().csv_directory), indent=2, allow_nan=False))
