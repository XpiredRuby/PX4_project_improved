#!/usr/bin/env python3
"""Recompile a real browser export natively and compare reference states only."""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import io
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission_planner.preview import preview  # noqa: E402


def read(path):
    path = Path(path)
    return (
        gzip.decompress(path.read_bytes())
        if path.suffix == ".gz"
        else path.read_bytes()
    )


def compare(bundle_path, csv_path, output, tolerance=1e-5):
    bundle = json.loads(read(bundle_path))
    csv_bytes = read(csv_path)
    digest = hashlib.sha256(csv_bytes).hexdigest()
    if not bundle.get("accepted") or digest != bundle["mission"]["trajectory_sha256"]:
        raise ValueError(
            "Browser mission CSV does not match its accepted bundle digest"
        )
    document = bundle["requested_document"]
    source = "drawing" if "motion" in document else "interview"
    payload = dict(source=source, document=document, planning_mode="closest")
    if bundle.get("aircraft_setup"):
        payload["aircraft_profile"] = bundle["aircraft_setup"]["profile"]
    native = preview(payload)
    if not native.get("accepted"):
        raise ValueError("Native runtime did not accept browser request")
    browser_rows = list(csv.DictReader(io.StringIO(csv_bytes.decode())))
    native_rows = list(csv.DictReader(io.StringIO(native["mission"]["trajectory_csv"])))
    bt = np.array([float(r["time"]) for r in browser_rows])
    nt = np.array([float(r["time"]) for r in native_rows])
    if (
        not np.isfinite(bt).all()
        or np.any(np.diff(bt) <= 0.0)
        or bt[0] != 0.0
        or abs(bt[-1] - nt[-1]) > tolerance
    ):
        raise ValueError("Browser/native mission clocks disagree")
    errors = {}
    for key in (
        "x",
        "y",
        "z",
        "yaw",
        "vx",
        "vy",
        "vz",
        "yaw_rate",
        "ax",
        "ay",
        "az",
        "jx",
        "jy",
        "jz",
        "yaw_acceleration",
        "yaw_jerk",
    ):
        actual = np.array([float(r[key]) for r in browser_rows])
        reference = np.interp(bt, nt, [float(r[key]) for r in native_rows])
        if not np.isfinite(actual).all() or not np.isfinite(reference).all():
            raise ValueError("Browser/native states contain non-finite data")
        errors[key] = float(np.max(np.abs(actual - reference)))
    result = dict(
        scope="real browser Pyodide vs native planned reference; no measured flight",
        passed=all(e <= tolerance for e in errors.values()),
        browser_rows=len(browser_rows),
        native_rows=len(native_rows),
        duration_difference_s=float(abs(bt[-1] - nt[-1])),
        maximum_absolute_differences=errors,
        absolute_numerical_tolerance=tolerance,
        browser_full_csv_sha256=digest,
        native_full_csv_sha256=hashlib.sha256(
            native["mission"]["trajectory_csv"].encode()
        ).hexdigest(),
        browser_runtime="Pyodide 0.27.7",
        browser_requested_document=document,
        browser_speed=bundle["summary"]["speed_preferences"],
        native_speed=native["summary"]["speed_preferences"],
    )
    Path(output).write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    sys.exit(0 if compare(args.bundle, args.csv, args.output)["passed"] else 1)
