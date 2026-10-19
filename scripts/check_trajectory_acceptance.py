#!/usr/bin/env python3
"""Run the declared drawn/interview acceptance matrix without claiming SITL ran."""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission_planner.acceptance_cases import acceptance_cases  # noqa: E402
from mission_planner.editor import preview  # noqa: E402
from mission_planner.flight_release import source_digest  # noqa: E402


def audit_export(result):
    """Independently inspect full execution samples, not decimated display plots."""
    rows = list(csv.DictReader(io.StringIO(result["trajectory_csv"])))
    values = {k: np.array([float(row[k]) for row in rows]) for k in
              ("time", "x", "y", "z", "yaw", "vx", "vy", "vz", "ax", "ay", "az", "jx", "jy", "jz")}
    assert all(np.isfinite(v).all() for v in values.values()), "nonfinite execution samples"
    assert values["time"][0] == 0 and np.all(np.diff(values["time"]) > 0), "invalid sample clock"
    for key in ("vx", "vy", "vz", "ax", "ay", "az", "jx", "jy", "jz"):
        assert np.max(np.abs(values[key][[0, -1]])) < 1e-6, "nonstationary endpoint: " + key
    speed = np.sqrt(sum(values[k]**2 for k in ("vx", "vy", "vz")))
    bound = result["summary"]["retiming"]["speed_limit_m_s"]
    assert np.max(speed) <= bound + 1e-6, "execution samples exceed the effective cap"
    digest = hashlib.sha256(result["trajectory_csv"].encode()).hexdigest()
    assert digest == result["summary"]["controller_trajectory_sha256"], "CSV binding differs"
    assert result["verification"]["accepted"] is True
    assert result["campaign_ran"] is False and result["flight_approval"] is False
    return {"samples": len(rows), "maximum_sampled_speed_m_s": float(np.max(speed)),
            "trajectory_sha256": digest, "duration_s": float(values["time"][-1]),
            "limiting_constraint": result["summary"]["retiming"]["limiting_constraint"]}


def run_matrix(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    results = []
    for name, payload, expected in acceptance_cases():
        started = time.monotonic()
        try:
            result = preview(payload)
            actual = result.get("accepted") is True
            detail = audit_export(result) if actual else {"reason": result.get("error")}
        except ValueError as exc:
            actual, detail = False, {"reason": str(exc)}
        except Exception as exc:
            actual, detail = None, {"error": f"{type(exc).__name__}: {exc}"}
        passed = actual is expected
        results.append({"case": name, "expected_accepted": expected, "accepted": actual,
                        "passed": passed, "elapsed_s": time.monotonic()-started, **detail})
        report = {"release_class": "SITL_ONLY", "campaign_ran": False,
                  "source_digest_sha256": source_digest(), "cases": results,
                  "passed": all(c["passed"] for c in results),
                  "scope": "Offline input/geometry/timing/export acceptance; Gazebo evidence is separate."}
        (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        print(f"{name}: {'PASS' if passed else 'FAIL'}", flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = run_matrix(args.output)
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
