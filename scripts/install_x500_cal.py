"""Install the calibrated Gazebo model `x500_cal` next to PX4's stock x500 (the stock model is not modified).

Coefficients come from public real X500 V2 flights (validation/2026-11-23-x500-public-logs):
thrust coefficient matched to the fleet's median hover throttle, motor lag from ESC-RPM logs,
and rotor drag chosen on half of the replayed aircraft (calibrated_model.json).
Usage: python scripts/install_x500_cal.py --autopilot ~/PX4-Autopilot [--variant thrust=...,drag=...]
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "sitl_replay", ROOT / "scripts/sitl_replay.py"
)
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
EVIDENCE = ROOT / "validation/2026-11-23-x500-public-logs/calibrated_model.json"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--autopilot", type=Path, required=True)
    ap.add_argument("--variant", help="override the evidence-selected coefficients")
    args = ap.parse_args()
    text = args.variant or json.loads(EVIDENCE.read_text())["selected_variant"]
    models = args.autopilot.resolve() / "Tools/simulation/gz/models"
    name = replay.write_variant_model(
        args.autopilot.resolve(), models, replay.parse_variant(text)
    )
    print(f"Installed {models / name} ({text}).")
    print("Register it for Studio (SITL only):")
    print(
        f"  python -m mission_planner.simulation_binding --autopilot {args.autopilot} --model {name} --airframe-id 4001 \\"
    )
    print(
        "    --profile validation/2026-11-22-full-200m/x500-cruise-profile.json --output ~/px4-x500-cal-binding.json"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
