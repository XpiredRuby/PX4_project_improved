#!/usr/bin/env python3
"""Rehearse GPS loss and command expiry for an exactly reviewed aircraft plan.

Requires a retained, confirmed profile-bound studio plan and local PX4/Gazebo.
All endpoints and vehicle ownership checks are inherited from the SITL harness.
An intentionally incomplete trajectory can pass its declared fault-response
checks, while its nominal trajectory result remains failed.
"""
# ruff: noqa: E402
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "scripts"), str(ROOT / "analysis")]
import planner_sitl_campaign as campaign
from planner_rehearsal import assess_case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm-sitl", action="store_true", required=True)
    parser.add_argument("--autopilot", type=Path, required=True)
    parser.add_argument("--reviewed-plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--world", default="afvl_validation")
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    if not (args.reviewed_plan / "simulation_binding.json").is_file():
        parser.error("a profile-bound, confirmed retained plan is required")
    lock = open("/tmp/px4-trajectory-studio.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            command = path.read_bytes()
        except OSError:
            continue
        if b"/build/px4_sitl_default/bin/px4\x00" in command:
            raise RuntimeError("an existing PX4 instance is preserved; wait until it closes")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    prepare = campaign.prepare_case
    results = []
    try:
        for fault in ("gps-outage-4s", "control-stall"):
            def inject(name, output, reviewed_plan, selected=fault):
                case = prepare(name, output, reviewed_plan)
                if selected == "gps-outage-4s":
                    case.update(expected="navigation_response", gps_outage_s=4.,
                                trigger_phase="TRAJECTORY", trigger_clock_s=8.)
                else:
                    case.update(expected="control_stall", control_stall_s=.4, stall_clock_s=8.)
                return case
            campaign.prepare_case = inject
            destination = args.output / fault
            destination.mkdir()
            campaign.write_json(args.output / "status.json", {"stage": "RUNNING", "fault": fault})
            options = SimpleNamespace(autopilot=args.autopilot.resolve(), output=destination,
                                      reviewed_plan=args.reviewed_plan.resolve(), world=args.world, gui=args.gui)
            campaign.run_one("reviewed-plan", options)
            assessment = assess_case(destination / "reviewed-plan")
            result = {"fault": fault, "fault_response_passed": assessment["fault_response_passed"],
                      "sampled_rehearsal_passed": assessment["sampled_rehearsal_passed"],
                      "assessment_sha256": hashlib.sha256((destination / "reviewed-plan/assessment.json").read_bytes()).hexdigest()}
            results.append(result)
            campaign.write_json(args.output / "results.json", results)
            if not result["fault_response_passed"]:
                raise RuntimeError(f"{fault} response failed; inspect retained assessment")
    except Exception as exc:
        campaign.write_json(args.output / "status.json", {"stage": "FAILED", "error": str(exc)})
        raise
    finally:
        campaign.prepare_case = prepare
        lock.close()
    campaign.write_json(args.output / "status.json", {"stage": "COMPLETED", "results": results})
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
