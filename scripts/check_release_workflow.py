#!/usr/bin/env python3
"""Run nominal release hooks and operator abort in an owned local Gazebo.

Every artifact is SITL_ONLY. No physical qualification or RC test is inferred.
"""
# ruff: noqa: E402
import argparse
import fcntl
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts'), str(ROOT / 'analysis')]
import planner_sitl_campaign as campaign
from planner_rehearsal import assess_case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm-sitl', action='store_true', required=True)
    parser.add_argument('--autopilot', type=Path, required=True)
    parser.add_argument('--reviewed-plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--world', default='afvl_validation')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--cases', nargs='+', choices=['nominal', 'operator-abort'],
                        default=['nominal', 'operator-abort'])
    args = parser.parse_args()
    lock = open('/tmp/px4-trajectory-studio.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            command = path.read_bytes()
        except OSError:
            continue
        if b'/build/px4_sitl_default/bin/px4\x00' in command:
            raise RuntimeError('Existing PX4 instance preserved; stop it normally first')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    original = campaign.prepare_case
    results = []
    try:
        for name in args.cases:
            def prepare(case_name, output, reviewed_plan, selected=name):
                case = original(case_name, output, reviewed_plan)
                case['release_probe'] = dict(folder=str(output), abort_after_s=3. if selected == 'operator-abort' else None)
                if selected == 'operator-abort':
                    case['expected'] = 'release_abort'
                return case
            campaign.prepare_case = prepare
            destination = args.output / name
            destination.mkdir()
            campaign.write_json(args.output / 'status.json', dict(stage='RUNNING', case=name))
            options = SimpleNamespace(autopilot=args.autopilot.resolve(), output=destination,
                                      reviewed_plan=args.reviewed_plan.resolve(), world=args.world, gui=args.gui)
            result = campaign.run_one('reviewed-plan', options)
            assessment = assess_case(destination / 'reviewed-plan')
            if name == 'nominal' and not assessment['sampled_rehearsal_passed']:
                raise RuntimeError('Nominal release rehearsal failed sampled assessment')
            result.update(release_class='SITL_ONLY', physical_qualification=False,
                          sampled_rehearsal_passed=assessment['sampled_rehearsal_passed'])
            results.append(result)
            campaign.write_json(args.output / 'results.json', results)
    except Exception as exc:
        campaign.write_json(args.output / 'status.json', dict(stage='FAILED', error=str(exc)))
        raise
    finally:
        campaign.prepare_case = original
        lock.close()
    campaign.write_json(args.output / 'status.json', dict(stage='COMPLETED', results=results))
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
