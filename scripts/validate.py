#!/usr/bin/env python3
"""Cross-platform repository validation entrypoint."""

from __future__ import annotations

import argparse
import compileall
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "trajectory_generator"
CONTROLLER = ROOT / "controller"


def run(command: list[str], cwd: Path = ROOT) -> None:
    print(f"+ {' '.join(command)}", flush=True)
    subprocess.run(command, cwd=cwd, check=True, env=os.environ.copy())


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-coverage', action='store_true',
                        help='Collect controller branch coverage during the full test pass')
    args = parser.parse_args(argv)
    os.environ.setdefault("MPLBACKEND", "Agg")
    run([sys.executable, "main.py"], cwd=GENERATOR)
    shutil.copy2(GENERATOR / "trajectory.csv", CONTROLLER / "trajectory.csv")

    with tempfile.TemporaryDirectory() as directory:
        temporary = Path(directory)
        outputs = (temporary / "first.csv", temporary / "second.csv")
        for output in outputs:
            run([
                sys.executable,
                "plan_waypoint_mission.py",
                "--plan", str(ROOT / "missions" / "five_stop_example.json"),
                "--output", str(output),
            ], cwd=GENERATOR)
        if outputs[0].read_bytes() != outputs[1].read_bytes():
            raise RuntimeError("Waypoint mission compilation is not deterministic")

    for directory in (
        CONTROLLER,
        GENERATOR,
        ROOT / "analysis",
        ROOT / "mission_planner",
        ROOT / "tests",
    ):
        if not compileall.compile_dir(directory, quiet=1):
            raise RuntimeError(f"Compilation failed in {directory}")

    # Discovery prevents new safety tests from being silently omitted. The
    # trajectory-generator import isolation is covered by its regression test.
    test_command = [sys.executable]
    if args.controller_coverage:
        test_command += ['-m', 'coverage', 'run', '--branch', '--source=controller']
    run(test_command + [
        "-m", "unittest", "discover",
        "-s", str(ROOT / "tests"),
        "-v",
    ])

    print("VALIDATION_PASSED")


if __name__ == "__main__":
    main()
