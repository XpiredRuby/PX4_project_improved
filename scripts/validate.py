#!/usr/bin/env python3
"""Cross-platform repository validation entrypoint."""

from __future__ import annotations

import compileall
import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "trajectory_generator"
CONTROLLER = ROOT / "controller"


def run(command: list[str], cwd: Path = ROOT) -> None:
    print(f"+ {' '.join(command)}", flush=True)
    subprocess.run(command, cwd=cwd, check=True, env=os.environ.copy())


def main() -> None:
    os.environ.setdefault("MPLBACKEND", "Agg")
    run([sys.executable, "main.py"], cwd=GENERATOR)
    shutil.copy2(GENERATOR / "trajectory.csv", CONTROLLER / "trajectory.csv")

    for directory in (CONTROLLER, GENERATOR, ROOT / "analysis", ROOT / "tests"):
        if not compileall.compile_dir(directory, quiet=1):
            raise RuntimeError(f"Compilation failed in {directory}")

    # Discovery prevents new safety tests from being silently omitted. The
    # trajectory-generator import isolation is covered by its regression test.
    run([
        sys.executable,
        "-m", "unittest", "discover",
        "-s", str(ROOT / "tests"),
        "-v",
    ])

    print("VALIDATION_PASSED")


if __name__ == "__main__":
    main()
