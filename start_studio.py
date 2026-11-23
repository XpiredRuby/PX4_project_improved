"""One-command launcher for Vin Trajectory Planner.

    python start_studio.py            (Windows: py start_studio.py)

Checks Python packages, builds the web app the first time if Node.js is available,
starts the local server and opens http://127.0.0.1:8767 in your browser.
Extra options are passed to the server, e.g. --port 9000.
"""
from __future__ import annotations

import importlib.util
import shutil
import subprocess
import sys
import threading
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DIST = ROOT / "studio-v2" / "dist"
RELEASES = "https://github.com/XpiredRuby/PX4_project_improved/releases/latest"


def step(text: str) -> None:
    print(f"==> {text}", flush=True)


def ensure_python_packages() -> None:
    missing = [name for name in ("numpy", "pyulog") if importlib.util.find_spec(name) is None]
    if not missing:
        return
    command = [sys.executable, "-m", "pip", "install", "-r", str(ROOT / "requirements.txt")]
    step(f"Missing Python packages: {', '.join(missing)}")
    answer = input("    Install them now with pip? [Y/n] ").strip().lower()
    if answer not in ("", "y", "yes"):
        raise SystemExit("Run this, then start again:\n    " + " ".join(command))
    subprocess.check_call(command)


def ensure_web_app() -> None:
    if (DIST / "index.html").is_file():
        return
    npm = shutil.which("npm")
    if npm is None:
        raise SystemExit(
            "The web app has not been built and Node.js is not installed.\n"
            "Either install Node.js 22+ (https://nodejs.org) and run this again,\n"
            f"or download the ready-to-run ZIP instead: {RELEASES}"
        )
    step("First run: building the web app (about a minute)")
    subprocess.check_call([npm, "ci"], cwd=ROOT / "studio-v2")
    subprocess.check_call([npm, "run", "build"], cwd=ROOT / "studio-v2")


def main() -> None:
    if sys.version_info < (3, 10):
        raise SystemExit("Python 3.10 or newer is required.")
    ensure_python_packages()
    ensure_web_app()
    port = 8767
    if "--port" in sys.argv:
        port = int(sys.argv[sys.argv.index("--port") + 1])
    url = f"http://127.0.0.1:{port}"
    step(f"Opening {url}  (press Ctrl+C here to stop)")
    threading.Timer(1.5, lambda: webbrowser.open(url)).start()
    sys.path.insert(0, str(ROOT))
    from mission_planner.research_studio import main as serve

    serve()


if __name__ == "__main__":
    main()
