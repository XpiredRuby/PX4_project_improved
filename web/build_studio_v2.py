"""Package the React engineering workspace with the existing verified browser planner.

Run after: cd studio-v2 && npm install && npm run build
This creates a separate studio-v2/preview directory; it never modifies the
legacy site or grants any simulation or hardware execution authority.
"""
from __future__ import annotations

from pathlib import Path
import json
import shutil
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from web.build_public_studio import _build_into  # noqa: E402

SOURCE = ROOT / "studio-v2/dist"
PREVIEW = ROOT / "studio-v2/preview"


def build() -> None:
    if not (SOURCE / "index.html").is_file():
        raise ValueError("Build React first: cd studio-v2 && npm install && npm run build")
    with tempfile.TemporaryDirectory(prefix="px4-studio-v2-") as temp:
        verified = Path(temp) / "verified"
        information = _build_into(verified)
        staged = Path(temp) / "staged"
        shutil.copytree(SOURCE, staged)
        # Reuse exactly the same Pyodide planner, config, manifest, catalogs and worker.
        # The legacy entrypoint remains untouched.
        skip = {"index.html", "build-info.json"}
        for item in verified.iterdir():
            if item.name in skip:
                continue
            destination = staged / item.name
            if item.is_dir():
                if destination.exists():
                    raise ValueError(f"React dist conflicts with planner directory: {item.name}")
                shutil.copytree(item, destination)
            else:
                if destination.exists():
                    raise ValueError(f"React dist conflicts with planner asset: {item.name}")
                shutil.copy2(item, destination)
        html = (staged / "index.html").read_text()
        html = html.replace('<meta name="theme-color"', '<meta name="px4-build" content="' + information["build_id"] + '"><meta name="theme-color"', 1)
        if "<script src=\"/browser-api.js\"></script>" not in html:
            raise ValueError("React entrypoint must load the isolated planner bridge")
        html = html.replace('/browser-api.js"', '/browser-api.js?build=' + information["build_id"] + '"')
        (staged / "index.html").write_text(html)
        (staged / "studio-v2-info.json").write_text(json.dumps({
            "schema_version": 1,
            "planner_build_id": information["build_id"],
            "execution": "browser planner only",
            "simulation": "not run",
            "weather": "Open-Meteo forecast for airspeed analysis only",
            "physical_flight_approval": False
        }, indent=2) + "\n")
        if PREVIEW.exists():
            if PREVIEW.is_symlink():
                raise ValueError("Refusing to replace symlink")
            shutil.rmtree(PREVIEW)
        shutil.copytree(staged, PREVIEW)
    print(f"React engineering preview packaged at {PREVIEW}, planner {information['build_id']}")


if __name__ == "__main__":
    build()
