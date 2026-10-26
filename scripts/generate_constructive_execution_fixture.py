"""Retain a constructive reference fixture without starting a flight runner.

Usage: python scripts/generate_constructive_execution_fixture.py OUTPUT_DIRECTORY
The fixture is accepted by the planner and deliberately remains incompatible
with the baseline spline-only runner. It must not be relabeled executable.
"""

import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission_planner.closest import closest_preview  # noqa: E402


def smooth_unavailable(*args, **kwargs):
    raise ValueError("Fixture deliberately selects the constructive family")


def generate(destination):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    doc = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
    base = doc["path"]["points"][0]
    doc["path"].update(
        closure="open_stop", maximum_deviation_m=0,
        points=[dict(base, point_id=f"p{i}", north_m=n, east_m=e, down_m=-5,
                     hard=True, exact=False)
                for i, (n, e) in enumerate([(0, 0), (0, 2), (2, 2)])],
    )
    doc["motion"].update(yaw_mode="tangent", fixed_yaw_deg=None)
    request = dict(source="drawing", document=doc)
    result = closest_preview(request, smooth_unavailable)
    assert result["accepted"] and not result["simulation_compatible"]
    assert not result["mission"]["execution_ready"]
    assert not result["flight_approval"] and not result["campaign_ran"]
    for name, value in (("request.json", request), ("result.json", result)):
        (destination/name).write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")
    for name, text, expected in (
        ("route.csv", result["trajectory_csv"], result["summary"]["controller_trajectory_sha256"]),
        ("full-mission.csv", result["mission"]["trajectory_csv"], result["mission"]["trajectory_sha256"]),
    ):
        (destination/name).write_bytes(text.encode())
        assert hashlib.sha256((destination/name).read_bytes()).hexdigest() == expected
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(destination.iterdir())}
    (destination/"sha256.json").write_text(json.dumps(hashes, indent=2)+"\n")
    # Keep the complete unmodified response and CSVs together without repeating
    # over a megabyte of numeric text in the repository's review diff.
    paths = sorted(destination.iterdir())
    with zipfile.ZipFile(destination/"artifacts.zip", "w") as archive:
        for path in paths:
            entry = zipfile.ZipInfo(path.name, date_time=(2026, 10, 26, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(entry, path.read_bytes())
    for name in ("result.json", "route.csv", "full-mission.csv"):
        (destination/name).unlink()
    print("Accepted reference retained; baseline native execution remains refused.")


if __name__ == "__main__":
    generate(sys.argv[1])
