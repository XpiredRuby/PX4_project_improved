"""Reproduce the pinned peer's data audit against our retained fixture.

Run from the repository root after fetching the peer commit. This reads pinned
repository code but does not merge it or start a native runner/simulator.
"""

from copy import deepcopy
import ast
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import zipfile


PEER = "72cb875c31bfd1a9a0074e7884986792ad594910"
IMPLEMENTATION = "d212903f3449139ce11db7bada699492e610f684"
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def source(path):
    return subprocess.check_output(["git", "show", f"{PEER}:{path}"], cwd=ROOT)


auditor = source("analysis/reference_audit.py")
namespace = {"__name__": "pinned_peer_reference_audit"}
exec(compile(auditor, "pinned_peer/analysis/reference_audit.py", "exec"), namespace)
audit = namespace["audit_reference"]
vehicle = json.loads((ROOT/"config/planner/x500_sitl_vehicle_model.json").read_text())
environment = json.loads((ROOT/"config/planner/empty_world_environment.json").read_text())
with zipfile.ZipFile(HERE/"constructive-execution/artifacts.zip") as archive:
    results = {name: audit(archive.read(name).decode(), vehicle, environment)
               for name in ("route.csv", "full-mission.csv")}
    route = archive.read("route.csv").decode()
assert all(result["passed"] for result in results.values())

reader = csv.DictReader(io.StringIO(route))
fields, rows = reader.fieldnames, list(reader)
negative = {}
for name, field, value in (("wrong_velocity", "vx", "20"),
                           ("nonfinite_position", "x", "nan"),
                           ("duplicate_clock", "time", rows[0]["time"])):
    damaged = deepcopy(rows)
    damaged[1][field] = value
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows(damaged)
    try:
        report = audit(stream.getvalue(), vehicle, environment)
        negative[name] = dict(rejected=not report["passed"], checks=report["checks"])
    except ValueError as exc:
        negative[name] = dict(rejected=True, error=str(exc))
assert all(result["rejected"] for result in negative.values())

# Evaluate only the peer's simplifier to reproduce the resource-policy conflict.
module = ast.parse(source("mission_planner/closest.py"))
function = next(node for node in module.body
                if isinstance(node, ast.FunctionDef) and node.name == "_simplify_sketch")
scope = dict(deepcopy=deepcopy, math=__import__("math"), MAX_SKETCH_VERTICES=48)
exec(compile(ast.Module(body=[function], type_ignores=[]), "pinned_peer/simplifier", "exec"), scope)
intent = json.loads((ROOT/"missions/drawn_path_example.json").read_text())
intent["path"]["points"] = [dict(point_id=f"p{i}", north_m=i/2, east_m=0,
                                     down_m=-20, hard=True, exact=False)
                            for i in range(49)]
peer_count = len(scope["_simplify_sketch"](intent, 48)["path"]["points"])
assert peer_count == 49
record = dict(auditor_repository="XpiredRuby/PX4_project_improved",
              auditor_commit=PEER, auditor_path="analysis/reference_audit.py",
              auditor_sha256=hashlib.sha256(auditor).hexdigest(),
              reviewed_planner_commit=IMPLEMENTATION,
              scope="Sampled independent reference cross-check; no Gazebo or telemetry",
              results=results, corrupted_artifacts=negative,
              policy_difference=dict(required_points=49, requested_budget=48,
                                     peer_retained_points=peer_count,
                                     our_behavior="Explicit PreviewBudgetError; input unchanged"))
(HERE/"peer-data-audit.json").write_text(json.dumps(record, indent=2, allow_nan=False)+"\n")
print("PASS: both reference artifacts; three corrupted artifacts refused; 49-point policy difference reproduced.")
