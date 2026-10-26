"""Generate cross-runtime rest-to-rest states using exact rational arithmetic.

This oracle imports no planner code. Run from any directory; the committed
fixture is consumed independently by Python verification and browser tests.
"""

import csv
from fractions import Fraction as F
import io
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COEFFICIENTS = (0, 0, 0, 0, 35, -84, 70, -20)
FIELDS = (
    ("x", "y", "z", "yaw"), ("vx", "vy", "vz", "yaw_rate"),
    ("ax", "ay", "az", "yaw_acceleration"),
    ("jx", "jy", "jz", "yaw_jerk"),
)


def state(phase, fraction):
    duration = F(phase["duration_s"])
    start, end = phase["start"], phase["end"]
    row = {"time": float(F(phase["start_time_s"]) + fraction * duration)}
    for order, keys in enumerate(FIELDS):
        value = sum(
            F(c * math.factorial(k), math.factorial(k-order))
            * fraction**(k-order)
            for k, c in enumerate(COEFFICIENTS) if k >= order
        ) / duration**order
        for axis, key in enumerate(keys):
            result = (F(end[axis])-F(start[axis])) * value
            row[key] = float(result + F(start[axis]) if order == 0 else result)
    return row


def generate():
    # Noninteger timing, coupled N/E/Down movement, unwrapped heading above
    # 2*pi and a stationary turn. These are oracle fixtures, not flight plans.
    phases = [
        dict(name="LEG", start=[0, 0, -5, 7], end=[3, -4, -6, 7],
             start_time_s="0", duration_s="7.5"),
        dict(name="TURN", start=[3, -4, -6, 7], end=[3, -4, -6, 11],
             start_time_s="7.5", duration_s="3.25"),
        dict(name="LEG", start=[3, -4, -6, 11], end=[0, 0, -5, 11],
             start_time_s="10.75", duration_s="8.25"),
    ]
    rows, queries = [], []
    for index, phase in enumerate(phases):
        for u in (F(0), F(1, 5), F(7, 20), F(4, 5), F(1)):
            if not (index and u == 0):
                rows.append(dict(state(phase, u), phase=phase["name"]))
        for i in range(41):
            u = F(i, 40)
            queries.append(dict(phase_index=index, fraction=str(u),
                                expected=state(phase, u)))
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return dict(schema_version=1, frame="LOCAL_NED", units="m, s, rad",
                provenance="Exact rational polynomial differentiation; no planner imports",
                flight_approval=False, simulation_ran=False,
                phases=phases, trajectory_csv=stream.getvalue(), queries=queries)


if __name__ == "__main__":
    target = ROOT / "tests/fixtures/septic_reference.json"
    target.write_text(json.dumps(generate(), indent=2, allow_nan=False) + "\n")
    print(target)
