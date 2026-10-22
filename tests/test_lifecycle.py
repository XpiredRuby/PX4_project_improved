"""Mission endpoints, continuous phase bounds and route identity regressions."""

import csv
from copy import deepcopy
import hashlib
import io
import json
import math
from pathlib import Path
import unittest

from mission_planner.preview import preview
from mission_planner.contracts import (
    MissionIntent,
    VehicleModel,
    EnvironmentSnapshot,
    load_contract,
)
from mission_planner.lifecycle import build_mission, _evaluate
from mission_planner.lifecycle_verification import derivative_bound, verify_phase
from mission_planner.export import FIELDS

ROOT = Path(__file__).resolve().parents[1]


class LifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = load_contract(
            ROOT / "config/planner/x500_sitl_vehicle_model.json", VehicleModel.from_dict
        )
        cls.env = load_contract(
            ROOT / "config/planner/empty_world_environment.json",
            EnvironmentSnapshot.from_dict,
        )
        document = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        document["path"]["points"] = [
            document["path"]["points"][0],
            document["path"]["points"][-1],
        ]
        document["motion"].update(yaw_mode="tangent", fixed_yaw_deg=None)
        cls.result = preview(dict(source="drawing", document=document))

    def test_origin_lifecycle_strict_clock_and_controller_artifact_identity(self):
        r = self.result
        mission = r["mission"]
        self.assertEqual(
            mission["controller_route_sha256"],
            hashlib.sha256(r["trajectory_csv"].encode()).hexdigest(),
        )
        self.assertEqual(
            mission["trajectory_sha256"],
            hashlib.sha256(mission["trajectory_csv"].encode()).hexdigest(),
        )
        rows = list(csv.DictReader(mission["trajectory_csv"].splitlines()))
        self.assertEqual(
            [p["name"] for p in mission["phases"]],
            ["TAKEOFF", "ROUTE", "TURN_HOME", "RETURN_HOME", "LAND"],
        )
        self.assertTrue(
            all(
                float(b["time"]) > float(a["time"])
                for a, b in zip(rows, rows[1:], strict=False)
            )
        )
        for row in (rows[0], rows[-1]):
            for key in (
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "ax",
                "ay",
                "az",
                "jx",
                "jy",
                "jz",
                "yaw_rate",
                "yaw_acceleration",
                "yaw_jerk",
            ):
                self.assertAlmostEqual(float(row[key]), 0.0, places=8)
        self.assertEqual(rows[0]["phase"], "TAKEOFF")
        self.assertEqual(rows[-1]["phase"], "LAND")
        self.assertAlmostEqual(
            max(float(row["height_m"]) for row in rows),
            -r["intent"]["path"]["points"][0]["down_m"],
        )
        self.assertTrue(all(float(row["height_m"]) >= 0 for row in rows))
        self.assertGreater(
            max(float(row["yaw"]) for row in rows)
            - min(float(row["yaw"]) for row in rows),
            1.0,
        )
        self.assertFalse(mission["execution_ready"])
        self.assertFalse(mission["verification"]["actual_landing_or_shutdown_proven"])
        # Original route fields are unchanged, with only its timestamps offset.
        route = list(csv.DictReader(r["trajectory_csv"].splitlines()))
        emitted = [row for row in rows if row["phase"] == "ROUTE"]
        self.assertEqual(len(emitted), len(route) - 1)
        offset = mission["phases"][0]["duration_s"]
        for before, after in zip(route[1:], emitted, strict=True):
            for key in FIELDS:
                self.assertAlmostEqual(
                    float(after[key]),
                    float(before[key]) + (offset if key == "time" else 0),
                    places=8,
                )

    def test_phase_bounds_cover_all_emitted_derivatives_and_stationary_joins(self):
        mission = self.result["mission"]
        rows = list(csv.DictReader(mission["trajectory_csv"].splitlines()))
        for phase in mission["phases"]:
            if phase["name"] == "ROUTE":
                continue
            self.assertTrue(verify_phase(phase, self.vehicle, self.env)["accepted"])
            emitted = [r for r in rows if r["phase"] == phase["name"]]
            bound = phase["verification"]
            for row in emitted:
                for fields, key in (
                    (("vx", "vy", "vz"), "speed_bound_m_s"),
                    (("ax", "ay", "az"), "acceleration_bound_m_s2"),
                    (("jx", "jy", "jz"), "jerk_bound_m_s3"),
                ):
                    self.assertLessEqual(
                        math.sqrt(sum(float(row[k]) ** 2 for k in fields)),
                        bound[key] + 1e-8,
                    )
            for t in (0.0, phase["duration_s"]):
                row = _evaluate(phase["start"], phase["end"], phase["duration_s"], t)
                self.assertTrue(
                    all(
                        abs(row[k]) < 1e-9
                        for k in (
                            "vx",
                            "vy",
                            "vz",
                            "ax",
                            "ay",
                            "az",
                            "jx",
                            "jy",
                            "jz",
                            "yaw_rate",
                        )
                    )
                )
        for order, known in ((1, 2.1875), (2, 7.5131884044), (3, 52.5)):
            self.assertGreaterEqual(derivative_bound(order), known - 1e-9)
            self.assertLess(derivative_bound(order), known * 1.01)

    def test_fixed_heading_and_closed_route(self):
        r = self.result
        raw = json.loads(json.dumps(r["intent"]))
        raw["motion"].update(yaw_mode="fixed", fixed_yaw_deg=45.0)
        intent = MissionIntent.from_dict(raw)
        route = list(csv.DictReader(r["trajectory_csv"].splitlines()))
        for row in route:
            row.update(
                yaw=str(math.pi / 4), yaw_rate="0", yaw_acceleration="0", yaw_jerk="0"
            )

        def compose():
            s = io.StringIO()
            w = csv.DictWriter(s, fieldnames=FIELDS, lineterminator="\n")
            w.writeheader()
            w.writerows(route)
            return build_mission(s.getvalue(), intent, self.vehicle, self.env)

        mission = compose()
        self.assertTrue(
            all(abs(p["yaw"] - math.pi / 4) < 1e-9 for p in mission["samples"])
        )
        self.assertNotIn("TURN_HOME", [p["name"] for p in mission["phases"]])
        route[-1].update(x="0", y="0")
        self.assertNotIn("RETURN_HOME", [p["name"] for p in compose()["phases"]])
        route[0]["x"] = "1"
        with self.assertRaisesRegex(ValueError, "launch origin"):
            compose()

    def test_sharp_turn_keeps_two_degree_heading_certificate(self):
        raw = json.loads(json.dumps(self.result["intent"]))
        raw["path"]["points"] = [
            dict(
                point_id=f"p{i}",
                north_m=n,
                east_m=e,
                down_m=-20.0,
                hard=False,
                exact=False,
            )
            for i, (n, e) in enumerate(((0, 0), (15, 0), (15, 12)))
        ]
        result = preview(dict(source="drawing", document=raw))
        self.assertTrue(result["verification"]["accepted"])
        route_yaw = [p["yaw"] for p in result["samples"]]
        self.assertGreater(max(route_yaw) - min(route_yaw), 1.0)

    def test_independent_verifier_refuses_bad_time_and_overspeed(self):
        phase = deepcopy(self.result["mission"]["phases"][0])
        for value in (0.0, -1.0, float("nan"), float("inf")):
            phase["duration_s"] = value
            with self.assertRaises(ValueError):
                verify_phase(phase, self.vehicle, self.env)
        phase["duration_s"] = 1.0
        self.assertFalse(verify_phase(phase, self.vehicle, self.env)["accepted"])


if __name__ == "__main__":
    unittest.main()
