"""Offline studio acceptance cases for uncapped requests and bounded references."""
import hashlib
import json
from pathlib import Path
import unittest

from mission_planner.aircraft import assess_profile
from mission_planner.preview import preview

ROOT = Path(__file__).resolve().parents[1]


class ResearchRequestTests(unittest.TestCase):
    def test_requested_speed_survives_retiming_and_verified_reference_is_bounded(self):
        cases = [
            ("straight", [(0, 0), (8, 0)], 25, 25, False),
            ("sharp turn", [(0, 0), (4, 0), (4, 4)], 50, 50, False),
            ("named stop", [(0, 0), (2, 0), (5, 1)], 1e200, 1.25, True),
            ("short segment", [(0, 0), (.00001, 0), (3, 1)], 50, 2, False),
        ]
        for name, coords, requested, cap, stop in cases:
            with self.subTest(name=name):
                document = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
                document["path"]["points"] = [
                    dict(point_id=f"P{i}", north_m=n, east_m=e, down_m=-6,
                         hard=stop or i in (0, len(coords) - 1), exact=False)
                    for i, (n, e) in enumerate(coords)]
                document["motion"].update(target_speed_m_s=requested, maximum_speed_m_s=cap)
                result = preview(dict(planning_mode="closest", source="drawing", document=document))
                self.assertTrue(result["accepted"], result.get("error"))
                speed = result["summary"]["speed_preferences"]
                self.assertEqual(speed["requested_cruise_m_s"], requested)
                self.assertLessEqual(speed["planned_route_peak_sampled_m_s"], min(cap, 3) + 1e-6)
                digest = hashlib.sha256(result["trajectory_csv"].encode()).hexdigest()
                self.assertEqual(result["summary"]["controller_trajectory_sha256"], digest)
                self.assertFalse(result["campaign_ran"])

    def test_same_geometry_under_two_repository_aircraft_profiles(self):
        hashes = []
        for name in ("x500", "hex"):
            with self.subTest(name=name):
                profile = json.loads((ROOT / f"config/planner/aircraft/{name}.json").read_text())
                setup = assess_profile(profile)
                self.assertTrue(setup["ready"])
                document = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
                document["path"]["points"] = [document["path"]["points"][0], document["path"]["points"][-1]]
                document["path"]["maximum_deviation_m"] = 1.5
                document.update(vehicle_model_id=profile["profile_id"],
                                environment_id="profile-env-" + setup["profile_sha256"][:16])
                document["motion"].update(target_speed_m_s=50, maximum_speed_m_s=50)
                result = preview(dict(planning_mode="closest", source="drawing", document=document,
                                      aircraft_profile=profile))
                self.assertTrue(result["accepted"], result.get("error"))
                self.assertEqual(result["vehicle_model"]["vehicle_model_id"], profile["profile_id"])
                hashes.append(result["summary"]["vehicle_model_sha256"])
        self.assertNotEqual(*hashes)
