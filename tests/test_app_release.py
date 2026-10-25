"""Release-boundary regressions; no connected simulator is started."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from mission_planner.closest import _models, _motion, _simplify_sketch, closest_preview
from mission_planner.preview import preview
from tests.test_closest_preview import drawing, refused
from web.build_public_studio import build


class AppReleaseTests(unittest.TestCase):
    def test_malformed_payloads_fail_at_the_boundary(self):
        valid = drawing([(0, 0), (0, 1)])
        for payload in [
            None,
            [],
            {},
            dict(valid, source=[]),
            dict(valid, document=[]),
            dict(valid, document={"path": None, "motion": {}}),
            dict(valid, document={"path": {"points": [None]}, "motion": {}}),
            dict(valid, planning_mode="unsafe"),
        ]:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                preview(payload)
        for value in [0, -1, True, "2", None, float("inf"), float("nan")]:
            payload = deepcopy(valid)
            payload["document"]["motion"]["maximum_speed_m_s"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                preview(payload)

    def test_original_motion_survives_adjustment_but_effective_limits_remain_binding(
        self,
    ):
        request = drawing(
            [(0, 0), (0, 2)], maximum_speed_m_s=1.25, target_speed_m_s=2.0
        )
        original = deepcopy(request)
        result = closest_preview(request, refused)
        self.assertEqual(request, original)
        self.assertEqual(result["requested_intent"]["motion"]["target_speed_m_s"], 2.0)
        self.assertEqual(result["intent"]["motion"]["target_speed_m_s"], 1.25)
        adjustment = next(
            c
            for c in result["planning_adjustments"]["adjustments"]
            if c["code"] == "cruise_speed"
        )
        self.assertIn("Never exceed", adjustment["reason"])
        self.assertEqual(result["summary"]["retiming"]["speed_limit_m_s"], 1.125)

    def test_positive_small_caps_are_never_increased(self):
        vehicle, _, _ = _models({})
        motion = {"maximum_speed_m_s": 0.03, "target_speed_m_s": 0.02}
        _motion(motion, vehicle, [])
        self.assertEqual(motion["maximum_speed_m_s"], 0.03)
        self.assertEqual(motion["target_speed_m_s"], 0.02)

    def test_more_than_48_mandatory_marks_survive_simplification(self):
        intent = drawing([(i * 0.1, 0) for i in range(90)])["document"]
        for i, p in enumerate(intent["path"]["points"]):
            p["hard"] = i % 3 != 0
        original = deepcopy(intent)
        result = _simplify_sketch(intent)
        required = [
            p["point_id"]
            for i, p in enumerate(intent["path"]["points"])
            if p["hard"] or i in (0, 89)
        ]
        self.assertEqual([p["point_id"] for p in result["path"]["points"]], required)
        self.assertEqual(intent, original)

    def test_portable_build_is_deterministic_clean_and_bound_to_all_assets(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / "app"
            first = build(dest)
            (dest / "python/removed-flight-runner.py").write_text(
                "old execution module"
            )
            second = build(dest)
            self.assertEqual(first, second)
            self.assertFalse((dest / "python/removed-flight-runner.py").exists())
            manifest = json.loads((dest / "python-manifest.json").read_text())
            self.assertEqual(first["build_id"], manifest["build_id"])
            self.assertIn(first["build_id"], (dest / "index.html").read_text())
            for name, digest in first["assets"].items():
                self.assertEqual(
                    hashlib.sha256((dest / name).read_bytes()).hexdigest(), digest
                )
            self.assertFalse(
                any(
                    "controller/" in name or "studio_jobs" in name
                    for name in first["assets"]
                )
            )
            old = (dest / "index.html").read_bytes()
            with patch(
                "web.build_public_studio.catalog",
                side_effect=ValueError("broken input"),
            ):
                with self.assertRaises(ValueError):
                    build(dest)
            self.assertEqual((dest / "index.html").read_bytes(), old)

    def test_build_refuses_to_replace_user_files_or_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / "user"
            dest.mkdir()
            (dest / "valuable.txt").write_text("keep me")
            with self.assertRaises(ValueError):
                build(dest)
            self.assertEqual((dest / "valuable.txt").read_text(), "keep me")
        with self.assertRaises(ValueError):
            build(Path(__file__).resolve().parents[1])
