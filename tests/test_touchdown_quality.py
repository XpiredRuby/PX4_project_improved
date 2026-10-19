import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controller"))
from touchdown_quality import touchdown_violations  # noqa: E402


class TouchdownQualityTests(unittest.TestCase):
    def snapshot(self, **changes):
        return dict({"vx": 0., "vy": 0., "vz": .1, "roll": 0., "pitch": 0.,
                     "position_age_s": .05, "attitude_age_s": .05}, **changes)

    def test_normal_contact_and_boundary_are_accepted(self):
        self.assertEqual(touchdown_violations(self.snapshot()), [])
        self.assertEqual(touchdown_violations(self.snapshot(vx=.5, vz=.5,
                                                           roll=math.radians(10))), [])

    def test_speed_tilt_and_unknown_motion_are_rejected(self):
        for changes in ({"vx": .4, "vy": .4}, {"vz": -.6},
                        {"roll": math.radians(15)}, {"position_age_s": .3},
                        {"vx": math.nan}):
            self.assertTrue(touchdown_violations(self.snapshot(**changes)))

    def test_combined_roll_and_pitch_exceed_contact_tilt_limit(self):
        pose = self.snapshot(roll=math.radians(8), pitch=math.radians(8))
        self.assertIn("touchdown tilt exceeded 10 degrees", touchdown_violations(pose))
        self.assertTrue(touchdown_violations(self.snapshot(roll=math.radians(10.01))))

    def test_horizontal_contact_position_requires_finite_frozen_reference(self):
        reference = (12., -3.)
        pose = self.snapshot(x=13.5, y=-3.)
        self.assertEqual(touchdown_violations(pose, reference), [])
        self.assertIn("touchdown position exceeded 1.5 m",
                      touchdown_violations(dict(pose, y=-2.9), reference))
        for reference_xy, changes in [((math.nan, 0.), {}), ((0.,), {}),
                                       (reference, {"x": math.nan})]:
            with self.subTest(reference=reference_xy, changes=changes):
                self.assertTrue(touchdown_violations(dict(pose, **changes), reference_xy))
        self.assertTrue(touchdown_violations(self.snapshot(), reference))
