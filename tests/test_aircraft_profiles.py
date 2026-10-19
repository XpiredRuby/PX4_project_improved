"""Aircraft-independent setup, allocation, budgeting and simulator identity."""
from copy import deepcopy
from itertools import product
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from mission_planner.aircraft import assess_profile, import_px4_parameters, mission_budget
from mission_planner.allocation import allocation_margin, right_inverse, vehicle_rotor_duration_s
from mission_planner.contracts import VehicleModel, canonical_sha256
from mission_planner.editor import preview
from mission_planner.simulation_binding import create_binding, inspect_model, validate_binding

ROOT = Path(__file__).resolve().parents[1]


def profile(name="quad"):
    return json.loads((ROOT / f"config/planner/aircraft/{name}.json").read_text())


def request(p):
    setup = assess_profile(p)
    document = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
    document["path"]["points"] = [document["path"]["points"][0], document["path"]["points"][-1]]
    document.update(vehicle_model_id=p["profile_id"], environment_id="profile-env-"+setup["profile_sha256"][:16])
    return dict(source="drawing", document=document, aircraft_profile=p)


class AircraftProfileTests(unittest.TestCase):
    def test_missing_inputs_are_questions_not_assumed_x500_values(self):
        result = assess_profile({"mass_kg": 2.})
        self.assertFalse(result["ready"])
        missing = {q["key"] for q in result["missing_questions"]}
        self.assertIn("rotor_maximum_thrust_n", missing)
        self.assertIn("usable_battery_wh", missing)
        self.assertNotIn("mass_kg", missing)
        self.assertNotIn("vehicle_model", result)

    def test_aircraft_types_and_adapters_are_explicit(self):
        for field, value in (("vehicle_type", "fixed_wing"), ("vehicle_type", "vtol"),
                             ("vehicle_type", "helicopter"), ("autopilot", "ardupilot"), ("navigation", "vio")):
            p = dict(profile(), **{field: value})
            with self.subTest(field=field, value=value):
                result = assess_profile(p)
                self.assertFalse(result["ready"])
                self.assertTrue(result["compatibility_blockers"])

    def test_quad_hex_octo_and_custom_models_round_trip_with_distinct_hashes(self):
        hashes = set()
        for name, count in (("quad", 4), ("hex", 6), ("octo", 8), ("x500", 4)):
            with self.subTest(name=name):
                result = assess_profile(profile(name))
                self.assertTrue(result["ready"], result)
                model = VehicleModel.from_dict(result["vehicle_model"])
                self.assertEqual(len(model.rotors), count)
                self.assertEqual(model.to_dict(), result["vehicle_model"])
                self.assertEqual(canonical_sha256(model), result["vehicle_model_sha256"])
                self.assertFalse(model.identified_for_physical_flight)
                hashes.add(canonical_sha256(model))
        self.assertEqual(len(hashes), 4)

    def test_bad_values_and_unknown_fields_fail_closed(self):
        for field, value in (("mass_kg", True), ("mass_kg", float("nan")),
                             ("mass_kg", "2"), ("maximum_tilt_deg", 90.),
                             ("reserve_fraction", 1.), ("rotor_maximum_thrust_n", -1.)):
            p = dict(profile(), **{field: value})
            with self.subTest(field=field, value=value):
                self.assertFalse(assess_profile(p)["ready"])
        for key in ("command", "vehicle_path", "arm", "identified_for_physical_flight"):
            with self.assertRaisesRegex(ValueError, "unknown"):
                assess_profile(dict(profile(), **{key: True}))

    def test_wind_overload_and_singular_rotor_geometry_are_refused(self):
        for p in (dict(profile(), expected_wind_m_s=4.), dict(profile(), mass_kg=10.)):
            self.assertFalse(assess_profile(p)["ready"])
        p = profile("x500")
        for rotor in p["rotors"]:
            rotor["yaw_torque_per_thrust_m"] = .016
        self.assertFalse(assess_profile(p)["ready"])

    def test_allocation_box_matches_all_corners_and_exact_wrench(self):
        for name in ("quad", "hex", "octo"):
            model = VehicleModel.from_dict(assess_profile(profile(name))["vehicle_model"])
            lo, hi, reserve, yaw = model.mass_kg*9., model.mass_kg*11., .05, .03
            inverse = right_inverse(model.rotors)
            matrix = np.array([[1., -r.y_m, r.x_m, r.yaw_torque_per_thrust_m] for r in model.rotors]).T
            actual = []
            for wrench in product((lo, hi), (-reserve, reserve), (-reserve, reserve), (-yaw, yaw)):
                forces = inverse @ np.array(wrench)
                np.testing.assert_allclose(matrix @ forces, wrench, atol=1e-10)
                for force, rotor in zip(forces, model.rotors, strict=True):
                    actual.extend([(force-rotor.minimum_thrust_n)/(rotor.maximum_thrust_n-rotor.minimum_thrust_n),
                                   (rotor.maximum_thrust_n-force)/(rotor.maximum_thrust_n-rotor.minimum_thrust_n)])
            self.assertAlmostEqual(allocation_margin(model.rotors, lo, hi, yaw, reserve), min(actual))

    def test_more_mass_and_yaw_demand_reduce_feasible_motion(self):
        light = VehicleModel.from_dict(assess_profile(dict(profile(), roll_pitch_torque_reserve_nm=.01))["vehicle_model"])
        heavy = VehicleModel.from_dict(assess_profile(dict(profile(), mass_kg=4., roll_pitch_torque_reserve_nm=.01))["vehicle_model"])
        a = vehicle_rotor_duration_s(light, 100., 10.)[0]
        b = vehicle_rotor_duration_s(heavy, 100., 10.)[0]
        c = vehicle_rotor_duration_s(light, 100., 1000.)[0]
        self.assertGreater(b, a)
        self.assertGreater(c, a)

    def test_mission_budget_includes_lifecycle_contingency_and_reserve(self):
        setup = assess_profile(profile())
        budget = mission_budget(setup, 40., 20., 20.)
        self.assertAlmostEqual(budget["total_budgeted_s"], 270.)
        self.assertAlmostEqual(budget["allowed_flight_s"], 337.5)
        with self.assertRaisesRegex(ValueError, "energy budget exceeded"):
            mission_budget(setup, 400., 20., 20.)

    def test_readonly_parameter_import_provides_reviewable_ceilings(self):
        text = "# QGC export\n1 1 MPC_XY_VEL_MAX 8 9\n1 1 MPC_Z_VEL_MAX_UP 3 9\n1 1 MPC_Z_VEL_MAX_DN 2 9\n1 1 MPC_THR_HOVER .6 9"
        result = import_px4_parameters(text)
        self.assertEqual(result["suggestions"]["maximum_speed_m_s"]["value"], 2.)
        self.assertTrue(result["suggestions"]["maximum_speed_m_s"]["complete_parameter_group"])
        self.assertNotIn("mass_kg", result["suggestions"])
        self.assertTrue(result["requires_review"])
        with self.assertRaises(ValueError):
            import_px4_parameters(text + "\n1 1 MPC_XY_VEL_MAX 7 9")


class AircraftPreviewTests(unittest.TestCase):
    def test_same_drawing_compiles_and_binds_quad_hex_octo_profiles(self):
        for name in ("quad", "hex", "octo", "x500"):
            with self.subTest(name=name):
                payload = request(profile(name))
                before = deepcopy(payload)
                result = preview(payload)
                self.assertTrue(result["accepted"])
                self.assertTrue(result["verification"]["accepted"])
                self.assertEqual(result["vehicle_model"]["vehicle_model_id"], profile(name)["profile_id"])
                self.assertEqual(result["summary"]["aircraft_profile_sha256"], result["aircraft_setup"]["profile_sha256"])
                self.assertEqual(result["summary"]["mission_budget_sha256"], canonical_sha256(result["mission_budget"]))
                self.assertAlmostEqual(result["path_error_budget"]["planning_tolerance_m"], .3)
                self.assertFalse(result["simulation_compatible"])
                self.assertFalse(result["campaign_ran"])
                self.assertEqual(payload, before)

    def test_unanswered_setup_returns_questions_and_no_trajectory(self):
        result = preview({"source": "drawing", "document": {}, "aircraft_profile": {}})
        self.assertFalse(result["accepted"])
        self.assertTrue(result["aircraft_setup"]["missing_questions"])
        self.assertNotIn("trajectory_csv", result)

    def test_energy_tolerance_and_aircraft_id_mismatches_refuse_accepted_plan(self):
        p = profile()
        payload = request(p)
        payload["aircraft_profile"]["usable_battery_wh"] = 1.
        # Updating a profile changes its bound environment; use a fresh request.
        payload = request(payload["aircraft_profile"])
        with self.assertRaisesRegex(ValueError, "energy budget"):
            preview(payload)
        payload = request(profile())
        payload["document"]["path"]["maximum_deviation_m"] = .1
        with self.assertRaisesRegex(ValueError, "tolerance is consumed"):
            preview(payload)
        payload = request(profile())
        payload["document"]["vehicle_model_id"] = "wrong-aircraft"
        with self.assertRaises(ValueError):
            preview(payload)


class SimulationBindingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.autopilot = Path(self.temp.name)
        self.modeldir = self.autopilot / "Tools/simulation/gz/models/test_quad"
        self.modeldir.mkdir(parents=True)
        p = profile()
        setup = assess_profile(p)
        pieces = ['<sdf version="1.9"><model name="test_quad"><link name="base"><inertial><mass>2</mass><inertia><izz>.04</izz></inertia></inertial></link>']
        for i, r in enumerate(setup["vehicle_model"]["rotors"]):
            pieces.append(f'<link name="rotor_{i}"><pose>{r["x_m"]} {-r["y_m"]} 0 0 0 0</pose></link>')
            pieces.append(f'<plugin name="gz::sim::systems::MulticopterMotorModel"><linkName>rotor_{i}</linkName><turningDirection>{"ccw" if r["yaw_torque_per_thrust_m"]>0 else "cw"}</turningDirection><momentConstant>.016</momentConstant><motorConstant>.00001</motorConstant><maxRotVelocity>1000</maxRotVelocity></plugin>')
        pieces.append('</model></sdf>')
        (self.modeldir / "model.sdf").write_text(''.join(pieces))
        airframe = self.autopilot / "ROMFS/px4fmu_common/init.d-posix/airframes/4001_test_quad"
        airframe.parent.mkdir(parents=True)
        airframe.write_text('# test fixture\n')

    def test_binding_checks_mass_geometry_capacity_and_fingerprints(self):
        binding = create_binding(profile(), self.autopilot, "test_quad", 4001)
        self.assertEqual(validate_binding(binding, self.autopilot), binding)
        with self.assertRaisesRegex(ValueError, "mass"):
            create_binding(dict(profile(), mass_kg=3.), self.autopilot, "test_quad", 4001)
        with self.assertRaisesRegex(ValueError, "rotor"):
            create_binding(dict(profile(), rotor_maximum_thrust_n=11.), self.autopilot, "test_quad", 4001)
        (self.modeldir / "model.sdf").write_text((self.modeldir / "model.sdf").read_text()+'\n')
        with self.assertRaisesRegex(ValueError, "assets"):
            validate_binding(binding, self.autopilot)

    def test_browser_paths_and_unhandled_transforms_are_refused(self):
        with self.assertRaises(ValueError):
            inspect_model(self.autopilot, "../../outside")
        path = self.modeldir / "model.sdf"
        path.write_text(path.read_text().replace('<link name="base">', '<link name="base"><pose>0 0 0 0 1 0</pose>'))
        with self.assertRaisesRegex(ValueError, "transform"):
            inspect_model(self.autopilot, "test_quad")


if __name__ == "__main__":
    unittest.main()
