"""Closest-feasible preview regressions; no flight runner is started."""
from copy import deepcopy
import csv
import io
import json
import math
import threading
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from mission_planner.closest import closest_preview, _repair_bound, _simplify_sketch
from mission_planner.lifecycle_verification import verify_phase
from mission_planner.preview import preview
from mission_planner.studio_jobs import StudioJobs
from mission_planner.contracts import VehicleModel, EnvironmentSnapshot

ROOT=Path(__file__).resolve().parents[1]


def drawing(points, **motion):
    value=json.loads((ROOT/"missions/drawn_path_example.json").read_text())
    start=value["path"]["points"][0]
    value["path"].update(closure="open_stop",maximum_deviation_m=.5,
        points=[dict(start,point_id=f"p{i}",north_m=n,east_m=e,hard=False,exact=False)
                for i,(n,e) in enumerate(points)])
    value["motion"].update(yaw_mode="tangent",fixed_yaw_deg=None,**motion)
    return dict(source="drawing",document=value)


def refused(*args,**kwargs):
    raise ValueError("test: smooth fit unavailable")


def rows(result):
    return list(csv.DictReader(io.StringIO(result["trajectory_csv"])))


class ClosestPreviewTests(unittest.TestCase):
    def test_dense_vin_handwriting_keeps_original_shadow_and_produces_checked_mission(self):
        corners = [(0,0),(8,0),(0,3),(8,6),(1,8),(7,8),(8,8),(1,8),
                   (1,11),(7,11),(7,14),(1,14)]
        dense = [corners[0]]
        for a,b in zip(corners,corners[1:], strict=False):
            dense.extend((a[0]+(b[0]-a[0])*i/80,a[1]+(b[1]-a[1])*i/80) for i in range(1,81))
        request = drawing(dense)
        result = preview(dict(request,planning_mode="closest"))
        self.assertTrue(result["accepted"])
        self.assertEqual(len(result["requested_intent"]["path"]["points"]),len(dense))
        self.assertLessEqual(len(result["intent"]["path"]["points"]),48)
        self.assertIn("sketch_simplification",[c["code"] for c in result["planning_adjustments"]["adjustments"]])
        self.assertTrue(all(result["verification"]["continuous_phase_checks"].values()))
        self.assertLess(result["planning_adjustments"]["curve_error_bound_from_requested_m"],.5)
        full=list(csv.DictReader(io.StringIO(result["mission"]["trajectory_csv"])))
        self.assertEqual(float(full[0]["z"]),0.)
        self.assertEqual(float(full[-1]["z"]),0.)

    def test_scribble_oversized_input_repairs_instead_of_256_point_refusal(self):
        points=[(0,0)]+[(60*math.sin(i*.73),70*math.sin(i*1.07)) for i in range(1,401)]
        result=preview(dict(drawing(points),planning_mode="closest"))
        self.assertTrue(result["accepted"])
        self.assertEqual(len(result["requested_intent"]["path"]["points"]),401)
        self.assertTrue(all(math.hypot(p["north_m"],p["east_m"])<=40. for p in result["intent"]["path"]["points"]))
        self.assertFalse(result["planning_adjustments"]["original_rounding_met"])
        self.assertGreater(result["planning_adjustments"]["curve_error_bound_from_requested_m"],.5)

    def test_continuous_repair_bound_detects_order_and_simplified_away_loop(self):
        original=drawing([(0,0),(0,5),(5,5),(0,5),(0,0)])["document"]
        simple=_simplify_sketch(original,2)
        self.assertGreaterEqual(_repair_bound(original,simple),math.sqrt(50))
        same=drawing([(0,0),(0,1),(0,2),(0,3)])["document"]
        self.assertLess(_repair_bound(same,_simplify_sketch(same,2)),1e-9)

    def test_real_reversal_falls_back_with_continuous_turns_and_ground_endpoints(self):
        request=drawing([(0,0),(0,3),(0,0)])
        result=preview(dict(request,planning_mode="closest"))
        self.assertTrue(result["accepted"])
        self.assertEqual(result["planning_adjustments"]["method"],"stop_and_turn")
        self.assertTrue(all(result["verification"]["continuous_phase_checks"].values()))
        trajectory=rows(result)
        self.assertGreater(max(float(r["yaw"]) for r in trajectory)-min(float(r["yaw"]) for r in trajectory),3.)
        self.assertTrue(any(p["name"]=="TURN" for p in result["verification"]["phases"]))
        self.assertTrue(all(abs(float(r["x"]))<1e-8 for r in trajectory))
        full=list(csv.DictReader(io.StringIO(result["mission"]["trajectory_csv"])))
        self.assertEqual(float(full[0]["z"]),0.)
        self.assertEqual(float(full[-1]["z"]),0.)
        self.assertFalse(result["flight_approval"])
        self.assertFalse(result["simulation_compatible"])

    def test_real_impossible_deadline_becomes_longer_verified_plan(self):
        request=drawing([(0,0),(0,5)],time_mode="maximum",requested_duration_s=.1)
        result=preview(dict(request,planning_mode="closest"))
        self.assertTrue(result["accepted"])
        self.assertGreater(result["summary"]["retiming"]["selected_duration_s"],.1)
        self.assertIn("route_time",[c["code"] for c in result["planning_adjustments"]["adjustments"]])

    def test_zero_rounding_and_exact_flythrough_can_stop_without_cutting_corner(self):
        request=drawing([(0,0),(0,2),(2,2)])
        request["document"]["path"]["maximum_deviation_m"]=0.
        request["document"]["path"]["points"][1]["exact"]=True
        result=closest_preview(request,refused)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["planning_adjustments"]["curve_error_bound_from_requested_m"],0.)
        self.assertTrue(result["requested_intent"]["path"]["points"][1]["exact"])
        self.assertFalse(result["intent"]["path"]["points"][1]["exact"])
        self.assertTrue(result["intent"]["path"]["points"][1]["hard"])

    def test_duplicate_only_route_is_a_stationary_hover(self):
        result=closest_preview(drawing([(0,0),(0,0),(0,0)]),refused)
        self.assertTrue(result["accepted"])
        self.assertEqual(len(result["verification"]["phases"]),1)
        self.assertEqual(result["verification"]["phases"][0]["name"],"HOLD")
        self.assertTrue(all(float(r["x"])==float(r["y"])==0. for r in rows(result)))

    def test_projection_respects_immutable_model_and_reports_original_error(self):
        request=drawing([(0,0),(0,100)],maximum_speed_m_s=50,target_speed_m_s=40)
        for p in request["document"]["path"]["points"]:
            p["down_m"]=-80.
        original=deepcopy(request)
        result=closest_preview(request,refused)
        self.assertEqual(request,original)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["vehicle_model"]["maximum_speed_m_s"],3.)
        self.assertLessEqual(result["intent"]["motion"]["maximum_speed_m_s"],3.)
        self.assertTrue(all(math.hypot(p["north_m"],p["east_m"])<=40. for p in result["intent"]["path"]["points"]))
        self.assertTrue(all(-p["down_m"]<=30. for p in result["intent"]["path"]["points"]))
        self.assertFalse(result["planning_adjustments"]["original_rounding_met"])
        self.assertGreater(result["planning_adjustments"]["curve_error_bound_from_requested_m"],60.)

    def test_non_origin_first_point_gets_a_declared_launch_leg(self):
        result=closest_preview(drawing([(5,0),(8,0)]),refused)
        self.assertEqual(result["intent"]["path"]["points"][0]["north_m"],0.)
        self.assertGreaterEqual(result["planning_adjustments"]["requested_path_displacement_bound_m"],5.)

    def test_non_positive_speed_is_refused_without_mutating_the_drawing(self):
        request=drawing([(0,0),(0,1)],maximum_speed_m_s=0.,target_speed_m_s=-1.)
        original=deepcopy(request)
        with self.assertRaisesRegex(ValueError,"Maximum speed must be greater than zero"):
            closest_preview(request,refused)
        self.assertEqual(request,original)

    def test_periodic_two_lap_visit_count_is_preserved(self):
        # Interview converter supplies loop count; exercise its full preparation.
        doc=dict(schema_version=1,mission_id="circle-test",vehicle_model_id="px4-x500-sitl-v1",
                 environment_id="empty-world-v1",pattern="circle",altitude_m=5.,maximum_deviation_m=.5,
                 maximum_speed_m_s=1.25,target_speed_m_s=1.,yaw_mode="tangent",fixed_yaw_deg=None,
                 time_mode="minimize",requested_duration_s=None,loop_count=2,
                 circle=dict(center_north_m=1.,center_east_m=0.))
        result=closest_preview(dict(source="interview",document=doc),refused)
        self.assertEqual(result["summary"]["retiming"]["loop_count"],2)
        legs=[p for p in result["verification"]["phases"] if p["name"]=="LEG"]
        returns=sum(math.hypot(*p["end"][:2])<1e-8 for p in legs)
        self.assertEqual(returns,2)

    def test_tiny_circle_no_longer_refuses_its_radius(self):
        doc=dict(schema_version=1,mission_id="tiny-circle",vehicle_model_id="px4-x500-sitl-v1",
                 environment_id="empty-world-v1",pattern="circle",altitude_m=5.,maximum_deviation_m=.1,
                 maximum_speed_m_s=1.,target_speed_m_s=1.,yaw_mode="tangent",fixed_yaw_deg=None,
                 time_mode="minimize",requested_duration_s=None,loop_count=1,
                 circle=dict(center_north_m=.25,center_east_m=0.))
        result=closest_preview(dict(source="interview",document=doc),refused)
        self.assertTrue(result["accepted"])
        self.assertIn("tiny_circle",[c["code"] for c in result["planning_adjustments"]["adjustments"]])

    def test_independent_checker_rejects_a_too_fast_phase(self):
        result=closest_preview(drawing([(0,0),(0,3)]),refused)
        phase=deepcopy(result["verification"]["phases"][0])
        phase["duration_s"] /= 100
        checked=verify_phase(phase,VehicleModel.from_dict(result["vehicle_model"]),
                             EnvironmentSnapshot.from_dict(result["environment"]))
        self.assertFalse(checked["accepted"])

    def test_resource_fallback_refines_scale_and_reports_geometry_change(self):
        with patch("mission_planner.closest.MAX_ROUTE_ROWS",150):
            result=closest_preview(drawing([(0,0),(0,20)]),refused)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["planning_adjustments"]["method"],"scaled_stop_and_turn")
        self.assertFalse(result["planning_adjustments"]["original_rounding_met"])
        scale=next(c["scale"] for c in result["planning_adjustments"]["adjustments"] if c["code"]=="route_scale")
        self.assertGreater(scale,.125)
        self.assertLess(scale,.5)

    def test_profile_energy_and_uncertainty_stay_binding_during_repair(self):
        profile=json.loads((ROOT/"config/planner/aircraft/x500.json").read_text())
        request=drawing([(0,0),(0,40)])
        request["aircraft_profile"]=profile
        request["document"]["vehicle_model_id"]=profile["profile_id"]
        from mission_planner.aircraft import assess_profile
        setup=assess_profile(profile)
        request["document"]["environment_id"]="profile-env-"+setup["profile_sha256"][:16]
        result=closest_preview(request,refused)
        self.assertTrue(result["accepted"])
        self.assertTrue(result["mission"]["energy_budget"]["passed"])
        self.assertGreaterEqual(result["path_error_budget"]["total_tolerance_m"],
                                profile["position_uncertainty_m"]+profile["tracking_error_m"])
        self.assertEqual(result["vehicle_model"]["mass_kg"],profile["mass_kg"])

    def test_unknown_fields_and_non_finite_data_remain_invalid(self):
        request=drawing([(0,0),(0,1)])
        with self.assertRaises(ValueError):
            closest_preview(dict(request,arm=True),refused)
        request["document"]["path"]["points"][1]["north_m"]=float("nan")
        with self.assertRaises(ValueError):
            closest_preview(request,refused)

    def test_strict_api_still_refuses_an_impossible_deadline(self):
        request=drawing([(0,0),(0,5)],time_mode="maximum",requested_duration_s=.1)
        with self.assertRaises(ValueError):
            preview(request)

    def test_retained_stop_turn_plan_is_refused_before_startup(self):
        state=SimpleNamespace(_lock=threading.RLock(),folder=lambda _:Path("unused"))
        summary=json.dumps({"retiming":{"limiting_constraint":"stop_and_turn"}})
        with patch.object(Path,"read_text",return_value=summary),patch("mission_planner.studio_jobs.subprocess.Popen") as launch:
            with self.assertRaisesRegex(ValueError,"connected playback"):
                StudioJobs.start(state,dict(plan_id="test",controller_trajectory_sha256="digest",confirm_interpretation=True))
            launch.assert_not_called()

    def test_new_preview_certificate_cannot_enter_the_spline_only_runner(self):
        result=closest_preview(drawing([(0,0),(0,1)]),refused)
        self.assertFalse(StudioJobs.compatible(SimpleNamespace(binding=None),result))


if __name__=="__main__":
    unittest.main()
