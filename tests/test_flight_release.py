"""Physical release integrity and live refusal tests with synthetic evidence.

Temporary fixtures are deliberately not aircraft qualification evidence.
"""
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mission_planner.aircraft import controller_overrides
from mission_planner.editor import preview
from mission_planner.flight_release import (CATEGORIES, PLAN_FILES, ROOT, digest,
                                            initialize, inspect_plan, prepare,
                                            source_digest, write_json)
from mission_planner.flight_preflight import FlightAuthority
sys.path.insert(0, str(ROOT / "controller"))
from mission_config import MissionConfig  # noqa: E402
from VehicleState import VehicleState  # noqa: E402
import offboard_runner  # noqa: E402


class FlightReleaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p = json.loads((ROOT / "config/planner/aircraft/x500.json").read_text())
        p.update(usable_battery_wh=200., contingency_time_s=190.)
        document = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        document["path"]["points"] = [document["path"]["points"][0], document["path"]["points"][-1]]
        document["path"]["maximum_deviation_m"] = 1.
        from mission_planner.aircraft import assess_profile
        setup = assess_profile(p)
        document.update(vehicle_model_id=p["profile_id"], environment_id="profile-env-"+setup["profile_sha256"][:16])
        cls.result = preview(dict(source="drawing", document=document, aircraft_profile=p))
        assert cls.result["accepted"]

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="SYNTHETIC-release-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.plan = self.root / "plan"
        self.plan.mkdir()
        r = self.result
        (self.plan / "trajectory.csv").write_text(r["trajectory_csv"])
        for filename, key in (("summary.json", "summary"), ("intent.json", "intent"),
                              ("verification.json", "verification"), ("vehicle_model.json", "vehicle_model"),
                              ("environment.json", "environment"), ("mission_budget.json", "mission_budget"),
                              ("path_error_budget.json", "path_error_budget")):
            write_json(self.plan / filename, r[key])
        write_json(self.plan / "aircraft_profile.json", r["aircraft_setup"]["profile"])
        self.parameters = {f"TEST_{i}": float(i) for i in range(30)}
        self.parameters.update({f"PWM_MAIN_FUNC{i+1}": 101+i for i in range(4)})
        write_json(self.plan / "simulation_binding.json", {"controller_parameters": self.parameters})
        self.config = MissionConfig(**dict(controller_overrides(r["aircraft_setup"]["profile"]),
                                          cruise_height_m=r["summary"]["controller_cruise_height_m"]))
        self.run = self.plan / "rehearsal/reviewed-plan"
        (self.run / "plan").mkdir(parents=True)
        for n in PLAN_FILES:
            shutil.copyfile(self.plan / n, self.run / "plan" / n)
        write_json(self.run / "simulation_configuration.json", dict(flight_release_source_sha256=source_digest()))
        write_json(self.run / "assessment.json", dict(campaign_ran=True, sampled_rehearsal_passed=True,
                   scenario_expected="nominal", independent_contact_and_hold_audit=dict(overall_passed=True),
                   checks=[dict(passed=True)]))
        write_json(self.run / "final_ground.json", dict(armed=False, landed_state=1,
                   continuous_confirmation_s=2., actuators=[0.]*4))
        (self.run / "batch/reviewed-plan").mkdir(parents=True)
        write_json(self.run / "batch/reviewed-plan/run_manifest_SYNTHETIC.json",
                   dict(outcome="SUCCESS", cleanup_error=None, controller_config=asdict(self.config),
                        px4_parameters=self.parameters,
                        controller_runtime=dict(trajectory_sha256=digest(self.plan / "trajectory.csv")),
                        source_sha256={p.name:digest(p) for p in (ROOT / "controller").glob("*.py")}))
        for name in ("batch/reviewed-plan/audit.json", "batch/reviewed-plan/scenario.json",
                     "batch/reviewed-plan/research_log_SYNTHETIC.csv", "runtime/SYNTHETIC.ulg"):
            f = self.run / name
            f.parent.mkdir(parents=True, exist_ok=True)
            f.write_text("SYNTHETIC UNIT TEST ONLY")
        assessment = json.loads((self.run / "assessment.json").read_text())
        evidence_files = [self.run / name for name in ("plan/trajectory.csv", "final_ground.json",
            "batch/reviewed-plan/audit.json", "batch/reviewed-plan/scenario.json",
            "batch/reviewed-plan/research_log_SYNTHETIC.csv", "runtime/SYNTHETIC.ulg",
            "batch/reviewed-plan/run_manifest_SYNTHETIC.json")]
        assessment["evidence_sha256"] = {str(f.relative_to(self.run)):digest(f) for f in evidence_files}
        write_json(self.run / "assessment.json", assessment)
        self.request_path = self.root / "request.json"
        self.request = initialize(self.plan, self.request_path)
        self.request.update(reviewer="SYNTHETIC TEST REVIEWER",
                            expires_at=(datetime.now(timezone.utc)+timedelta(hours=1)).isoformat())
        self.request["aircraft"] = dict(uid2_hex="01"*18, flight_sw_version=123,
                                       flight_custom_version_hex="02"*8, board_version=1,
                                       target_system=1, target_component=1)
        self.request["site"] = dict(latitude_deg=47., longitude_deg=8., launch_radius_m=5.,
                                    cleared_radius_m=150., measured_wind_upper_bound_m_s=0.)
        self.request["launch"].update(reviewed_at=datetime.now(timezone.utc).isoformat(), operator="TEST OPERATOR", mentor="TEST MENTOR",
                                      mentor_present=True, manual_override_tested=True, area_clear=True,
                                      minimum_pack_voltage_v=14.)
        self.request["motor_output_indices"] = [0, 1, 2, 3]
        self.request["motor_disarmed_values"] = [0.] * 4
        self.request["motor_stopped_tolerance"] = .01
        self.request["motor_function_parameters"] = [f"PWM_MAIN_FUNC{i+1}" for i in range(4)]
        for i, e in enumerate(self.request["evidence"]):
            e["path"] = f"synthetic-{i}.json"
            write_json(self.root / e["path"], self.parameters if e["category"] == "px4_parameters" else {"synthetic_test_only": True})
            e["sha256"], e["reviewed_by"] = digest(self.root / e["path"]), "TEST REVIEWER"
        self.output = self.root / "prepared"

    def prepared(self):
        write_json(self.request_path, self.request)
        result = prepare(self.request_path, self.output)
        return FlightAuthority(self.output / "release.json", result["release_sha256"],
                               "TEST OPERATOR", "TEST MENTOR", True, "udp:127.0.0.1:14540")

    def controller(self, authority):
        state = VehicleState()
        now = time.monotonic()
        state.battery_received_at = state.rc_received_at = state.gps_received_at = now
        state.battery_remaining_percent, state.battery_voltage_v = 100, 16.
        state.rc_channel_count, state.rc_rssi = 8, 100
        state.heartbeat_received_at = now
        state.heartbeat_received = True
        state.heartbeat_main_mode = 6
        state.extended_state_received_at = now
        state.landed_state = 1
        state.actuator_received_at = now
        state.actuator_outputs = [0.] * 16
        state.gps_lat_deg, state.gps_lon_deg = 47., 8.
        c = SimpleNamespace(config=self.config, connection_string=authority.connection,
                            rotor_count=4, trajectory_path=self.output / "plan/trajectory.csv",
                            state=state, state_lock=threading.Lock(), max_heartbeat_age_s=1.5)
        authority.bind_controller(c)
        return c

    def test_template_has_all_categories_and_cannot_prepare(self):
        template = json.loads(self.request_path.read_text())
        self.assertEqual({e['category'] for e in template['evidence']}, set(CATEGORIES))
        self.assertFalse(template['launch']['mentor_present'])
        with self.assertRaisesRegex(ValueError, "reviewer"):
            prepare(self.request_path, self.output)
        self.assertFalse(self.output.exists())

    def test_preparation_preserves_sitl_bytes_and_binds_every_file(self):
        a = self.prepared()
        self.assertEqual(digest(self.output / 'plan/trajectory.csv'), digest(self.plan / 'trajectory.csv'))
        self.assertEqual(json.loads((self.output / 'plan/summary.json').read_text())['release_class'], 'SITL_ONLY')
        self.assertEqual(a.release['source_sha256'], source_digest())
        self.assertEqual(a.release['controller_config'], asdict(self.config))
        self.assertFalse((self.output / 'CONSUMED.json').exists())

    def test_review_requires_mentor_identity_expiry_and_all_evidence(self):
        for mutate in (lambda r:r['launch'].update(mentor_present=False),
                       lambda r:r['aircraft'].update(uid2_hex='00'*18),
                       lambda r:r.update(expires_at='2000-01-01T00:00:00Z'),
                       lambda r:r['evidence'].pop(),
                       lambda r:r.update(motor_output_indices=[0,1,2,2]),
                       lambda r:r['site'].update(measured_wind_upper_bound_m_s=1.),
                       lambda r:r['site'].update(cleared_radius_m=10.)):
            original = deepcopy(self.request)
            mutate(self.request)
            write_json(self.request_path, self.request)
            with self.assertRaises(ValueError):
                prepare(self.request_path, self.output)
            self.assertFalse(self.output.exists())
            self.request = original

    def test_rehearsal_changes_and_failed_run_refuse_preparation(self):
        assessment = json.loads((self.run/'assessment.json').read_text())
        assessment['sampled_rehearsal_passed'] = False
        write_json(self.run/'assessment.json', assessment)
        with self.assertRaisesRegex(ValueError, 'passing nominal'):
            inspect_plan(self.plan)
        assessment['sampled_rehearsal_passed'] = True
        write_json(self.run/'assessment.json', assessment)
        (self.run/'plan/trajectory.csv').write_text('tampered')
        with self.assertRaisesRegex(ValueError, 'evidence changed|rehearsed plan changed'):
            inspect_plan(self.plan)

    def test_raw_log_tampering_cannot_keep_a_passing_assessment(self):
        (self.run / "runtime/SYNTHETIC.ulg").write_text("changed log")
        with self.assertRaisesRegex(ValueError, "raw rehearsal evidence changed"):
            inspect_plan(self.plan)

    def test_short_abort_budget_cannot_be_physically_prepared(self):
        from mission_planner.aircraft import assess_profile, mission_budget
        from mission_planner.contracts import canonical_sha256
        profile = json.loads((self.plan / "aircraft_profile.json").read_text())
        profile["contingency_time_s"] = 30.
        setup = assess_profile(profile)
        summary = json.loads((self.plan / "summary.json").read_text())
        budget = mission_budget(setup, self.result["mission_budget"]["route_duration_s"], 20., 20.)
        summary.update(aircraft_profile_sha256=setup["profile_sha256"], mission_budget_sha256=canonical_sha256(budget))
        write_json(self.plan / "aircraft_profile.json", profile)
        write_json(self.plan / "mission_budget.json", budget)
        write_json(self.plan / "summary.json", summary)
        for name in ("aircraft_profile.json", "mission_budget.json", "summary.json"):
            shutil.copyfile(self.plan / name, self.run / "plan" / name)
        self.request["aircraft_profile_sha256"] = setup["profile_sha256"]
        write_json(self.request_path, self.request)
        with self.assertRaisesRegex(ValueError, "abort/landing contingency"):
            prepare(self.request_path, self.output)
        self.assertFalse(self.output.exists())

    def test_tampered_bundle_or_wrong_confirmation_refuses_before_transport(self):
        a = self.prepared()
        c = self.controller(a)
        (self.output/'plan/trajectory.csv').write_text('tampered')
        with self.assertRaisesRegex(ValueError, 'changed'):
            a.bind_controller(c)
        with self.assertRaisesRegex(ValueError, 'confirmation'):
            FlightAuthority(self.output/'release.json', '0'*64, 'TEST OPERATOR', 'TEST MENTOR', True, a.connection)

    def test_runtime_config_change_is_not_authorized(self):
        a = self.prepared()
        c = self.controller(a)
        c.connection_string = 'udp:10.0.0.1:14540'
        with self.assertRaisesRegex(ValueError, 'runtime controller'):
            a.bind_controller(c)

    def test_single_use_confirmation_and_atomic_consumption(self):
        a = self.prepared()
        # Synthetic telemetry has no producer to refresh it while disk hashes
        # are checked. Test one-use consumption without depending on disk speed;
        # stale-telemetry refusal has its own explicit tests below.
        with patch("time.monotonic", return_value=time.monotonic()):
            c = self.controller(a)
            a.before_arm(c)
            self.assertTrue((self.output/'CONSUMED.json').exists())
            with self.assertRaisesRegex(ValueError, 'consumed'):
                a.before_arm(c)
            c.state.armed = True
            a.check(c, force=True)  # Flight does not abruptly expire.

    def test_expired_local_confirmation_refuses_without_consuming(self):
        a = self.prepared()
        c = self.controller(a)
        a.confirmed_at -= 301
        with self.assertRaisesRegex(RuntimeError, 'five minutes'):
            a.before_arm(c)
        self.assertFalse((self.output/'CONSUMED.json').exists())

    def test_stale_battery_and_rc_unknown_and_site_errors_latch_abort(self):
        for key, value in [('battery_received_at', None), ('rc_rssi', 255),
                           ('battery_remaining_percent', 94), ('battery_voltage_v', 13),
                           ('gps_lat_deg', 48.)]:
            with self.subTest(field=key):
                a = self.prepared()
                c = self.controller(a)
                setattr(c.state, key, value)
                with self.assertRaises(RuntimeError):
                    a.check(c, force=True)
                setattr(c.state, key, 100)
                with self.assertRaises(RuntimeError):
                    a.check(c, force=True)
                shutil.rmtree(self.output)

    def test_abort_and_energy_deadline_reach_same_guard_in_bootstrap_and_route(self):
        a = self.prepared()
        c = self.controller(a)
        a.before_arm(c)
        (self.output/'ABORT').touch()
        a.last_check = -float('inf')  # poll after the documented 100 ms cache window
        with self.assertRaisesRegex(RuntimeError, 'abort'):
            offboard_runner.raise_if_controller_failed(c)
        (self.output/'ABORT').unlink()
        a.error = None
        a.started_at -= 1000
        with self.assertRaisesRegex(RuntimeError, 'flight time'):
            a.check(c, force=True)

    def test_live_wrong_identity_never_reads_parameters_or_sends_modes(self):
        a = self.prepared()
        c = self.controller(a)
        version = SimpleNamespace(uid2=[3]*18, flight_sw_version=123,
                                  flight_custom_version=[2]*8, board_version=1)
        c.master = SimpleNamespace(target_system=1, target_component=1, mav=Mock(),
                                   recv_match=Mock(return_value=version))
        c._message_is_from_target = lambda m: True
        with patch('px4_policy.read_px4_parameters') as read:
            with self.assertRaisesRegex(RuntimeError, 'UID2'):
                a.connected(c, self.parameters)
            read.assert_not_called()
        c.master.mav.set_mode_send.assert_not_called()
        self.assertFalse((self.output/'CONSUMED.json').exists())

    def test_live_parameter_change_refuses_and_success_retains_readonly_probe(self):
        from pymavlink import mavutil
        a = self.prepared()
        c = self.controller(a)
        version = SimpleNamespace(uid2=[1]*18, flight_sw_version=123,
                                  flight_custom_version=[2]*8, board_version=1,
                                  capabilities=(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
                                                | mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED))
        c.master = SimpleNamespace(target_system=1, target_component=1, mav=Mock(),
                                   recv_match=Mock(return_value=version))
        c._message_is_from_target = lambda m: True
        with patch('px4_policy.read_px4_parameters', return_value=dict(self.parameters, TEST_0=3)):
            with self.assertRaisesRegex(RuntimeError, 'parameter differs'):
                a.connected(c, self.parameters)
        with patch('px4_policy.read_px4_parameters', return_value=self.parameters):
            a.connected(c, self.parameters)
        self.assertTrue((self.output/'live_preflight.json').exists())
        c.master.mav.param_set_send.assert_not_called()
        c.master.mav.set_mode_send.assert_not_called()

    def test_battery_unknown_and_other_pack_do_not_create_fresh_evidence(self):
        s = VehicleState()
        s.update_battery(SimpleNamespace(id=1, voltages=[4000]*4, battery_remaining=100))
        self.assertIsNone(s.battery_received_at)
        s.update_battery(SimpleNamespace(id=0, voltages=[65535]*10, battery_remaining=-1))
        self.assertTrue(__import__('math').isnan(s.battery_voltage_v))
        self.assertEqual(s.battery_remaining_percent, -1)

    def test_all_motors_in_custom_output_map_are_checked(self):
        s = VehicleState()
        s.actuator_outputs = [0.]*16
        s.actuator_outputs[7] = .5
        s.actuator_received_at = time.monotonic()
        c = SimpleNamespace(state=s, state_lock=threading.Lock(), rotor_count=6,
                            propulsion_output_indices=[0,1,2,3,6,7])
        outputs, _, _ = offboard_runner.propulsion_snapshot(c)
        self.assertEqual(len(outputs), 6)
        self.assertEqual(outputs[-1], .5)
        self.assertFalse(offboard_runner.propulsion_stopped(c))
        c.propulsion_disarmed_values = [900.] * 6
        c.propulsion_stop_tolerance = 1.
        s.actuator_outputs = [900.] * 16
        self.assertTrue(offboard_runner.propulsion_stopped(c))
        s.actuator_outputs[7] = 1100.
        self.assertFalse(offboard_runner.propulsion_stopped(c))


if __name__ == '__main__':
    unittest.main()
