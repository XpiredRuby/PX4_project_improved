"""Fault-injection tests for command ownership and shutdown evidence."""
# ruff: noqa: E402
from dataclasses import replace
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))

from PID_position_new import PositionController
from VehicleState import VehicleState
from magless_bootstrap import _raw_navigation_reasons, send_attitude_target
from mission_config import MissionConfig
from mission_state import FailureAction
import offboard_runner
from touchdown_quality import touchdown_violations
from test_fixed_overlay import FakeMaster, FakeMessage
import test_magless_bootstrap
import test_fixed_overlay
from test_runner_protocol import FakeController, TickClock


class ControlLifecycleTests(unittest.TestCase):
    def test_estimator_replay_cannot_renew_validity_or_replace_flags(self):
        state = VehicleState()
        message = dict(time_usec=1000000, flags=55, vel_ratio=.1,
                       pos_horiz_ratio=.1, pos_vert_ratio=.1, mag_ratio=0., hagl_ratio=0.)
        with patch.object(state, "_now", return_value=1.):
            state.update_estimator_status(SimpleNamespace(**message))
        with patch.object(state, "_now", return_value=2.):
            state.update_estimator_status(SimpleNamespace(**dict(message, flags=0)))
        self.assertEqual(state.estimator_flags, 55)
        self.assertEqual(state.estimator_source_advanced_at, 1.)
        self.assertEqual(state.estimator_received_at, 2.)
        state.update_estimator_status(SimpleNamespace(**dict(message, time_usec=999999)))
        self.assertTrue(state.estimator_source_regressed)
        self.assertEqual(state.estimator_time_usec, 1000000)

    def test_missing_invalid_estimator_timestamp_is_not_fresh_evidence(self):
        state = VehicleState()
        for timestamp in (float("nan"), float("inf"), -1.):
            state.update_estimator_status(SimpleNamespace(time_usec=timestamp))
        state.update_estimator_status(SimpleNamespace())
        self.assertIsNone(state.estimator_source_advanced_at)
        self.assertFalse(state.estimator_received)

    def test_stale_regressed_estimator_cannot_authorize_failsafe_brake(self):
        for change in ({"estimator_source_age_s": 2.}, {"estimator_age_s": float("nan")},
                       {"estimator_source_regressed": True}, {"estimator_source_age_s": -.1}):
            with self.subTest(change=change):
                c = self.controller()
                c._snapshot = lambda _now, values=change: dict(
                    self.healthy_brake_pose(), **values
                )
                c._watchdog_step(.3)
                self.assertEqual(c.master.mav.calls, [])
                self.assertTrue(c.native_land_active)

    def test_arm_requires_fresh_disarmed_offboard_ownership_at_send_time(self):
        c = self.controller()
        c.master.mav.command_long_send = Mock()
        for heartbeat in ((None, None, None), (3, 0, False), (6, 0, True)):
            with self.subTest(heartbeat=heartbeat):
                with patch.object(offboard_runner, "heartbeat_snapshot", return_value=heartbeat):
                    with self.assertRaisesRegex(RuntimeError, "OFFBOARD ownership"):
                        offboard_runner.request_arm(c)
                c.master.mav.command_long_send.assert_not_called()
        with patch.object(offboard_runner, "heartbeat_snapshot", return_value=(6, 0, False)):
            offboard_runner.request_arm(c)
        c.master.mav.command_long_send.assert_called_once()

    def test_arm_rechecks_ownership_after_waiting_for_transport(self):
        c = self.controller()
        c.master.mav.command_long_send = Mock()

        def heartbeat(_controller):
            self.assertTrue(c.mav_send_lock.locked())
            return 3, 0, True

        with patch.object(offboard_runner, "heartbeat_snapshot", side_effect=heartbeat):
            with self.assertRaisesRegex(RuntimeError, "OFFBOARD ownership"):
                offboard_runner.request_arm(c)
        c.master.mav.command_long_send.assert_not_called()

    def test_arm_never_sends_after_receiver_failure_or_handoff(self):
        for field, value in (("receiver_error", "disconnected"),
                             ("native_land_active", True), ("setpoint_error", "expired")):
            with self.subTest(field=field):
                c = self.controller()
                setattr(c, field, value)
                c.master.mav.command_long_send = Mock()
                with patch.object(offboard_runner, "heartbeat_snapshot", return_value=(6, 0, False)):
                    with self.assertRaisesRegex(RuntimeError, "OFFBOARD ownership"):
                        offboard_runner.request_arm(c)
                c.master.mav.command_long_send.assert_not_called()

    def test_duplicate_measurement_time_cannot_replace_position_or_attitude(self):
        state = VehicleState()
        position = SimpleNamespace(time_boot_ms=100, x=1., y=2., z=-3., vx=.1, vy=.2, vz=.3)
        attitude = SimpleNamespace(time_boot_ms=100, roll=.1, pitch=.2, yaw=.3,
                                   rollspeed=.4, pitchspeed=.5, yawspeed=.6)
        with patch.object(state, "_now", return_value=1.):
            state.update_position(position)
            state.update_attitude(attitude)
        with patch.object(state, "_now", return_value=1.1):
            state.update_position(SimpleNamespace(**dict(vars(position), x=500., vx=50.)))
            state.update_attitude(SimpleNamespace(**dict(vars(attitude), roll=3., rollspeed=30.)))
        self.assertEqual((state.x, state.vx, state.roll, state.roll_rate), (1., .1, .1, .4))
        self.assertEqual((state.position_received_at, state.attitude_received_at), (1.1, 1.1))
        self.assertEqual((state.position_source_advanced_at, state.attitude_source_advanced_at), (1., 1.))

    def controller(self):
        c = PositionController()
        c.master = FakeMaster()
        c.running = c.control_running = True
        c.tracking_reference = (1., 2., -10.)
        c.latest_setpoint = (1., 0., 0., 0.)
        c.latest_setpoint_updated_at = 0.
        c.setpoint_last_sent_at = 0.
        return c

    def test_watchdog_resends_short_gap_but_cannot_extend_expired_motion(self):
        c = self.controller()
        with patch("PID_position_new.time.monotonic", return_value=.15):
            c._watchdog_step(.15)
        self.assertEqual(len(c.master.mav.calls), 1)
        self.assertEqual(c.latest_setpoint_updated_at, 0.)
        # Recent successful transport activity cannot renew command authority.
        c.setpoint_last_sent_at = .49
        c._watchdog_step(.5)
        self.assertEqual(len(c.master.mav.calls), 1)
        self.assertIn("expired", c.setpoint_error)
        self.assertEqual(c.failure_action_snapshot(), FailureAction.PX4_FAILSAFE)
        self.assertTrue(c.native_land_active)
        self.assertFalse(c.control_running)
        with patch("PID_position_new.time.monotonic", return_value=.51):
            self.assertFalse(c.publish_velocity(2., 0., 0., 0.))
        self.assertEqual(len(c.master.mav.calls), 1)

    def test_watchdog_unknown_or_regressed_publication_time_yields(self):
        for stamp in (None, math.nan, math.inf, 2.):
            with self.subTest(stamp=stamp):
                c = self.controller()
                c.latest_setpoint_updated_at = stamp
                c._watchdog_step(1.)
                self.assertTrue(c.native_land_active)
                self.assertEqual(c.master.mav.calls, [])

    def test_watchdog_samples_clock_after_waiting_for_publication_lock(self):
        c = self.controller()
        acquired = threading.Event()

        def sampled_clock():
            self.assertTrue(c.setpoint_lock.locked())
            acquired.set()
            return .21

        with patch("PID_position_new.time.monotonic", side_effect=sampled_clock):
            with c.setpoint_lock:
                watchdog = threading.Thread(target=c._watchdog_step)
                watchdog.start()
                c.latest_setpoint_updated_at = .20
                c.setpoint_last_sent_at = .20
            watchdog.join(1.)
        self.assertFalse(watchdog.is_alive())
        self.assertTrue(acquired.is_set())
        self.assertIsNone(c.setpoint_error)
        self.assertFalse(c.native_land_active)

    def test_watchdog_loop_leaves_clock_sampling_to_locked_step(self):
        c = self.controller()
        c.setpoint_watchdog_stop = Mock()
        c.setpoint_watchdog_stop.wait.side_effect = [False, True]
        with patch.object(c, "_watchdog_step") as step:
            c._setpoint_watchdog_loop()
        step.assert_called_once_with()

    def healthy_brake_pose(self):
        return {"x": 1., "y": 2., "z": -10., "vx": 1., "vy": 0., "vz": 0., "yaw": .2,
                "armed": True, "heartbeat_main_mode": 6, "estimator_flags": 55,
                "position_age_s": .01, "position_source_age_s": .01,
                "attitude_age_s": .01, "attitude_source_age_s": .01,
                "heartbeat_age_s": .01, "estimator_age_s": .01}

    def test_expired_control_brakes_once_with_fresh_navigation_then_yields(self):
        c = self.controller()
        c._snapshot = lambda _now: self.healthy_brake_pose()
        c._watchdog_step(.3)
        self.assertEqual(len(c.master.mav.calls), 1)
        self.assertEqual(c.master.mav.calls[0][8:11], (0., 0., 0.))
        self.assertEqual(c.setpoint_failsafe_brakes, 1)
        c._watchdog_step(.4)
        self.assertEqual(len(c.master.mav.calls), 1)
        self.assertTrue(c.native_land_active)
        self.assertFalse(c.control_running)

    def test_expiry_never_brakes_with_invalid_navigation_or_other_owner(self):
        for change in ({"heartbeat_main_mode": 3}, {"armed": False},
                       {"estimator_flags": 1}, {"heartbeat_age_s": 2.},
                       {"position_source_age_s": math.inf}, {"attitude_source_regressed": True}):
            with self.subTest(change=change):
                c = self.controller()
                c._snapshot = lambda _now, change=change: dict(self.healthy_brake_pose(), **change)
                c._watchdog_step(.3)
                self.assertEqual(c.master.mav.calls, [])
                self.assertTrue(c.native_land_active)

    def test_prearm_mode_request_does_not_reclaim_an_armed_or_unknown_vehicle(self):
        c = self.controller()
        for heartbeat in ((6, 0, True), (None,) * 3):
            with self.subTest(heartbeat=heartbeat):
                c.master.mav.set_mode_send = Mock()
                with patch.object(offboard_runner, "heartbeat_snapshot", return_value=heartbeat):
                    with self.assertRaisesRegex(RuntimeError, "fresh disarmed"):
                        offboard_runner.ensure_mode(c, "OFFBOARD", 1., False,
                                                   require_disarmed=True)
                c.master.mav.set_mode_send.assert_not_called()

    def test_watchdog_reads_latest_command_after_transport_lock(self):
        c = self.controller()
        c.latest_setpoint = (2., 1., 0., .3)
        c.latest_setpoint_updated_at = .1
        with patch("PID_position_new.time.monotonic", return_value=.2):
            c.send_velocity(1., 0., 0., 0., source="watchdog")
        self.assertEqual(c.master.mav.calls[-1][8:11], (2., 1., 0.))
        self.assertEqual(c.master.mav.calls[-1][14], .3)

    def test_command_waiting_for_transport_cannot_send_after_native_handoff(self):
        c = self.controller()
        started = threading.Event()
        result = []

        def delayed_send():
            started.set()
            result.append(c.send_velocity(1., 0., 0., 0.))

        with c.mav_send_lock:
            t = threading.Thread(target=delayed_send)
            t.start()
            self.assertTrue(started.wait(1.))
            c.begin_native_land_handoff()
        t.join(1.)
        self.assertFalse(t.is_alive())
        self.assertEqual(result, [False])
        self.assertEqual(c.master.mav.calls, [])

    def test_slow_computation_cannot_publish_old_feedback_as_fresh(self):
        c = self.controller()
        with patch("PID_position_new.time.monotonic", return_value=.3):
            with self.assertRaisesRegex(RuntimeError, "computation"):
                c.publish_velocity(1., 0., 0., 0., computed_at=0.)
        self.assertTrue(c.native_land_active)
        self.assertEqual(c.master.mav.calls, [])

    def test_velocity_expiring_while_waiting_for_transport_never_sends(self):
        c = self.controller()
        with patch("PID_position_new.time.monotonic", side_effect=[0., .3]):
            with self.assertRaisesRegex(RuntimeError, "waiting for transport"):
                c.publish_velocity(1., 0., 0., 0., computed_at=0.)
        self.assertTrue(c.native_land_active)
        self.assertFalse(c.control_running)
        self.assertEqual(c.master.mav.calls, [])

    def test_bootstrap_waiting_for_transport_rechecks_age_and_ownership(self):
        c = self.controller()
        c.master.mav.set_attitude_target_send = Mock()
        healthy = {"yaw": 0., "heartbeat_age_s": .1, "armed": True, "heartbeat_main_mode": 6}
        c._snapshot = Mock(return_value=healthy)
        with patch("magless_bootstrap.time.monotonic", side_effect=[0., .3]):
            with self.assertRaisesRegex(RuntimeError, "expired"):
                send_attitude_target(c, 0., .6)
        for change in ({"heartbeat_age_s": 2.}, {"armed": None}, {"heartbeat_main_mode": 3}):
            c._snapshot = Mock(side_effect=[healthy, dict(healthy, **change)])
            with patch("magless_bootstrap.time.monotonic", side_effect=[0., .1]):
                with self.assertRaisesRegex(RuntimeError, "ownership"):
                    send_attitude_target(c, 0., .6)
        c.master.mav.set_attitude_target_send.assert_not_called()

    def test_nonfinite_commands_never_reach_transport(self):
        c = self.controller()
        for index in range(4):
            command = [0.] * 4
            command[index] = math.nan
            with self.subTest(index=index), self.assertRaises(ValueError):
                c.send_velocity(*command)
        c._snapshot = lambda _now: {"yaw": 0.}
        for pitch, thrust, yaw in ((math.nan, .6, 0.), (0., -.1, 0.),
                                  (0., 1.1, 0.), (0., .6, math.inf)):
            with self.subTest(pitch=pitch, thrust=thrust, yaw=yaw):
                with self.assertRaises(ValueError):
                    send_attitude_target(c, pitch, thrust, yaw)
        self.assertEqual(c.master.mav.calls, [])

    def test_stop_does_not_send_an_unsolicited_setpoint(self):
        c = self.controller()
        c.stop()
        self.assertEqual(c.master.mav.calls, [])

    def test_other_component_cannot_supply_flight_state_or_command_ack(self):
        c = self.controller()
        for kind in ("HEARTBEAT", "COMMAND_ACK", "LOCAL_POSITION_NED",
                     "ATTITUDE", "ACTUATOR_OUTPUT_STATUS", "EXTENDED_SYS_STATE"):
            with self.subTest(kind=kind):
                self.assertFalse(c.handle_mavlink_message(FakeMessage(kind, 1, 2)))
        self.assertEqual(c.state.message_counts, {})

    def test_heartbeat_evidence_expires_and_recovers(self):
        c = self.controller()
        c.state.heartbeat_received = True
        c.state.heartbeat_received_at = 0.
        c.state.heartbeat_main_mode = 4
        c.state.heartbeat_sub_mode = 6
        c.state.armed = False
        for now, expected in ((1., (4, 6, False)), (1.51, (None,) * 3),
                              (-1., (None,) * 3), (math.nan, (None,) * 3)):
            with patch.object(offboard_runner.time, "monotonic", return_value=now):
                self.assertEqual(offboard_runner.heartbeat_snapshot(c), expected)

    def test_missing_heartbeat_cannot_confirm_landing_shutdown(self):
        c = FakeController()
        with (patch.object(offboard_runner.time, "monotonic", TickClock()),
              patch.object(offboard_runner.time, "sleep", return_value=None),
              patch.object(offboard_runner, "heartbeat_snapshot", return_value=(None,) * 3),
              patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12)),
              patch.object(offboard_runner, "propulsion_snapshot", return_value=((0.,)*4, .1, 4))):
            with self.assertRaisesRegex(TimeoutError, "safe shutdown"):
                offboard_runner.wait_for_native_landing(c, 10, .4, 0.)

    def test_heartbeat_gap_resets_continuous_shutdown_confirmation(self):
        c = FakeController()
        readings = iter([(4, 6, False), (None,) * 3, (4, 6, False)])

        def heartbeat(_c):
            return next(readings, (4, 6, False))

        with (patch.object(offboard_runner.time, "monotonic", TickClock(.02)),
              patch.object(offboard_runner.time, "sleep", return_value=None),
              patch.object(offboard_runner, "heartbeat_snapshot", side_effect=heartbeat),
              patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12)),
              patch.object(offboard_runner, "propulsion_snapshot", return_value=((0.,)*4, .1, 4))):
            offboard_runner.wait_for_native_landing(c, 10, 2., .2)
        self.assertGreaterEqual(c.native_samples, 6)

    def test_incomplete_propulsion_vector_never_establishes_shutdown(self):
        for outputs in ((), (0.,), (0.,)*3):
            c = FakeController()
            with (self.subTest(outputs=outputs),
                  patch.object(offboard_runner.time, "monotonic", TickClock()),
                  patch.object(offboard_runner.time, "sleep", return_value=None),
                  patch.object(offboard_runner, "heartbeat_snapshot", return_value=(4, 6, False)),
                  patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12)),
                  patch.object(offboard_runner, "propulsion_snapshot", return_value=(outputs, .1, 4))):
                with self.assertRaises(TimeoutError):
                    offboard_runner.wait_for_native_landing(c, 10, .4, 0.)

    def test_runner_observes_watchdog_failure_even_when_worker_has_no_error(self):
        c = self.controller()
        c.setpoint_error = "Control command expired"
        with self.assertRaisesRegex(RuntimeError, "setpoint_error"):
            offboard_runner.raise_if_controller_failed(c)
        self.assertEqual(c.failure_action_snapshot(), FailureAction.PX4_FAILSAFE)

    def test_prearm_ground_gate_rejects_an_already_armed_vehicle(self):
        c = FakeController()
        with (patch.object(offboard_runner, "heartbeat_snapshot", return_value=(6, 0, True)),
              patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12))):
            with self.assertRaisesRegex(RuntimeError, "already armed"):
                offboard_runner.wait_for_initial_ground_state(c, .2)

    def test_lost_arm_ack_and_unknown_heartbeat_use_failsafe_cleanup(self):
        c = Mock()
        c.master = object()
        c.failure_action = FailureAction.LAND
        c.set_failure_action.side_effect = lambda action: setattr(c, "failure_action", action)
        c.failure_action_snapshot.side_effect = lambda: c.failure_action
        record = Mock()
        with (patch.object(offboard_runner, "PositionController", return_value=c),
              patch.object(offboard_runner, "RunRecord", return_value=record),
              patch.object(offboard_runner, "audit_px4_configuration", return_value={}),
              patch.object(offboard_runner, "wait_for_bootstrap_ready"),
              patch.object(offboard_runner, "wait_for_initial_ground_state"),
              patch.object(offboard_runner, "capture_launch_reference"),
              patch.object(offboard_runner, "stream_for"),
              patch.object(offboard_runner, "ensure_mode"),
              patch.object(offboard_runner, "request_arm", return_value=0),
              patch.object(offboard_runner, "wait_for_command_ack", side_effect=TimeoutError("lost ACK")),
              patch.object(offboard_runner, "heartbeat_snapshot", return_value=(None,) * 3),
              patch.object(offboard_runner, "landing_snapshot", return_value=(2, .1, 12)),
              patch.object(offboard_runner, "final_state_snapshot", return_value={}),
              patch.object(offboard_runner, "wait_for_failsafe_landing") as cleanup):
            with self.assertRaisesRegex(TimeoutError, "lost ACK"):
                offboard_runner.main()
        cleanup.assert_called_once()
        c.set_failure_action.assert_called_with(FailureAction.PX4_FAILSAFE)
        self.assertEqual(record.finalize.call_args.kwargs["cleanup_status"],
                         "px4_failsafe_land_and_disarm_confirmed")

    def test_prearm_failure_does_not_command_landing_for_another_owner(self):
        c = Mock()
        c.master = object()
        c.failure_action_snapshot.return_value = FailureAction.LAND
        with (patch.object(offboard_runner, "PositionController", return_value=c),
              patch.object(offboard_runner, "RunRecord"),
              patch.object(offboard_runner, "audit_px4_configuration", return_value={}),
              patch.object(offboard_runner, "wait_for_bootstrap_ready", side_effect=RuntimeError("prearm rejected")),
              patch.object(offboard_runner, "heartbeat_snapshot", return_value=(6, 0, True)),
              patch.object(offboard_runner, "final_state_snapshot", return_value={}),
              patch.object(offboard_runner, "wait_for_native_landing") as cleanup):
            with self.assertRaisesRegex(RuntimeError, "prearm rejected"):
                offboard_runner.main()
        cleanup.assert_not_called()
        c.prepare_native_land_handoff.assert_not_called()
        c.stop.assert_called_once()

    def test_bootstrap_rejects_stale_heartbeat_or_pilot_takeover(self):
        c, healthy = test_magless_bootstrap.MaglessBootstrapTests._healthy_raw_navigation()
        for update in ({"heartbeat_age_s": 2.}, {"armed": True, "heartbeat_main_mode": 3},
                       {"attitude_source_age_s": 2.}, {"position_source_regressed": True}):
            with self.subTest(update=update):
                self.assertTrue(_raw_navigation_reasons(c, dict(healthy, **update)))

    def test_attitude_replay_does_not_renew_source_freshness_and_regression_latches(self):
        state = VehicleState()
        msg = SimpleNamespace(time_boot_ms=100, roll=.1, pitch=.2, yaw=.3,
                              rollspeed=0., pitchspeed=0., yawspeed=0.)
        with patch.object(state, "_now", return_value=1.):
            state.update_attitude(msg)
        with patch.object(state, "_now", return_value=2.):
            state.update_attitude(msg)
        self.assertEqual(state.attitude_received_at, 2.)
        self.assertEqual(state.attitude_source_advanced_at, 1.)
        state.update_attitude(SimpleNamespace(**dict(vars(msg), time_boot_ms=99, roll=2.)))
        self.assertTrue(state.attitude_source_regressed)
        self.assertEqual(state.roll, .1)

    def test_contact_rejects_replayed_measurements(self):
        healthy = {"vx": 0., "vy": 0., "vz": 0., "roll": 0., "pitch": 0.,
                   "position_age_s": .01, "attitude_age_s": .01}
        for update in ({"position_source_age_s": 1.}, {"attitude_source_age_s": math.inf},
                       {"attitude_source_regressed": True}):
            with self.subTest(update=update):
                self.assertTrue(touchdown_violations(dict(healthy, **update)))

    def test_invalid_deadlines_counts_types_and_command_age_are_rejected(self):
        for update in ({"mission_timeout_s": 0.}, {"land_timeout_s": -1.},
                       {"align_hold_s": 0.}, {"gps_max_age_s": -1.},
                       {"home_min_samples": 3.5}, {"gps_min_satellites": True},
                       {"land_timeout_s": "180"}, {"max_control_command_age_s": .1},
                       {"max_control_command_age_s": .6}, {"bootstrap_pitch_deg": 30.}):
            with self.subTest(update=update), self.assertRaises(ValueError):
                replace(MissionConfig(), **update).validate()

    def test_worker_resuming_after_handoff_does_not_write_stale_offboard_row(self):
        c = test_fixed_overlay.FixedOverlayTests().make_controller()
        c.master = FakeMaster()
        c.log_file = Mock(closed=False)
        c.writer = Mock()
        c._snapshot = lambda _now: {"x": c.x0, "y": c.y0, "z": c.z0, "yaw": c.yaw0}
        c._validate_runtime_health = lambda *_args: None
        c._feedback_position = lambda snapshot: (snapshot["x"], snapshot["y"], snapshot["z"])
        c._update_tracking_governor = lambda *_args: None
        c.update_phase = lambda *_args: None
        entered = threading.Event()
        release = threading.Event()

        def blocked_row(**_args):
            entered.set()
            release.wait(2.)
            return {"phase": "TAKEOFF"}

        c._build_log_row = blocked_row
        t = threading.Thread(target=c.run)
        t.start()
        try:
            self.assertTrue(entered.wait(1.))
            c.begin_native_land_handoff()
        finally:
            release.set()
            t.join(2.)
            c.running = False
        self.assertFalse(t.is_alive())
        self.assertIsNone(c.worker_error)
        c.writer.writerow.assert_not_called()

    def test_delayed_disarm_is_reported_after_motor_shutdown(self):
        c = FakeController()
        with (patch.object(offboard_runner.time, "monotonic", TickClock(.2)),
              patch.object(offboard_runner.time, "sleep", return_value=None),
              patch.object(offboard_runner, "heartbeat_snapshot",
                           side_effect=lambda _c: (4, 6, c.native_samples < 9)),
              patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12)),
              patch.object(offboard_runner, "propulsion_snapshot", return_value=((0.,)*4, .1, 4))):
            with self.assertRaisesRegex(offboard_runner.UnsafeTouchdown, "automatic disarm"):
                offboard_runner.wait_for_native_landing(c, 10, 30., 0.)
        self.assertGreaterEqual(c.native_samples, 9)

    def test_timeout_return_is_an_abort_even_when_landing_succeeds(self):
        c = Mock()
        c.config = replace(MissionConfig(), mission_timeout_s=.01)
        c.worker_error = c.receiver_error = c.setpoint_error = None
        c.master = object()
        c.failure_action_snapshot.return_value = FailureAction.LAND
        worker = Mock()
        worker.is_alive.return_value = False
        record = Mock()
        with (patch.object(offboard_runner, "PositionController", return_value=c),
              patch.object(offboard_runner, "RunRecord", return_value=record),
              patch.object(offboard_runner.threading, "Thread", return_value=worker),
              patch.object(offboard_runner.time, "monotonic", TickClock()),
              patch.object(offboard_runner, "audit_px4_configuration",
                           return_value={"MPC_THR_HOVER": .6, "MPC_THR_MAX": 1.}),
              patch.object(offboard_runner, "wait_for_bootstrap_ready"),
              patch.object(offboard_runner, "wait_for_initial_ground_state"),
              patch.object(offboard_runner, "capture_launch_reference"),
              patch.object(offboard_runner, "stream_for"),
              patch.object(offboard_runner, "ensure_mode"),
              patch.object(offboard_runner, "request_arm", return_value=0),
              patch.object(offboard_runner, "wait_for_command_ack"),
              patch.object(offboard_runner, "wait_for_armed"),
              patch.object(offboard_runner, "run_magless_yaw_bootstrap"),
              patch.object(offboard_runner, "recover_local_home"),
              patch.object(offboard_runner, "heartbeat_snapshot", return_value=(4, 6, False)),
              patch.object(offboard_runner, "landing_snapshot", return_value=(1, .1, 12)),
              patch.object(offboard_runner, "final_state_snapshot", return_value={}),
              patch.object(offboard_runner, "wait_for_native_landing")):
            offboard_runner.main()
        c.request_return_home.assert_called_once_with("mission timeout")
        self.assertEqual(str(record.finalize.call_args.kwargs["outcome"]), "ABORTED_TO_LAND")


if __name__ == "__main__":
    unittest.main()
