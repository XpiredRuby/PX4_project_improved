#!/usr/bin/env python3

import os
import sys
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXED = ROOT / "controller"
sys.path.insert(0, str(FIXED))

from PID_Controller import PIDController
from PID_position_new import PositionController, wrapped_angle


class FakeMav:
    def __init__(self):
        self.calls = []

    def set_position_target_local_ned_send(self, *args):
        self.calls.append(args)


class FakeMaster:
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = FakeMav()


class FakeMessage:
    def __init__(self, msg_type, source_system, source_component):
        self.msg_type = msg_type
        self.source_system = source_system
        self.source_component = source_component

    def get_type(self):
        return self.msg_type

    def get_srcSystem(self):
        return self.source_system

    def get_srcComponent(self):
        return self.source_component


class FixedOverlayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.previous_cwd = Path.cwd()
        # The validated controller intentionally loads trajectory.csv from
        # its launch directory, matching the WSL runtime deployment.
        os.chdir(FIXED)

    @classmethod
    def tearDownClass(cls):
        os.chdir(cls.previous_cwd)

    def make_controller(self):
        controller = PositionController()
        controller.x0 = 1.0
        controller.y0 = -2.0
        controller.z0 = 0.05
        controller.yaw0 = 0.25
        controller.target_x = controller.x0
        controller.target_y = controller.y0
        controller.target_z = controller.z0 - 8.0
        controller.pid_x.setpoint = controller.target_x
        controller.pid_y.setpoint = controller.target_y
        controller.pid_z.setpoint = controller.target_z
        return controller

    def test_takeoff_holds_common_xy_origin(self):
        controller = self.make_controller()
        control = controller.takeoff_controller(
            x=1.20,
            y=-2.10,
            z=0.05,
            yaw=0.25,
            dt=0.05,
        )
        self.assertEqual(
            control["desired"][:2],
            (controller.x0, controller.y0),
        )
        self.assertLess(control["command"][0], 0.0)
        self.assertGreater(control["command"][1], 0.0)
        self.assertEqual(control["target"].x, 0.0)
        self.assertEqual(control["target"].y, 0.0)

    def test_landing_holds_trajectory_endpoint(self):
        controller = self.make_controller()
        controller.land_x = 2.0
        controller.land_y = -1.0
        controller.land_yaw_unwrapped = 0.5
        controller.pid_x.setpoint = controller.land_x
        controller.pid_y.setpoint = controller.land_y
        control = controller.landing_controller(
            x=2.25,
            y=-1.25,
            z=-7.95,
            yaw=0.4,
            dt=0.05,
        )
        self.assertLess(control["command"][0], 0.0)
        self.assertGreater(control["command"][1], 0.0)
        self.assertEqual(
            control["desired"][:2],
            (controller.land_x, controller.land_y),
        )
        self.assertAlmostEqual(control["yaw_unwrapped"], 0.5)

    def test_all_phase_controllers_share_exact_result_contract(self):
        controller = self.make_controller()
        controller.mission_time = 0.0
        controller.land_x = controller.x0
        controller.land_y = controller.y0
        controller.land_yaw_unwrapped = controller.yaw0
        expected_keys = {
            "target",
            "desired",
            "planned",
            "pid",
            "command",
            "limited",
            "yaw_unwrapped",
            "yaw_cmd",
        }
        results = [
            controller.takeoff_controller(1.0, -2.0, 0.05, 0.25, 0.05),
            controller.trajectory_controller(1.0, -2.0, 0.05, 0.05),
            controller.landing_controller(1.0, -2.0, -7.95, 0.25, 0.05),
            controller.done_controller(1.0, -2.0, 0.05, 0.25),
        ]
        for result in results:
            self.assertEqual(set(result), expected_keys)
            self.assertEqual(len(result["desired"]), 3)
            self.assertEqual(len(result["planned"]), 3)
            self.assertEqual(len(result["pid"]), 3)
            self.assertEqual(len(result["command"]), 3)
            self.assertEqual(len(result["limited"]), 3)
            self.assertAlmostEqual(
                result["yaw_cmd"],
                wrapped_angle(result["yaw_unwrapped"]),
            )

    def test_horizontal_velocity_limit_is_vector_norm(self):
        controller = self.make_controller()
        command, limited = controller._limit_velocity_command(
            3.0,
            4.0,
            2.0,
        )
        self.assertAlmostEqual(
            (command[0] ** 2 + command[1] ** 2) ** 0.5,
            controller.max_horizontal_speed,
        )
        self.assertEqual(command[2], controller.max_vertical_speed)
        self.assertEqual(limited, (True, True, True))

    def test_pid_conditional_antiwindup(self):
        pid = PIDController(
            Kp=1.0,
            Ki=1.0,
            Kd=0.0,
            setpoint=10.0,
            output_limits=(-1.0, 1.0),
        )
        for _ in range(100):
            self.assertEqual(pid.update(0.0, 0.05), 1.0)
        self.assertAlmostEqual(pid._integral, 0.0)

    def test_trajectory_loader_exposes_exported_yaw_derivatives(self):
        point = self.make_controller().trajectory.points[0]
        self.assertTrue(hasattr(point, "yaw_acceleration"))
        self.assertTrue(hasattr(point, "yaw_jerk"))

    def test_velocity_mask_excludes_force_bit(self):
        controller = self.make_controller()
        controller.master = FakeMaster()
        controller.send_velocity(1.0, 2.0, 3.0, 0.4)
        args = controller.master.mav.calls[-1]
        self.assertEqual(args[4], 0b0000100111000111)
        self.assertEqual(args[4] & (1 << 9), 0)

    def test_runtime_health_rejects_stale_or_wrong_mode(self):
        controller = self.make_controller()
        controller.phase = "TRAJECTORY"
        healthy = {
            "position_age_s": 0.01,
            "attitude_age_s": 0.01,
            "heartbeat_age_s": 0.10,
            "armed": True,
            "heartbeat_main_mode": 6,
        }
        controller._validate_runtime_health(healthy)

        stale = dict(healthy, position_age_s=0.30)
        with self.assertRaisesRegex(RuntimeError, "Stale runtime telemetry"):
            controller._validate_runtime_health(stale)

        wrong_mode = dict(healthy, heartbeat_main_mode=4)
        with self.assertRaisesRegex(RuntimeError, "left OFFBOARD"):
            controller._validate_runtime_health(wrong_mode)

    def test_foreign_heartbeat_cannot_override_px4_state(self):
        controller = self.make_controller()
        controller.master = FakeMaster()

        px4_heartbeat = FakeMessage("HEARTBEAT", 1, 1)
        qgc_heartbeat = FakeMessage("HEARTBEAT", 255, 190)
        other_px4_message = FakeMessage("LOCAL_POSITION_NED", 1, 2)

        self.assertTrue(
            controller._message_is_from_target(px4_heartbeat)
        )
        self.assertFalse(
            controller._message_is_from_target(qgc_heartbeat)
        )
        self.assertTrue(
            controller._message_is_from_target(other_px4_message)
        )

    def test_runner_deadlines_use_monotonic_clock(self):
        source = (
            FIXED / "vishnu_offboard_runner.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("time.time()", source)
        self.assertIn("time.monotonic()", source)

    def test_watchdog_resends_when_control_publication_stalls(self):
        controller = self.make_controller()
        controller.master = FakeMaster()
        controller.running = True
        controller._start_setpoint_watchdog()
        time.sleep(0.24)
        controller.running = False
        controller.setpoint_watchdog_stop.set()
        controller.setpoint_thread.join(timeout=1.0)

        self.assertIsNone(controller.setpoint_error)
        self.assertGreaterEqual(controller.setpoint_watchdog_resends, 1)
        self.assertGreaterEqual(
            len(controller.master.mav.calls),
            controller.setpoint_watchdog_resends,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
