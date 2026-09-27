#!/usr/bin/env python3

import csv
import math
import threading
import time

from pymavlink import mavutil

from PID_Controller import PIDController
from VehicleState import VehicleState
from trajectory import Trajectory, TrajectoryPoint


def clamp(value, low, high):
    return max(low, min(value, high))


def wrapped_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class PositionController:
    """Measured-data fixes around Vishnu's outer-loop controller."""

    def __init__(self):
        self.connection_string = "udp:127.0.0.1:14540"
        self.control_rate = 20.0
        self.control_dt = 1.0 / self.control_rate

        self.master = None
        self.state = VehicleState()
        self.state_lock = threading.Lock()
        self.mav_send_lock = threading.Lock()
        self.setpoint_lock = threading.Lock()
        self.setpoint_watchdog_stop = threading.Event()
        self.running = False
        self.receiver_thread = None
        self.setpoint_thread = None
        self.receiver_error = None
        self.setpoint_error = None
        self.worker_error = None

        self.latest_setpoint = None
        self.latest_setpoint_updated_at = None
        self.setpoint_last_sent_at = None
        self.setpoint_last_gap_s = math.nan
        self.setpoint_max_gap_s = 0.0
        self.setpoint_send_count = 0
        self.setpoint_control_sends = 0
        self.setpoint_watchdog_resends = 0
        self.setpoint_watchdog_timeout = 0.10
        self.setpoint_watchdog_poll = 0.02

        self.max_position_age_s = 0.25
        self.max_attitude_age_s = 0.25
        self.max_heartbeat_age_s = 1.50
        self.max_horizontal_speed = 3.0
        self.max_vertical_speed = 1.0
        self.max_trajectory_clock_step_s = 0.10
        self.trajectory_clock_limited = False

        # Baseline gains are intentionally unchanged.
        self.pid_x = PIDController(
            Kp=0.8, Ki=0.0, Kd=0.0, output_limits=(-3.0, 3.0)
        )
        self.pid_y = PIDController(
            Kp=0.8, Ki=0.0, Kd=0.0, output_limits=(-3.0, 3.0)
        )
        self.pid_z = PIDController(
            Kp=1.0, Ki=0.0, Kd=0.0, output_limits=(-1.0, 1.0)
        )

        self.target_x = self.target_y = self.target_z = 0.0
        self.x0 = self.y0 = self.z0 = self.yaw0 = 0.0

        self.phase = "TAKEOFF"
        self.phase_enter_time = None
        self.last_phase_transition = ""
        self.trajectory_start_time = None
        self.mission_time = 0.0

        self.takeoff_altitude = -8.0
        self.takeoff_start_z = None
        self.takeoff_x = None
        self.takeoff_y = None
        self.max_climb_rate = 0.5
        self.takeoff_tolerance = 0.1

        self.land_x = None
        self.land_y = None
        self.land_yaw_unwrapped = None
        self.max_descent_rate = 0.5
        self.descent_accel = 0.3
        self.current_vz_cmd = 0.0
        self.land_slow_altitude = -0.2
        self.ground_z = -0.05

        self.log_file = None
        self.writer = None
        self.filename = None
        self.log_fields = []
        self.log_flush_period = 1.0
        self.status_period = 1.0

        self.trajectory = Trajectory("trajectory.csv")
        self.duration = self.trajectory.duration

    def connect(self, timeout=10.0):
        print(f"Connecting to {self.connection_string}...")
        self.master = mavutil.mavlink_connection(self.connection_string)

        deadline = time.monotonic() + timeout
        heartbeat = None
        while time.monotonic() < deadline:
            candidate = self.master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=min(1.0, max(0.0, deadline - time.monotonic())),
            )
            if candidate is None:
                continue
            if (
                int(getattr(candidate, "autopilot", -1))
                == mavutil.mavlink.MAV_AUTOPILOT_PX4
            ):
                heartbeat = candidate
                break

        if heartbeat is None:
            raise TimeoutError(
                "Timed out waiting for a PX4 autopilot heartbeat"
            )

        self.master.target_system = heartbeat.get_srcSystem()
        self.master.target_component = heartbeat.get_srcComponent()
        with self.state_lock:
            self.state.note_message("HEARTBEAT")
            self.state.update_heartbeat(heartbeat)
        print(
            f"Connected to PX4 autopilot! "
            f"(System {self.master.target_system}, "
            f"Component {self.master.target_component})"
        )

    def _message_is_from_target(self, msg):
        if msg.get_srcSystem() != self.master.target_system:
            return False
        if (
            msg.get_type() == "HEARTBEAT"
            and msg.get_srcComponent() != self.master.target_component
        ):
            return False
        return True

    def request_message_intervals(self):
        requests = [
            ("MAVLINK_MSG_ID_LOCAL_POSITION_NED", 30.0),
            ("MAVLINK_MSG_ID_ATTITUDE", 30.0),
            ("MAVLINK_MSG_ID_HIGHRES_IMU", 20.0),
            ("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED", 20.0),
            ("MAVLINK_MSG_ID_ATTITUDE_TARGET", 20.0),
            ("MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS", 20.0),
            ("MAVLINK_MSG_ID_SERVO_OUTPUT_RAW", 20.0),
        ]
        sent = []
        with self.mav_send_lock:
            for constant_name, rate_hz in requests:
                message_id = getattr(mavutil.mavlink, constant_name, None)
                if message_id is None:
                    continue
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    message_id,
                    1_000_000.0 / rate_hz,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                sent.append(f"{constant_name}={rate_hz:g}Hz")
        print("[research] Requested telemetry: " + ", ".join(sent))

    def start_receiver(self):
        self.running = True
        self.receiver_thread = threading.Thread(
            target=self.mavlink_receiver,
            name="mavlink-receiver",
            daemon=True,
        )
        self.receiver_thread.start()
        self.request_message_intervals()
        print("Started MAVLink receiver thread.")

    def mavlink_receiver(self):
        try:
            while self.running:
                msg = self.master.recv_match(blocking=True, timeout=0.2)
                if msg is None:
                    continue
                msg_type = msg.get_type()
                if msg_type == "BAD_DATA":
                    continue
                if not self._message_is_from_target(msg):
                    continue

                with self.state_lock:
                    self.state.note_message(msg_type)
                    if msg_type == "LOCAL_POSITION_NED":
                        self.state.update_position(msg)
                    elif msg_type == "ATTITUDE":
                        self.state.update_attitude(msg)
                    elif msg_type == "HEARTBEAT":
                        self.state.update_heartbeat(msg)
                    elif msg_type == "HIGHRES_IMU":
                        self.state.update_highres_imu(msg)
                    elif msg_type == "POSITION_TARGET_LOCAL_NED":
                        self.state.update_position_target(msg)
                    elif msg_type == "ATTITUDE_TARGET":
                        self.state.update_attitude_target(msg)
                    elif msg_type == "ACTUATOR_OUTPUT_STATUS":
                        self.state.update_actuator_output_status(msg)
                    elif msg_type == "SERVO_OUTPUT_RAW":
                        self.state.update_servo_output_raw(msg)
        except Exception as exc:
            self.receiver_error = repr(exc)
            print(f"[research] MAVLink receiver failed: {exc!r}")

    def send_velocity(self, vx, vy, vz, yaw, source="direct"):
        # Ignore position, acceleration, and yaw rate. Use velocity and yaw.
        # The FORCE_SET bit is intentionally clear because acceleration is
        # ignored and this command is not a force setpoint.
        with self.mav_send_lock:
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000100111000111,
                0,
                0,
                0,
                vx,
                vy,
                vz,
                0,
                0,
                0,
                yaw,
                0,
            )

        sent_at = time.monotonic()
        with self.setpoint_lock:
            if self.setpoint_last_sent_at is not None:
                self.setpoint_last_gap_s = (
                    sent_at - self.setpoint_last_sent_at
                )
                self.setpoint_max_gap_s = max(
                    self.setpoint_max_gap_s,
                    self.setpoint_last_gap_s,
                )
            self.setpoint_last_sent_at = sent_at
            self.setpoint_send_count += 1
            if source == "control":
                self.setpoint_control_sends += 1
            elif source == "watchdog":
                self.setpoint_watchdog_resends += 1

    def publish_velocity(self, vx, vy, vz, yaw):
        updated_at = time.monotonic()
        with self.setpoint_lock:
            self.latest_setpoint = (vx, vy, vz, yaw)
            self.latest_setpoint_updated_at = updated_at
        self.send_velocity(vx, vy, vz, yaw, source="control")

    def _setpoint_watchdog_loop(self):
        while self.running:
            if self.setpoint_watchdog_stop.wait(
                self.setpoint_watchdog_poll
            ):
                return

            now = time.monotonic()
            with self.setpoint_lock:
                latest = self.latest_setpoint
                latest_updated = self.latest_setpoint_updated_at
                last_sent = self.setpoint_last_sent_at

            if latest is None:
                continue
            if last_sent is None:
                if (
                    latest_updated is None
                    or now - latest_updated
                    < self.setpoint_watchdog_timeout
                ):
                    continue
            elif now - last_sent < self.setpoint_watchdog_timeout:
                continue

            try:
                self.send_velocity(*latest, source="watchdog")
            except Exception as exc:
                self.setpoint_error = repr(exc)
                print(f"[research] Setpoint watchdog failed: {exc!r}")
                return

    def _start_setpoint_watchdog(self):
        with self.setpoint_lock:
            self.latest_setpoint = (0.0, 0.0, 0.0, self.yaw0)
            self.latest_setpoint_updated_at = time.monotonic()
            self.setpoint_last_sent_at = None
            self.setpoint_last_gap_s = math.nan
            self.setpoint_max_gap_s = 0.0
            self.setpoint_send_count = 0
            self.setpoint_control_sends = 0
            self.setpoint_watchdog_resends = 0

        self.setpoint_error = None
        self.setpoint_watchdog_stop.clear()
        self.setpoint_thread = threading.Thread(
            target=self._setpoint_watchdog_loop,
            name="setpoint-watchdog",
            daemon=True,
        )
        self.setpoint_thread.start()

    def _setpoint_stats(self, now_mono):
        with self.setpoint_lock:
            send_age = (
                math.inf
                if self.setpoint_last_sent_at is None
                else max(0.0, now_mono - self.setpoint_last_sent_at)
            )
            command_age = (
                math.inf
                if self.latest_setpoint_updated_at is None
                else max(
                    0.0,
                    now_mono - self.latest_setpoint_updated_at,
                )
            )
            return {
                "setpoint_send_age_s": send_age,
                "latest_setpoint_age_s": command_age,
                "setpoint_last_gap_s": self.setpoint_last_gap_s,
                "setpoint_max_gap_s": self.setpoint_max_gap_s,
                "setpoint_send_count": self.setpoint_send_count,
                "setpoint_control_sends": self.setpoint_control_sends,
                "setpoint_watchdog_resends": (
                    self.setpoint_watchdog_resends
                ),
            }

    def _snapshot(self, now_mono):
        with self.state_lock:
            s = self.state

            def age(timestamp):
                return (
                    math.inf
                    if timestamp is None
                    else max(0.0, now_mono - timestamp)
                )

            snapshot = {
                "x": s.x,
                "y": s.y,
                "z": s.z,
                "vx": s.vx,
                "vy": s.vy,
                "vz": s.vz,
                "roll": s.roll,
                "pitch": s.pitch,
                "yaw": s.yaw,
                "p": s.roll_rate,
                "q": s.pitch_rate,
                "r": s.yaw_rate,
                "mode": s.mode,
                "armed": s.armed,
                "heartbeat_custom_mode": s.heartbeat_custom_mode,
                "heartbeat_main_mode": s.heartbeat_main_mode,
                "heartbeat_sub_mode": s.heartbeat_sub_mode,
                "position_age_s": age(s.position_received_at),
                "attitude_age_s": age(s.attitude_received_at),
                "heartbeat_age_s": age(s.heartbeat_received_at),
                "imu_age_s": age(s.imu_received_at),
                "px4_position_target_age_s": age(
                    s.position_target_received_at
                ),
                "px4_attitude_target_age_s": age(
                    s.attitude_target_received_at
                ),
                "actuator_age_s": age(s.actuator_received_at),
                "servo_age_s": age(s.servo_received_at),
                "position_seq": s.message_counts.get(
                    "LOCAL_POSITION_NED", 0
                ),
                "attitude_seq": s.message_counts.get("ATTITUDE", 0),
                "heartbeat_seq": s.message_counts.get("HEARTBEAT", 0),
                "imu_seq": s.message_counts.get("HIGHRES_IMU", 0),
                "px4_target_seq": s.message_counts.get(
                    "POSITION_TARGET_LOCAL_NED", 0
                ),
                "attitude_target_seq": s.message_counts.get(
                    "ATTITUDE_TARGET", 0
                ),
                "actuator_seq": s.message_counts.get(
                    "ACTUATOR_OUTPUT_STATUS", 0
                ),
                "servo_seq": s.message_counts.get("SERVO_OUTPUT_RAW", 0),
                "imu_xacc": s.imu_xacc,
                "imu_yacc": s.imu_yacc,
                "imu_zacc": s.imu_zacc,
                "imu_xgyro": s.imu_xgyro,
                "imu_ygyro": s.imu_ygyro,
                "imu_zgyro": s.imu_zgyro,
                "px4_target_x": s.px4_target_x,
                "px4_target_y": s.px4_target_y,
                "px4_target_z": s.px4_target_z,
                "px4_target_vx": s.px4_target_vx,
                "px4_target_vy": s.px4_target_vy,
                "px4_target_vz": s.px4_target_vz,
                "px4_target_ax": s.px4_target_ax,
                "px4_target_ay": s.px4_target_ay,
                "px4_target_az": s.px4_target_az,
                "px4_target_yaw": s.px4_target_yaw,
                "px4_target_yaw_rate": s.px4_target_yaw_rate,
                "attitude_target_roll_rate": (
                    s.attitude_target_roll_rate
                ),
                "attitude_target_pitch_rate": (
                    s.attitude_target_pitch_rate
                ),
                "attitude_target_yaw_rate": (
                    s.attitude_target_yaw_rate
                ),
                "attitude_target_thrust": s.attitude_target_thrust,
                "attitude_target_q0": s.attitude_target_q[0],
                "attitude_target_q1": s.attitude_target_q[1],
                "attitude_target_q2": s.attitude_target_q[2],
                "attitude_target_q3": s.attitude_target_q[3],
            }
            for index, value in enumerate(s.actuator_outputs):
                snapshot[f"actuator_output_{index}"] = value
            for index, value in enumerate(s.servo_outputs):
                snapshot[f"servo_raw_{index + 1}"] = value

        snapshot.update(self._setpoint_stats(now_mono))
        return snapshot

    def wait_for_fresh_telemetry(
        self,
        timeout=8.0,
        freshness_limit=2.0,
    ):
        print("Waiting for fresh position, attitude, and heartbeat telemetry...")
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            now = time.monotonic()
            with self.state_lock:
                ready = (
                    self.state.position_received
                    and self.state.attitude_received
                    and self.state.heartbeat_received
                )
                timestamps = (
                    self.state.position_received_at,
                    self.state.attitude_received_at,
                    self.state.heartbeat_received_at,
                )
            fresh = ready and all(
                timestamp is not None
                and now - timestamp <= freshness_limit
                for timestamp in timestamps
            )
            if fresh:
                return
            if self.receiver_error is not None:
                raise RuntimeError(
                    f"MAVLink receiver failed during initialization: "
                    f"{self.receiver_error}"
                )
            time.sleep(0.01)

        with self.state_lock:
            flags = {
                "position": self.state.position_received,
                "attitude": self.state.attitude_received,
                "heartbeat": self.state.heartbeat_received,
            }
        raise TimeoutError(
            f"Timed out waiting for fresh initialization telemetry: "
            f"{flags}"
        )

    def initialize_target(self, timeout=8.0, freshness_limit=2.0):
        self.wait_for_fresh_telemetry(timeout, freshness_limit)

        with self.state_lock:
            self.x0 = self.state.x
            self.y0 = self.state.y
            self.z0 = self.state.z
            self.yaw0 = self.state.yaw

        self.target_x = self.x0
        self.target_y = self.y0
        self.target_z = self.z0 + self.takeoff_altitude

        self._reset_position_pids()
        self.pid_x.setpoint = self.target_x
        self.pid_y.setpoint = self.target_y
        self.pid_z.setpoint = self.target_z

        self.takeoff_start_z = None
        self.takeoff_x = None
        self.takeoff_y = None
        self.land_x = None
        self.land_y = None
        self.land_yaw_unwrapped = None
        self.current_vz_cmd = 0.0
        self.phase = "TAKEOFF"
        self.phase_enter_time = time.monotonic()

        print(
            f"Origin captured immediately before control: "
            f"x={self.x0:.3f} y={self.y0:.3f} "
            f"z={self.z0:.3f} yaw={self.yaw0:.3f}"
        )
        print(
            f"Target: x={self.target_x:.3f} y={self.target_y:.3f} "
            f"z={self.target_z:.3f}"
        )

    def setup_logger(self):
        self.filename = f"research_log_{int(time.time())}.csv"
        self.log_fields = [
            "count",
            "wall_time",
            "monotonic_time",
            "elapsed_s",
            "phase",
            "phase_transition",
            "phase_elapsed_s",
            "mode",
            "armed",
            "heartbeat_custom_mode",
            "heartbeat_main_mode",
            "heartbeat_sub_mode",
            "loop_dt_s",
            "effective_dt_s",
            "loop_lateness_s",
            "missed_periods",
            "compute_time_s",
            "mission_time_s",
            "trajectory_index",
            "trajectory_finished",
            "trajectory_clock_limited",
            "position_age_s",
            "attitude_age_s",
            "heartbeat_age_s",
            "imu_age_s",
            "px4_position_target_age_s",
            "px4_attitude_target_age_s",
            "actuator_age_s",
            "servo_age_s",
            "position_seq",
            "attitude_seq",
            "heartbeat_seq",
            "imu_seq",
            "px4_target_seq",
            "attitude_target_seq",
            "actuator_seq",
            "servo_seq",
            "setpoint_send_age_s",
            "latest_setpoint_age_s",
            "setpoint_last_gap_s",
            "setpoint_max_gap_s",
            "setpoint_send_count",
            "setpoint_control_sends",
            "setpoint_watchdog_resends",
            "desired_x",
            "x",
            "desired_y",
            "y",
            "desired_z",
            "z",
            "planned_vx",
            "pid_x_error",
            "pid_x_p",
            "pid_x_i",
            "pid_x_d",
            "pid_x_correction",
            "cmd_vx",
            "vx",
            "planned_vy",
            "pid_y_error",
            "pid_y_p",
            "pid_y_i",
            "pid_y_d",
            "pid_y_correction",
            "cmd_vy",
            "vy",
            "planned_vz",
            "pid_z_error",
            "pid_z_p",
            "pid_z_i",
            "pid_z_d",
            "pid_z_correction",
            "cmd_vz",
            "vz",
            "pid_x_saturated",
            "pid_y_saturated",
            "pid_z_saturated",
            "command_x_limited",
            "command_y_limited",
            "command_z_limited",
            "roll",
            "pitch",
            "target_yaw",
            "target_yaw_unwrapped",
            "yaw",
            "yaw_error",
            "p",
            "q",
            "r",
            "planned_yaw_rate",
            "planned_yaw_acceleration",
            "planned_yaw_jerk",
            "planned_ax",
            "planned_ay",
            "planned_az",
            "planned_jx",
            "planned_jy",
            "planned_jz",
            "imu_xacc",
            "imu_yacc",
            "imu_zacc",
            "imu_xgyro",
            "imu_ygyro",
            "imu_zgyro",
            "px4_target_x",
            "px4_target_y",
            "px4_target_z",
            "px4_target_vx",
            "px4_target_vy",
            "px4_target_vz",
            "px4_target_ax",
            "px4_target_ay",
            "px4_target_az",
            "px4_target_yaw",
            "px4_target_yaw_rate",
            "attitude_target_roll_rate",
            "attitude_target_pitch_rate",
            "attitude_target_yaw_rate",
            "attitude_target_thrust",
            "attitude_target_q0",
            "attitude_target_q1",
            "attitude_target_q2",
            "attitude_target_q3",
        ] + [
            f"actuator_output_{index}" for index in range(16)
        ] + [
            f"servo_raw_{index + 1}" for index in range(16)
        ]

        self.log_file = open(self.filename, "w", newline="")
        self.writer = csv.DictWriter(
            self.log_file,
            fieldnames=self.log_fields,
            extrasaction="ignore",
        )
        self.writer.writeheader()
        self.log_file.flush()
        print(f"Research logging: {self.filename}")

    def _transition(self, new_phase, now_mono):
        old_phase = self.phase
        self.phase = new_phase
        self.phase_enter_time = now_mono
        self.last_phase_transition = f"{old_phase}->{new_phase}"
        print(f"[research] Phase transition {self.last_phase_transition}")

    def _reset_position_pids(self):
        for pid in (self.pid_x, self.pid_y, self.pid_z):
            pid.reset()

    def update_phase(self, x, y, z, now_mono):
        self.last_phase_transition = ""

        if self.phase == "TAKEOFF":
            if abs(self.target_z - z) < self.takeoff_tolerance:
                self._reset_position_pids()
                self.trajectory.reset()
                self.trajectory_start_time = now_mono
                self.mission_time = 0.0
                self._transition("TRAJECTORY", now_mono)

        elif self.phase == "TRAJECTORY":
            if self.trajectory.finished:
                final_target = self.trajectory.points[-1]
                self.current_vz_cmd = 0.0
                self.land_x = self.x0 + final_target.x
                self.land_y = self.y0 + final_target.y
                self.land_yaw_unwrapped = self.yaw0 + final_target.yaw
                self.pid_x.setpoint = self.land_x
                self.pid_y.setpoint = self.land_y
                self._transition("LAND", now_mono)

        elif self.phase == "LAND":
            if (z - self.z0) >= self.ground_z:
                self._transition("DONE", now_mono)

    @staticmethod
    def _zero_pid_terms():
        return {
            "error": 0.0,
            "p": 0.0,
            "i": 0.0,
            "d": 0.0,
            "correction": 0.0,
            "saturated": False,
        }

    @staticmethod
    def _pid_terms(pid):
        return {
            "error": pid.last_error,
            "p": pid.last_p,
            "i": pid.last_i,
            "d": pid.last_d,
            "correction": pid.last_output,
            "saturated": pid.last_saturated,
        }

    @staticmethod
    def _control_result(
        target,
        desired,
        planned,
        pid_terms,
        command,
        limited,
        yaw_unwrapped,
    ):
        """Build the common phase-controller result without changing policy."""
        return {
            "target": target,
            "desired": desired,
            "planned": planned,
            "pid": pid_terms,
            "command": command,
            "limited": limited,
            "yaw_unwrapped": yaw_unwrapped,
            "yaw_cmd": wrapped_angle(yaw_unwrapped),
        }

    def _limit_velocity_command(self, vx, vy, vz):
        limited_x = False
        limited_y = False
        horizontal_speed = math.hypot(vx, vy)
        if horizontal_speed > self.max_horizontal_speed:
            scale = self.max_horizontal_speed / horizontal_speed
            vx *= scale
            vy *= scale
            limited_x = True
            limited_y = True

        limited_vz = clamp(
            vz,
            -self.max_vertical_speed,
            self.max_vertical_speed,
        )
        return (
            (vx, vy, limited_vz),
            (limited_x, limited_y, limited_vz != vz),
        )

    def takeoff_controller(self, x, y, z, yaw, dt):
        if self.takeoff_start_z is None:
            self.takeoff_start_z = z
            self.takeoff_x = x
            self.takeoff_y = y
            print(
                f"Takeoff start: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

        correction_x = self.pid_x.update(x, dt)
        correction_y = self.pid_y.update(y, dt)
        correction_z = self.pid_z.update(z, dt)
        climb_limited_vz = clamp(
            correction_z,
            -self.max_climb_rate,
            self.max_climb_rate,
        )
        command, limited = self._limit_velocity_command(
            correction_x,
            correction_y,
            climb_limited_vz,
        )
        limited = (
            limited[0],
            limited[1],
            limited[2] or climb_limited_vz != correction_z,
        )

        target = TrajectoryPoint(
            time=0.0,
            x=0.0,
            y=0.0,
            z=self.target_z - self.z0,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw=0.0,
        )

        return self._control_result(
            target=target,
            desired=(self.target_x, self.target_y, self.target_z),
            planned=(0.0, 0.0, 0.0),
            pid_terms=(
                self._pid_terms(self.pid_x),
                self._pid_terms(self.pid_y),
                self._pid_terms(self.pid_z),
            ),
            command=command,
            limited=limited,
            yaw_unwrapped=self.yaw0,
        )

    def trajectory_controller(self, x, y, z, dt):
        target = self.trajectory.get_target(self.mission_time)

        desired_x = self.x0 + target.x
        desired_y = self.y0 + target.y
        desired_z = self.z0 + target.z

        self.pid_x.setpoint = desired_x
        self.pid_y.setpoint = desired_y
        self.pid_z.setpoint = desired_z

        correction_x = self.pid_x.update(x, dt)
        correction_y = self.pid_y.update(y, dt)
        correction_z = self.pid_z.update(z, dt)

        command, limited = self._limit_velocity_command(
            target.vx + correction_x,
            target.vy + correction_y,
            target.vz + correction_z,
        )

        yaw_unwrapped = self.yaw0 + target.yaw
        return self._control_result(
            target=target,
            desired=(desired_x, desired_y, desired_z),
            planned=(target.vx, target.vy, target.vz),
            pid_terms=(
                self._pid_terms(self.pid_x),
                self._pid_terms(self.pid_y),
                self._pid_terms(self.pid_z),
            ),
            command=command,
            limited=limited,
            yaw_unwrapped=yaw_unwrapped,
        )

    def landing_controller(self, x, y, z, yaw, dt):
        target_vz = self.max_descent_rate
        self.current_vz_cmd = min(
            target_vz,
            self.current_vz_cmd + self.descent_accel * dt,
        )

        if (z - self.z0) > self.land_slow_altitude:
            scale = -(z - self.z0) / (-self.land_slow_altitude)
            self.current_vz_cmd = self.max_descent_rate * clamp(
                scale, 0.0, 1.0
            )

        correction_x = self.pid_x.update(x, dt)
        correction_y = self.pid_y.update(y, dt)
        command, limited = self._limit_velocity_command(
            correction_x,
            correction_y,
            self.current_vz_cmd,
        )

        target = TrajectoryPoint(
            time=0.0,
            x=self.land_x - self.x0,
            y=self.land_y - self.y0,
            z=0.0,
            vx=0.0,
            vy=0.0,
            vz=self.current_vz_cmd,
            yaw=wrapped_angle(
                self.land_yaw_unwrapped - self.yaw0
            ),
        )
        zero_z = self._zero_pid_terms()

        return self._control_result(
            target=target,
            desired=(self.land_x, self.land_y, self.z0),
            planned=(0.0, 0.0, self.current_vz_cmd),
            pid_terms=(
                self._pid_terms(self.pid_x),
                self._pid_terms(self.pid_y),
                zero_z,
            ),
            command=command,
            limited=limited,
            yaw_unwrapped=self.land_yaw_unwrapped,
        )

    def done_controller(self, x, y, z, yaw):
        target = TrajectoryPoint(
            time=0.0,
            x=x - self.x0,
            y=y - self.y0,
            z=z - self.z0,
            yaw=wrapped_angle(yaw - self.yaw0),
        )
        zeros = self._zero_pid_terms()
        return self._control_result(
            target=target,
            desired=(x, y, z),
            planned=(0.0, 0.0, 0.0),
            pid_terms=(zeros.copy(), zeros.copy(), zeros.copy()),
            command=(0.0, 0.0, 0.0),
            limited=(False, False, False),
            yaw_unwrapped=yaw,
        )

    def _build_log_row(
        self,
        count,
        wall_time,
        loop_start,
        controller_start,
        loop_dt,
        effective_dt,
        loop_lateness,
        missed_periods,
        compute_time,
        snapshot,
        control,
    ):
        target = control["target"]
        desired_x, desired_y, desired_z = control["desired"]
        planned_vx, planned_vy, planned_vz = control["planned"]
        pid_x, pid_y, pid_z = control["pid"]
        cmd_vx, cmd_vy, cmd_vz = control["command"]
        limited_x, limited_y, limited_z = control["limited"]
        target_yaw = control["yaw_cmd"]

        row = {
            "count": count,
            "wall_time": wall_time,
            "monotonic_time": loop_start,
            "elapsed_s": loop_start - controller_start,
            "phase": self.phase,
            "phase_transition": self.last_phase_transition,
            "phase_elapsed_s": (
                0.0
                if self.phase_enter_time is None
                else loop_start - self.phase_enter_time
            ),
            "mode": snapshot["mode"],
            "armed": snapshot["armed"],
            "loop_dt_s": loop_dt,
            "effective_dt_s": effective_dt,
            "loop_lateness_s": loop_lateness,
            "missed_periods": missed_periods,
            "compute_time_s": compute_time,
            "mission_time_s": self.mission_time,
            "trajectory_index": self.trajectory.index,
            "trajectory_finished": self.trajectory.finished,
            "trajectory_clock_limited": (
                self.trajectory_clock_limited
            ),
            "desired_x": desired_x,
            "x": snapshot["x"],
            "desired_y": desired_y,
            "y": snapshot["y"],
            "desired_z": desired_z,
            "z": snapshot["z"],
            "planned_vx": planned_vx,
            "pid_x_error": pid_x["error"],
            "pid_x_p": pid_x["p"],
            "pid_x_i": pid_x["i"],
            "pid_x_d": pid_x["d"],
            "pid_x_correction": pid_x["correction"],
            "cmd_vx": cmd_vx,
            "vx": snapshot["vx"],
            "planned_vy": planned_vy,
            "pid_y_error": pid_y["error"],
            "pid_y_p": pid_y["p"],
            "pid_y_i": pid_y["i"],
            "pid_y_d": pid_y["d"],
            "pid_y_correction": pid_y["correction"],
            "cmd_vy": cmd_vy,
            "vy": snapshot["vy"],
            "planned_vz": planned_vz,
            "pid_z_error": pid_z["error"],
            "pid_z_p": pid_z["p"],
            "pid_z_i": pid_z["i"],
            "pid_z_d": pid_z["d"],
            "pid_z_correction": pid_z["correction"],
            "cmd_vz": cmd_vz,
            "vz": snapshot["vz"],
            "pid_x_saturated": pid_x["saturated"],
            "pid_y_saturated": pid_y["saturated"],
            "pid_z_saturated": pid_z["saturated"],
            "command_x_limited": limited_x,
            "command_y_limited": limited_y,
            "command_z_limited": limited_z,
            "roll": snapshot["roll"],
            "pitch": snapshot["pitch"],
            "target_yaw": target_yaw,
            "target_yaw_unwrapped": control["yaw_unwrapped"],
            "yaw": snapshot["yaw"],
            "yaw_error": wrapped_angle(target_yaw - snapshot["yaw"]),
            "p": snapshot["p"],
            "q": snapshot["q"],
            "r": snapshot["r"],
            "planned_yaw_rate": target.yaw_rate,
            "planned_yaw_acceleration": target.yaw_acceleration,
            "planned_yaw_jerk": target.yaw_jerk,
            "planned_ax": target.ax,
            "planned_ay": target.ay,
            "planned_az": target.az,
            "planned_jx": target.jx,
            "planned_jy": target.jy,
            "planned_jz": target.jz,
        }
        row.update(snapshot)
        return row

    def _validate_runtime_health(self, snapshot):
        if self.receiver_error is not None:
            raise RuntimeError(
                f"MAVLink receiver failed: {self.receiver_error}"
            )
        if self.setpoint_error is not None:
            raise RuntimeError(
                f"Setpoint watchdog failed: {self.setpoint_error}"
            )

        limits = (
            ("position", snapshot["position_age_s"], self.max_position_age_s),
            ("attitude", snapshot["attitude_age_s"], self.max_attitude_age_s),
            ("heartbeat", snapshot["heartbeat_age_s"], self.max_heartbeat_age_s),
        )
        stale = [
            f"{name}={age:.3f}s>{limit:.3f}s"
            for name, age, limit in limits
            if not math.isfinite(age) or age > limit
        ]
        if stale:
            raise RuntimeError(
                "Stale runtime telemetry: " + ", ".join(stale)
            )

        if self.phase in ("TAKEOFF", "TRAJECTORY", "LAND"):
            if not snapshot["armed"]:
                raise RuntimeError(
                    f"Unexpected disarm during active phase {self.phase}"
                )
            if snapshot["heartbeat_main_mode"] != 6:
                raise RuntimeError(
                    "PX4 left OFFBOARD during active phase "
                    f"{self.phase}: main_mode="
                    f"{snapshot['heartbeat_main_mode']}"
                )

    def run(self):
        print("Starting fixed research controller loop")
        self.running = True
        self._start_setpoint_watchdog()

        controller_start = time.monotonic()
        previous_loop = controller_start
        next_tick = controller_start
        last_flush = controller_start
        last_status = -math.inf
        count = 0

        try:
            while self.running:
                loop_start = time.monotonic()

                lateness = max(0.0, loop_start - next_tick)
                missed_periods = int(lateness // self.control_dt)
                if missed_periods:
                    next_tick += missed_periods * self.control_dt
                    lateness = max(0.0, loop_start - next_tick)

                loop_dt = loop_start - previous_loop
                previous_loop = loop_start
                effective_dt = (
                    self.control_dt
                    if count == 0
                    else clamp(loop_dt, 0.001, 0.25)
                )

                snapshot = self._snapshot(loop_start)
                x, y, z = snapshot["x"], snapshot["y"], snapshot["z"]
                self._validate_runtime_health(snapshot)
                self.update_phase(x, y, z, loop_start)

                self.trajectory_clock_limited = False
                if self.phase == "TAKEOFF":
                    control = self.takeoff_controller(
                        x, y, z, snapshot["yaw"], effective_dt
                    )
                elif self.phase == "TRAJECTORY":
                    if (
                        self.last_phase_transition
                        == "TAKEOFF->TRAJECTORY"
                    ):
                        self.mission_time = 0.0
                    else:
                        trajectory_step = min(
                            loop_dt,
                            self.max_trajectory_clock_step_s,
                        )
                        self.trajectory_clock_limited = (
                            loop_dt
                            > self.max_trajectory_clock_step_s
                        )
                        self.mission_time += trajectory_step
                    control = self.trajectory_controller(
                        x, y, z, effective_dt
                    )
                elif self.phase == "LAND":
                    control = self.landing_controller(
                        x,
                        y,
                        z,
                        snapshot["yaw"],
                        effective_dt,
                    )
                else:
                    control = self.done_controller(
                        x, y, z, snapshot["yaw"]
                    )

                cmd_vx, cmd_vy, cmd_vz = control["command"]
                self.publish_velocity(
                    cmd_vx,
                    cmd_vy,
                    cmd_vz,
                    control["yaw_cmd"],
                )
                snapshot.update(
                    self._setpoint_stats(time.monotonic())
                )

                compute_time = time.monotonic() - loop_start
                row = self._build_log_row(
                    count=count,
                    wall_time=time.time(),
                    loop_start=loop_start,
                    controller_start=controller_start,
                    loop_dt=loop_dt,
                    effective_dt=effective_dt,
                    loop_lateness=lateness,
                    missed_periods=missed_periods,
                    compute_time=compute_time,
                    snapshot=snapshot,
                    control=control,
                )
                self.writer.writerow(row)

                now = time.monotonic()
                should_flush = (
                    now - last_flush >= self.log_flush_period
                    or bool(self.last_phase_transition)
                )
                if should_flush:
                    self.log_file.flush()
                    last_flush = now

                should_print = (
                    now - last_status >= self.status_period
                    or bool(self.last_phase_transition)
                )
                if should_print:
                    print(
                        f"{self.phase:12s} {snapshot['mode']:12s} "
                        f"Armed={snapshot['armed']} "
                        f"Pos=({x:.2f}, {y:.2f}, {z:.2f}) "
                        f"PlanVel=({control['planned'][0]:.2f}, "
                        f"{control['planned'][1]:.2f}, "
                        f"{control['planned'][2]:.2f}) "
                        f"CmdVel=({cmd_vx:.2f}, {cmd_vy:.2f}, "
                        f"{cmd_vz:.2f}) dt={loop_dt:.4f}"
                    )
                    last_status = now

                count += 1
                scheduled_tick = next_tick
                next_tick = scheduled_tick + self.control_dt
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nController interrupted")
            self.stop()
        except Exception as exc:
            self.worker_error = repr(exc)
            print(f"[research] Controller loop failed: {exc!r}")
            raise
        finally:
            if self.log_file is not None and not self.log_file.closed:
                self.log_file.flush()

    def stop(self):
        print("Stopping controller...")
        self.running = False
        self.setpoint_watchdog_stop.set()

        if (
            self.setpoint_thread is not None
            and self.setpoint_thread is not threading.current_thread()
        ):
            self.setpoint_thread.join(timeout=1.0)

        if self.master is not None:
            with self.state_lock:
                current_yaw = self.state.yaw
            self.send_velocity(
                0.0,
                0.0,
                0.0,
                current_yaw,
                source="direct",
            )

        if (
            self.receiver_thread is not None
            and self.receiver_thread is not threading.current_thread()
        ):
            self.receiver_thread.join(timeout=1.0)

        if self.log_file is not None and not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()

        with self.state_lock:
            counts = dict(sorted(self.state.message_counts.items()))
        print(f"[research] MAVLink message counts: {counts}")
        print("Controller stopped.")


def main():
    controller = PositionController()
    controller.connect()
    controller.start_receiver()
    time.sleep(1.0)
    controller.initialize_target()
    controller.setup_logger()
    controller.run()


if __name__ == "__main__":
    main()
