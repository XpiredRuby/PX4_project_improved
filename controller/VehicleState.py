import math
import time

from pymavlink import mavutil


class VehicleState:
    """Latest PX4 telemetry plus freshness and message counters."""

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0

        self.roll = self.pitch = self.yaw = 0.0
        self.roll_rate = self.pitch_rate = self.yaw_rate = 0.0

        self.mode = "UNKNOWN"
        self.armed = False
        self.heartbeat_custom_mode = 0
        self.heartbeat_main_mode = 0
        self.heartbeat_sub_mode = 0

        self.position_received = False
        self.attitude_received = False
        self.heartbeat_received = False
        self.imu_received = False
        self.position_target_received = False
        self.attitude_target_received = False
        self.actuator_received = False
        self.servo_received = False
        self.gps_received = False
        self.extended_state_received = False
        self.estimator_received = False
        self.command_ack_received = False

        self.position_received_at = None
        self.attitude_received_at = None
        self.heartbeat_received_at = None
        self.imu_received_at = None
        self.position_target_received_at = None
        self.attitude_target_received_at = None
        self.actuator_received_at = None
        self.servo_received_at = None
        self.gps_received_at = None
        self.extended_state_received_at = None
        self.estimator_received_at = None
        self.command_ack_received_at = None

        self.position_time_boot_ms = math.nan
        self.attitude_time_boot_ms = math.nan

        self.imu_xacc = self.imu_yacc = self.imu_zacc = math.nan
        self.imu_xgyro = self.imu_ygyro = self.imu_zgyro = math.nan

        self.px4_target_x = self.px4_target_y = self.px4_target_z = math.nan
        self.px4_target_vx = self.px4_target_vy = self.px4_target_vz = math.nan
        self.px4_target_ax = self.px4_target_ay = self.px4_target_az = math.nan
        self.px4_target_yaw = self.px4_target_yaw_rate = math.nan

        self.attitude_target_roll_rate = math.nan
        self.attitude_target_pitch_rate = math.nan
        self.attitude_target_yaw_rate = math.nan
        self.attitude_target_thrust = math.nan
        self.attitude_target_q = [math.nan] * 4

        # Both streams contain PX4 actuator_outputs data in natural output
        # units. Keep them separate so message timing is not conflated.
        self.actuator_outputs = [math.nan] * 16
        self.servo_outputs = [math.nan] * 16

        self.gps_fix_type = 0
        self.gps_satellites_visible = 0
        self.gps_hdop = math.inf
        self.gps_vdop = math.inf
        self.gps_horizontal_accuracy_m = math.inf
        self.gps_vertical_accuracy_m = math.inf
        self.gps_lat_deg = math.nan
        self.gps_lon_deg = math.nan
        self.gps_alt_m = math.nan

        self.landed_state = mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED

        self.estimator_flags = 0
        self.estimator_velocity_ratio = math.inf
        self.estimator_pos_horiz_ratio = math.inf
        self.estimator_pos_vert_ratio = math.inf
        self.estimator_mag_ratio = math.inf
        self.estimator_hagl_ratio = math.inf

        self.command_ack_command = -1
        self.command_ack_result = -1
        self.command_ack_progress = -1
        self.command_ack_by_command = {}
        self.message_counts = {}

    @staticmethod
    def _now():
        return time.monotonic()

    def note_message(self, msg_type):
        self.message_counts[msg_type] = self.message_counts.get(msg_type, 0) + 1

    def update_position(self, msg):
        self.x, self.y, self.z = msg.x, msg.y, msg.z
        self.vx, self.vy, self.vz = msg.vx, msg.vy, msg.vz
        self.position_time_boot_ms = getattr(msg, "time_boot_ms", math.nan)
        self.position_received = True
        self.position_received_at = self._now()

    def update_attitude(self, msg):
        self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
        self.roll_rate = msg.rollspeed
        self.pitch_rate = msg.pitchspeed
        self.yaw_rate = msg.yawspeed
        self.attitude_time_boot_ms = getattr(msg, "time_boot_ms", math.nan)
        self.attitude_received = True
        self.attitude_received_at = self._now()

    def update_heartbeat(self, msg):
        self.mode = mavutil.mode_string_v10(msg)
        self.armed = bool(
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        self.heartbeat_custom_mode = int(msg.custom_mode)
        self.heartbeat_main_mode = (
            self.heartbeat_custom_mode >> 16
        ) & 0xFF
        self.heartbeat_sub_mode = (
            self.heartbeat_custom_mode >> 24
        ) & 0xFF
        self.heartbeat_received = True
        self.heartbeat_received_at = self._now()

    def update_highres_imu(self, msg):
        self.imu_xacc, self.imu_yacc, self.imu_zacc = (
            msg.xacc,
            msg.yacc,
            msg.zacc,
        )
        self.imu_xgyro, self.imu_ygyro, self.imu_zgyro = (
            msg.xgyro,
            msg.ygyro,
            msg.zgyro,
        )
        self.imu_received = True
        self.imu_received_at = self._now()

    def update_position_target(self, msg):
        self.px4_target_x, self.px4_target_y, self.px4_target_z = (
            msg.x,
            msg.y,
            msg.z,
        )
        self.px4_target_vx, self.px4_target_vy, self.px4_target_vz = (
            msg.vx,
            msg.vy,
            msg.vz,
        )
        self.px4_target_ax, self.px4_target_ay, self.px4_target_az = (
            msg.afx,
            msg.afy,
            msg.afz,
        )
        self.px4_target_yaw = msg.yaw
        self.px4_target_yaw_rate = msg.yaw_rate
        self.position_target_received = True
        self.position_target_received_at = self._now()

    def update_attitude_target(self, msg):
        self.attitude_target_roll_rate = msg.body_roll_rate
        self.attitude_target_pitch_rate = msg.body_pitch_rate
        self.attitude_target_yaw_rate = msg.body_yaw_rate
        self.attitude_target_q = list(msg.q)

        thrust = getattr(msg, "thrust", math.nan)
        if isinstance(thrust, (list, tuple)):
            thrust = math.sqrt(sum(float(v) ** 2 for v in thrust))
        self.attitude_target_thrust = float(thrust)

        self.attitude_target_received = True
        self.attitude_target_received_at = self._now()

    def update_actuator_output_status(self, msg):
        values = list(getattr(msg, "actuator", []))
        if values:
            self.actuator_outputs = (
                [float(v) for v in values[:16]]
                + [math.nan] * max(0, 16 - len(values))
            )
            self.actuator_received = True
            self.actuator_received_at = self._now()

    def update_servo_output_raw(self, msg):
        values = [
            getattr(msg, f"servo{i}_raw", math.nan)
            for i in range(1, 17)
        ]
        if any(math.isfinite(float(v)) for v in values):
            self.servo_outputs = [float(v) for v in values]
            self.servo_received = True
            self.servo_received_at = self._now()

    @staticmethod
    def _gps_dilution(raw_value):
        raw_value = int(raw_value)
        return math.inf if raw_value >= 65535 else raw_value / 100.0

    @staticmethod
    def _gps_uncertainty(raw_value):
        raw_value = int(raw_value)
        return math.inf if raw_value >= 0xFFFFFFFF else raw_value / 1000.0

    def update_gps_raw_int(self, msg):
        self.gps_fix_type = int(msg.fix_type)
        satellites = int(msg.satellites_visible)
        self.gps_satellites_visible = 0 if satellites >= 255 else satellites
        self.gps_hdop = self._gps_dilution(msg.eph)
        self.gps_vdop = self._gps_dilution(msg.epv)
        self.gps_horizontal_accuracy_m = self._gps_uncertainty(
            getattr(msg, "h_acc", 0xFFFFFFFF)
        )
        self.gps_vertical_accuracy_m = self._gps_uncertainty(
            getattr(msg, "v_acc", 0xFFFFFFFF)
        )
        self.gps_lat_deg = float(msg.lat) / 1e7
        self.gps_lon_deg = float(msg.lon) / 1e7
        self.gps_alt_m = float(msg.alt) / 1000.0
        self.gps_received = True
        self.gps_received_at = self._now()

    def update_extended_sys_state(self, msg):
        self.landed_state = int(msg.landed_state)
        self.extended_state_received = True
        self.extended_state_received_at = self._now()

    def update_estimator_status(self, msg):
        self.estimator_flags = int(msg.flags)
        self.estimator_velocity_ratio = float(msg.vel_ratio)
        self.estimator_pos_horiz_ratio = float(msg.pos_horiz_ratio)
        self.estimator_pos_vert_ratio = float(msg.pos_vert_ratio)
        self.estimator_mag_ratio = float(msg.mag_ratio)
        self.estimator_hagl_ratio = float(msg.hagl_ratio)
        self.estimator_received = True
        self.estimator_received_at = self._now()

    def update_command_ack(self, msg):
        self.command_ack_command = int(msg.command)
        self.command_ack_result = int(msg.result)
        self.command_ack_progress = int(getattr(msg, "progress", -1))
        self.command_ack_by_command[self.command_ack_command] = (
            self.message_counts.get("COMMAND_ACK", 0),
            self.command_ack_result,
            self.command_ack_progress,
        )
        self.command_ack_received = True
        self.command_ack_received_at = self._now()
