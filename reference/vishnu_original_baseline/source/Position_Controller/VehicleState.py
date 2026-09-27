from pymavlink import mavutil


class VehicleState:
    """
    Latest measured state received from PX4.
    """

    def __init__(self):

        # -----------------------------
        # Position (LOCAL_POSITION_NED)
        # -----------------------------
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # -----------------------------
        # Velocity (LOCAL_POSITION_NED)
        # -----------------------------
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # -----------------------------
        # Attitude (ATTITUDE)
        # -----------------------------
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Optional if ATTITUDE message provides them
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

        # -----------------------------
        # Flight status
        # -----------------------------
        self.mode = "UNKNOWN"
        self.armed = False

        # -----------------------------
        # Message flags
        # -----------------------------
        self.position_received = False
        self.attitude_received = False
        self.heartbeat_received = False

    def update_position(self, msg):

        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

        self.position_received = True

    def update_attitude(self, msg):

        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = msg.yaw

        self.roll_rate = msg.rollspeed
        self.pitch_rate = msg.pitchspeed
        self.yaw_rate = msg.yawspeed

        self.attitude_received = True

    def update_heartbeat(self, msg):

        self.mode = mavutil.mode_string_v10(msg)

        self.armed = bool(
            msg.base_mode &
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        self.heartbeat_received = True