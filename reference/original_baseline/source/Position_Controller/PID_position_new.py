#!/usr/bin/env python3

import time
import csv
import math
import threading

from pymavlink import mavutil

from PID_Controller import PIDController
from VehicleState import VehicleState
from trajectory import Trajectory, TrajectoryPoint


class PositionController:

    def __init__(self):

        self.connection_string = "udp:127.0.0.1:14540"    # "/dev/ttyACM0"
        self.control_rate = 20.0
        self.control_dt = 1.0 / self.control_rate
        self.master = None
        self.state = VehicleState()
        self.state_lock = threading.Lock()
        self.running = False
        self.receiver_thread = None

        self.pid_x = PIDController(Kp=0.8, Ki=0.0, Kd=0.0, output_limits=(-3.0, 3.0))
        self.pid_y = PIDController(Kp=0.8, Ki=0.0, Kd=0.0, output_limits=(-3.0, 3.0))
        self.pid_z = PIDController(Kp=1.0, Ki=0.0, Kd=0.0, output_limits=(-1.0, 1.0))

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        self.phase = "TAKEOFF"
        self.trajectory_start_time = None
        self.mission_time = 0.0
        self.takeoff_altitude = -8.0
        self.takeoff_start_z = None
        self.takeoff_x = None
        self.takeoff_y = None
        self.max_climb_rate = 0.5
        self.takeoff_tolerance = 0.1

        self.land_start = None
        self.max_descent_rate = 0.5
        self.descent_accel = 0.3
        self.current_vz_cmd = 0.0
        self.land_slow_altitude = -0.2
        self.ground_z = -0.03

        self.log_file = None
        self.writer = None
        self.filename = None

        self.trajectory = Trajectory("trajectory.csv")
        self.duration = self.trajectory.duration

    def connect(self):
        print(f"Connecting to {self.connection_string}...")

        self.master = mavutil.mavlink_connection(
            self.connection_string
        )

        self.master.wait_heartbeat()

        print(
            f"Connected! "
            f"(System {self.master.target_system}, "
            f"Component {self.master.target_component})"
        )

    def start_receiver(self):
        self.running = True
        self.receiver_thread = threading.Thread(
            target=self.mavlink_receiver,
            daemon=True
        )
        self.receiver_thread.start()
        print("Started MAVLink receiver thread.")

    def mavlink_receiver(self):
        while self.running:
            msg = self.master.recv_match(blocking=True)
            if msg is None:
                continue
            msg_type = msg.get_type()
            with self.state_lock:
                if msg_type == "LOCAL_POSITION_NED":
                    self.state.update_position(msg)
                elif msg_type == "ATTITUDE":
                    self.state.update_attitude(msg)
                elif msg_type == "HEARTBEAT":
                    self.state.update_heartbeat(msg)

    def send_velocity(self, vx, vy, vz, yaw):
        self.master.mav.set_position_target_local_ned_send(
            0,  # int(time.time() * 1000),
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            # Use velocity + yaw
            # Ignore: position, acceleration, yaw_rate
            # (bit 10 / yaw is CLEARED so yaw is applied)
            0b0000101111000111,
            # position
            0, 0, 0,
            # velocity
            vx, vy, vz,
            # acceleration
            0, 0, 0,
            # yaw
            yaw,
            # yaw rate
            0
        )

    def initialize_target(self):
        print("Waiting for LOCAL_POSITION_NED...")

        while not self.state.position_received:
            time.sleep(0.01)

        with self.state_lock:
            self.x0 = self.state.x
            self.y0 = self.state.y
            self.z0 = self.state.z
            self.yaw0 = self.state.yaw

        print(
            f"Origin: "
            f"x={self.x0:.2f} "
            f"y={self.y0:.2f} "
            f"z={self.z0:.2f}"
        )

        # -------------------------------------------------
        # Target position
        # -------------------------------------------------
        self.target_x = self.x0
        self.target_y = self.y0
        self.target_z = self.z0 + self.takeoff_altitude

        self.pid_x.setpoint = self.target_x
        self.pid_y.setpoint = self.target_y
        self.pid_z.setpoint = self.target_z

        print(
            f"Target: "
            f"x={self.target_x:.2f} "
            f"y={self.target_y:.2f} "
            f"z={self.target_z:.2f}"
        )

    def setup_logger(self):
        self.filename = f"pid_log_{int(time.time())}.csv"

        self.log_file = open(
            self.filename,
            "w",
            newline=""
        )

        self.writer = csv.writer(self.log_file)

        self.writer.writerow([
            "count",
            "time",
            "phase",
            "mode",
            "armed",
            "dt",
            "target_x",
            "x",
            "target_y",
            "y",
            "target_z",
            "z",
            "cmd_vx",
            "vx",
            "cmd_vy",
            "vy",
            "cmd_vz",
            "vz",
            "roll",
            "pitch",
            "target_yaw",
            "yaw"
        ])

        print(f"Logging: {self.filename}")

    def takeoff_controller(self, x, y, z, yaw):
        """
        Takeoff controller:
        - Hold current XY position
        - Climb to target altitude
        - Generate vertical velocity command
        """
        if self.takeoff_start_z is None:
            self.takeoff_start_z = z
            self.takeoff_x = x
            self.takeoff_y = y

            print(
                f"Takeoff start: "
                f"x={x:.2f}, "
                f"y={y:.2f}, "
                f"z={z:.2f}"
            )

        # Hold XY
        vx_cmd = 0.0
        vy_cmd = 0.0

        # Altitude PID
        vz_cmd = self.pid_z.update(
            z,
            self.control_dt
        )

        vz_cmd = max(
            min(vz_cmd, self.max_climb_rate),
            -self.max_climb_rate
        )

        target = TrajectoryPoint(
            time=time.time(),
            x=self.takeoff_x,
            y=self.takeoff_y,
            z=self.takeoff_altitude,
            vx=0.0,
            vy=0.0,
            vz=vz_cmd,
            yaw=yaw
        )

        if abs(self.takeoff_altitude - z) < self.takeoff_tolerance:
            print("Takeoff complete")

            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()

            self.trajectory_start_time = time.time()
            self.phase = "TRAJECTORY"

        return (
            vx_cmd,
            vy_cmd,
            vz_cmd,
            target
        )

    def trajectory_controller(self, x, y, z):

        target = self.trajectory.get_target(
            self.mission_time
        )

        self.pid_x.setpoint = self.x0 + target.x
        self.pid_y.setpoint = self.y0 + target.y
        self.pid_z.setpoint = self.z0 + target.z

        vx_cmd = (
            target.vx +
            self.pid_x.update(
                x,
                self.control_dt
            )
        )

        vy_cmd = (
            target.vy +
            self.pid_y.update(
                y,
                self.control_dt
            )
        )

        vz_cmd = (
            target.vz +
            self.pid_z.update(
                z,
                self.control_dt
            )
        )
        return (
            vx_cmd,
            vy_cmd,
            vz_cmd,
            target
        )

    def landing_controller(self, z, yaw):
        """
        Landing controller:
        - Hold XY velocity at zero
        - Descend with smooth vertical velocity
        - Slow down near ground
        """
        target_vz = self.max_descent_rate

        max_change = (
            self.descent_accel *
            self.control_dt
        )

        if self.current_vz_cmd < target_vz:
            self.current_vz_cmd += max_change
            if self.current_vz_cmd > target_vz:
                self.current_vz_cmd = target_vz

        if (z - self.z0) > self.land_slow_altitude:
            scale = (
                -(z - self.z0)
                / (-self.land_slow_altitude)
            )
            scale = max(min(scale, 1.0), 0.0)
            self.current_vz_cmd = (
                self.max_descent_rate * scale
            )

        if (z - self.z0) >= -0.05:
            print("Landing complete")
            self.current_vz_cmd = 0.0
            self.phase = "DONE"

        target = TrajectoryPoint(
            time=time.time(),
            x=self.state.x,
            y=self.state.y,
            z=z,
            vx=0.0,
            vy=0.0,
            vz=self.current_vz_cmd,
            yaw=yaw
        )

        return (
            0.0,
            0.0,
            self.current_vz_cmd,
            target
        )

    def update_phase(self, x, y, z):
        """
        Handles transitions between flight phases.
        """
        # Takeoff -> Trajectory
        if self.phase == "TAKEOFF":
            if abs(self.target_z - z) < 0.1:
                print("Reached takeoff altitude")
                self.phase = "TRAJECTORY"
                self.trajectory_start_time = time.time()
                self.mission_time = 0.0
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()

        # Trajectory -> Landing
        elif self.phase == "TRAJECTORY":
            if self.trajectory.finished:
                print("Trajectory complete")
                self.phase = "LAND"
                self.current_vz_cmd = 0.0

        # Landing -> Done
        elif self.phase == "LAND":
            if (z - self.z0) >= self.ground_z:
                print("Landing complete")
                self.phase = "DONE"

    def run(self):
        """
        Main position controller loop.
        """
        print("Starting controller loop")

        self.running = True

        prev_time = time.time()
        count = 0

        try:
            while self.running:

                now = time.time()
                dt = now - prev_time
                prev_time = now

                with self.state_lock:
                    x = self.state.x
                    y = self.state.y
                    z = self.state.z
                    vx = self.state.vx
                    vy = self.state.vy
                    vz = self.state.vz
                    roll = self.state.roll
                    pitch = self.state.pitch
                    yaw = self.state.yaw
                    mode = self.state.mode
                    armed = self.state.armed

                self.update_phase(x, y, z)

                if self.phase == "TAKEOFF":
                    vx_cmd, vy_cmd, vz_cmd, target = self.takeoff_controller(
                        x, y, z, yaw
                    )

                elif self.phase == "TRAJECTORY":

                    self.mission_time = (
                        time.time() -
                        self.trajectory_start_time
                    )

                    vx_cmd, vy_cmd, vz_cmd, target = self.trajectory_controller(
                        x, y, z
                    )

                elif self.phase == "LAND":

                    vx_cmd, vy_cmd, vz_cmd, target = self.landing_controller(
                        z, yaw
                    )

                elif self.phase == "DONE":

                    vx_cmd = 0.0
                    vy_cmd = 0.0
                    vz_cmd = 0.0

                    target = TrajectoryPoint(
                        time=now,
                        x=x,
                        y=y,
                        z=z,
                        vx=0.0,
                        vy=0.0,
                        vz=0.0,
                        yaw=yaw
                    )

                # -----------------------------------------
                # Yaw command
                #   TRAJECTORY -> follow trajectory yaw
                #   other      -> hold initial yaw
                # -----------------------------------------
                if self.phase == "TAKEOFF":
                    yaw_cmd = self.yaw0
                elif self.phase == "TRAJECTORY":
                    yaw_cmd = self.yaw0 + target.yaw
                else:
                    yaw_cmd = yaw

                # Normalize to [-pi, pi]
                yaw_cmd = math.atan2(
                    math.sin(yaw_cmd),
                    math.cos(yaw_cmd)
                )

                # -----------------------------------------
                # Send velocity command
                # -----------------------------------------
                self.send_velocity(
                    vx_cmd,
                    vy_cmd,
                    vz_cmd,
                    yaw_cmd
                )

                # -----------------------------------------
                # Logging
                # -----------------------------------------
                self.writer.writerow([
                    count,
                    now,
                    self.phase,
                    mode,
                    armed,
                    dt,

                    # Position target
                    target.x + self.x0,
                    x,
                    target.y + self.y0,
                    y,
                    target.z + self.z0,
                    z,

                    # Velocity target
                    target.vx,
                    vx,
                    target.vy,
                    vy,
                    target.vz,
                    vz,

                    # Attitude
                    roll,
                    pitch,
                    yaw_cmd,
                    yaw
                ])

                self.log_file.flush()

                # -----------------------------------------
                # Print status
                # -----------------------------------------
                print(
                    f"{self.phase:12s} "
                    f"{mode:12s} "
                    f"Armed={armed} "
                    f"Pos=({x:.2f}, {y:.2f}, {z:.2f}) "
                    f"CmdVel=({vx_cmd:.2f}, "
                    f"{vy_cmd:.2f}, "
                    f"{vz_cmd:.2f})"
                )

                count += 1

                # -----------------------------------------
                # Maintain control rate
                # -----------------------------------------
                sleep_time = self.control_dt - (time.time() - now)

                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nController interrupted")
            self.stop()

    def stop(self):
        """
        Safely stop the controller.
        """
        print("Stopping controller...")

        # Stop loops
        self.running = False

        # Stop vehicle motion
        if self.master is not None:
            with self.state_lock:
                current_yaw = self.state.yaw
            self.send_velocity(
                0.0,
                0.0,
                0.0,
                current_yaw
            )

        # Wait for receiver thread to finish
        if self.receiver_thread is not None:
            self.receiver_thread.join(timeout=1.0)

        # Close logger
        if self.log_file is not None:
            self.log_file.close()

        print("Controller stopped.")


def main():
    controller = PositionController()
    controller.connect()
    controller.start_receiver()
    time.sleep(1)
    controller.initialize_target()
    controller.setup_logger()
    controller.run()


if __name__ == "__main__":
    main()