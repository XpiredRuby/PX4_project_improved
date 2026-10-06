#!/usr/bin/env python3
"""Small PX4 SITL mission controller derived from Vishnu's baseline."""

from __future__ import annotations

import argparse
import csv
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:  # Logic tests do not require a MAVLink installation.
    mavutil = None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def limit_xy(vx: float, vy: float, limit: float) -> tuple[float, float]:
    speed = math.hypot(vx, vy)
    if speed <= limit or speed == 0.0:
        return vx, vy
    scale = limit / speed
    return vx * scale, vy * scale


@dataclass(frozen=True)
class MissionConfig:
    connection: str = "udp:127.0.0.1:14540"
    control_hz: float = 10.0
    timing_tolerance: float = 0.10
    run_length_m: float = 200.0
    cruise_speed_m_s: float = 5.0
    takeoff_height_m: float = 8.0
    slow_descent_height_m: float = 5.0
    slow_descent_m_s: float = 0.1
    fast_descent_m_s: float = 0.5
    horizontal_accel_m_s2: float = 1.0
    position_kp: float = 0.8
    altitude_kp: float = 1.0
    takeoff_speed_m_s: float = 0.5
    position_tolerance_m: float = 0.5
    speed_tolerance_m_s: float = 0.3
    telemetry_timeout_s: float = 0.5
    prestream_s: float = 2.0
    mission_timeout_s: float = 240.0

    @property
    def period_s(self) -> float:
        return 1.0 / self.control_hz

    def validate(self) -> None:
        values = vars(self)
        for name, value in values.items():
            if name == "connection":
                continue
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.timing_tolerance >= 1.0:
            raise ValueError("timing_tolerance must be less than 1")
        if self.slow_descent_m_s >= self.fast_descent_m_s:
            raise ValueError("slow descent must be slower than fast descent")


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    landed_state: int | None = None
    position_time: float = 0.0
    attitude_time: float = 0.0
    heartbeat_time: float = 0.0
    landed_time: float = 0.0

    def ready(self, now: float, timeout: float) -> bool:
        fast = (self.position_time, self.attitude_time)
        slow = (self.heartbeat_time, self.landed_time)
        return (
            all(stamp > 0.0 and 0.0 <= now - stamp <= timeout for stamp in fast)
            and all(stamp > 0.0 and 0.0 <= now - stamp <= 1.5 for stamp in slow)
        )


@dataclass(frozen=True)
class Command:
    vx: float
    vy: float
    vz: float
    yaw: float
    target_x: float
    target_y: float
    target_z: float


class LoopTiming:
    """Measure whether the 10 Hz loop remains within its 10% period band."""

    def __init__(self, period_s: float, tolerance: float):
        self.period_s = period_s
        self.tolerance = tolerance
        self.samples = 0
        self.violations = 0
        self.consecutive_violations = 0

    @property
    def limits(self) -> tuple[float, float]:
        return (
            self.period_s * (1.0 - self.tolerance),
            self.period_s * (1.0 + self.tolerance),
        )

    def add(self, dt: float) -> bool:
        low, high = self.limits
        accepted = low - 1e-12 <= dt <= high + 1e-12
        self.samples += 1
        if accepted:
            self.consecutive_violations = 0
        else:
            self.violations += 1
            self.consecutive_violations += 1
        return accepted


class StraightTrajectory:
    """A 200 m reference with bounded acceleration and a 5 m/s speed cap."""

    def __init__(self, config: MissionConfig):
        self.config = config
        self.distance_m = 0.0
        self.speed_m_s = 0.0

    def step(self, dt: float) -> tuple[float, float]:
        remaining = max(0.0, self.config.run_length_m - self.distance_m)
        stopping_speed = math.sqrt(2.0 * self.config.horizontal_accel_m_s2 * remaining)
        goal_speed = min(self.config.cruise_speed_m_s, stopping_speed)
        change = self.config.horizontal_accel_m_s2 * dt
        self.speed_m_s += clamp(goal_speed - self.speed_m_s, -change, change)
        self.distance_m = min(
            self.config.run_length_m,
            self.distance_m + self.speed_m_s * dt,
        )
        if self.distance_m >= self.config.run_length_m:
            self.speed_m_s = 0.0
        return self.distance_m, self.speed_m_s


class MissionLogic:
    """Takeoff, trajectory following, and known-height landing state machine."""

    def __init__(self, config: MissionConfig, state: VehicleState, heading_deg: float):
        config.validate()
        self.config = config
        self.start_x = state.x
        self.start_y = state.y
        self.ground_z = state.z
        self.yaw = state.yaw
        heading = math.radians(heading_deg)
        self.north = math.cos(heading)
        self.east = math.sin(heading)
        self.trajectory = StraightTrajectory(config)
        self.phase = "TAKEOFF"

    def altitude_agl(self, state: VehicleState) -> float:
        return self.ground_z - state.z

    def _hold_xy(self, state: VehicleState, x: float, y: float, cap: float) -> tuple[float, float]:
        vx = self.config.position_kp * (x - state.x)
        vy = self.config.position_kp * (y - state.y)
        return limit_xy(vx, vy, cap)

    def command(self, state: VehicleState, dt: float) -> Command:
        target_z = self.ground_z - self.config.takeoff_height_m
        end_x = self.start_x + self.north * self.config.run_length_m
        end_y = self.start_y + self.east * self.config.run_length_m

        if self.phase == "TAKEOFF":
            vx, vy = self._hold_xy(state, self.start_x, self.start_y, 1.0)
            vz = clamp(
                self.config.altitude_kp * (target_z - state.z),
                -self.config.takeoff_speed_m_s,
                self.config.takeoff_speed_m_s,
            )
            if (
                abs(target_z - state.z) <= 0.2
                and abs(state.vz) <= self.config.speed_tolerance_m_s
            ):
                self.phase = "TRAJECTORY"
            return Command(vx, vy, vz, self.yaw, self.start_x, self.start_y, target_z)

        if self.phase == "TRAJECTORY":
            distance, reference_speed = self.trajectory.step(dt)
            target_x = self.start_x + self.north * distance
            target_y = self.start_y + self.east * distance
            vx = self.north * reference_speed + self.config.position_kp * (target_x - state.x)
            vy = self.east * reference_speed + self.config.position_kp * (target_y - state.y)
            vx, vy = limit_xy(vx, vy, self.config.cruise_speed_m_s)
            vz = clamp(
                self.config.altitude_kp * (target_z - state.z),
                -self.config.takeoff_speed_m_s,
                self.config.takeoff_speed_m_s,
            )
            error = math.hypot(end_x - state.x, end_y - state.y)
            speed = math.hypot(state.vx, state.vy)
            if (
                self.trajectory.distance_m >= self.config.run_length_m
                and error <= self.config.position_tolerance_m
                and speed <= self.config.speed_tolerance_m_s
            ):
                self.phase = "LAND_FAST"
            return Command(vx, vy, vz, self.yaw, target_x, target_y, target_z)

        vx, vy = self._hold_xy(state, end_x, end_y, 1.0)
        agl = self.altitude_agl(state)
        if agl <= self.config.slow_descent_height_m:
            self.phase = "LAND_SLOW"
        if state.landed_state == 1:
            self.phase = "DONE"
        descent = (
            0.0
            if self.phase == "DONE"
            else self.config.slow_descent_m_s
            if self.phase == "LAND_SLOW"
            else self.config.fast_descent_m_s
        )
        return Command(vx, vy, descent, self.yaw, end_x, end_y, self.ground_z)


class PX4Mission:
    def __init__(self, config: MissionConfig, heading_deg: float, log_path: Path):
        if mavutil is None:
            raise RuntimeError("Install requirements.txt before running the mission")
        self.config = config
        self.heading_deg = heading_deg
        self.log_path = log_path
        self.state = VehicleState()
        self.lock = threading.Lock()
        self.running = False
        self.master = None
        self.receiver = None

    def connect(self) -> None:
        print(f"Connecting to {self.config.connection}")
        self.master = mavutil.mavlink_connection(self.config.connection)
        self.master.wait_heartbeat(timeout=10)
        self.running = True
        self.receiver = threading.Thread(target=self._receive, daemon=True)
        self.receiver.start()
        self._request_messages()

    def _request_messages(self) -> None:
        message_ids = (
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
        )
        for message_id in message_ids:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                int(self.config.period_s * 1_000_000),
                0,
                0,
                0,
                0,
                0,
            )

    def _receive(self) -> None:
        while self.running:
            message = self.master.recv_match(blocking=True, timeout=0.2)
            if message is None:
                continue
            now = time.monotonic()
            with self.lock:
                kind = message.get_type()
                if kind == "LOCAL_POSITION_NED":
                    self.state.x, self.state.y, self.state.z = message.x, message.y, message.z
                    self.state.vx, self.state.vy, self.state.vz = message.vx, message.vy, message.vz
                    self.state.position_time = now
                elif kind == "ATTITUDE":
                    self.state.yaw = message.yaw
                    self.state.attitude_time = now
                elif kind == "HEARTBEAT":
                    self.state.mode = mavutil.mode_string_v10(message)
                    self.state.armed = bool(
                        message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    self.state.heartbeat_time = now
                elif kind == "EXTENDED_SYS_STATE":
                    self.state.landed_state = message.landed_state
                    self.state.landed_time = now

    def snapshot(self) -> VehicleState:
        with self.lock:
            return VehicleState(**vars(self.state))

    def wait_ready(self, timeout_s: float = 10.0) -> VehicleState:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            state = self.snapshot()
            if state.ready(time.monotonic(), self.config.telemetry_timeout_s):
                if state.armed:
                    raise RuntimeError("Vehicle must be disarmed during initialization")
                if state.landed_state != 1:
                    raise RuntimeError("Vehicle must report ON_GROUND during initialization")
                return state
            time.sleep(0.05)
        raise TimeoutError("Fresh position, attitude, heartbeat, and landed state not received")

    def send_velocity(self, command: Command) -> None:
        self.master.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            2503,
            0,
            0,
            0,
            command.vx,
            command.vy,
            command.vz,
            0,
            0,
            0,
            command.yaw,
            0,
        )

    def request_mode(self, custom_mode: int) -> None:
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode,
        )

    def wait_mode(self, name: str, custom_mode: int, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.request_mode(custom_mode)
            if name in self.snapshot().mode.upper():
                return
            time.sleep(self.config.period_s)
        raise TimeoutError(f"PX4 did not enter {name}")

    def arm(self) -> None:
        state = self.snapshot()
        if state.armed or state.landed_state != 1:
            raise RuntimeError("Arming requires fresh disarmed ON_GROUND state")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.snapshot().armed:
                return
            time.sleep(0.05)
        raise TimeoutError("PX4 did not arm")

    def disarm_on_ground(self) -> None:
        state = self.snapshot()
        if state.landed_state != 1:
            raise RuntimeError("Refusing to disarm without ON_GROUND confirmation")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def run(self) -> None:
        self.connect()
        initial = self.wait_ready()
        logic = MissionLogic(self.config, initial, self.heading_deg)
        zero = Command(0.0, 0.0, 0.0, initial.yaw, initial.x, initial.y, initial.z)
        prestream_end = time.monotonic() + self.config.prestream_s
        while time.monotonic() < prestream_end:
            self.send_velocity(zero)
            time.sleep(self.config.period_s)
        self.wait_mode("OFFBOARD", 6 << 16)
        self.arm()

        timing = LoopTiming(self.config.period_s, self.config.timing_tolerance)
        start = previous = time.monotonic()
        deadline = start
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ("time_s", "dt_s", "phase", "x", "y", "z", "vx_cmd", "vy_cmd", "vz_cmd")
            )
            while logic.phase != "DONE":
                now = time.monotonic()
                dt = self.config.period_s if now == start else now - previous
                previous = now
                state = self.snapshot()
                if not state.ready(now, self.config.telemetry_timeout_s):
                    raise RuntimeError("Telemetry became stale")
                if "OFFBOARD" not in state.mode.upper():
                    raise RuntimeError("PX4 left OFFBOARD mode")
                if now - start > self.config.mission_timeout_s:
                    raise TimeoutError("Mission timeout")
                if timing.samples and not timing.add(dt):
                    print(f"Timing warning: dt={dt:.4f}s, expected {timing.limits}")
                elif not timing.samples:
                    timing.add(dt)
                if timing.consecutive_violations >= 3:
                    raise RuntimeError("10 Hz loop exceeded its 10% timing band three times")
                command = logic.command(state, dt)
                self.send_velocity(command)
                writer.writerow(
                    (now - start, dt, logic.phase, state.x, state.y, state.z,
                     command.vx, command.vy, command.vz)
                )
                if timing.samples % 10 == 0:
                    stream.flush()
                    print(
                        f"{logic.phase:10s} x={state.x:7.2f} "
                        f"z={state.z:6.2f} vz={command.vz:4.2f}"
                    )
                deadline += self.config.period_s
                delay = deadline - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                else:
                    deadline = time.monotonic()
        self.send_velocity(zero)
        self.disarm_on_ground()
        print(f"Mission complete. Timing violations: {timing.violations}/{timing.samples}")

    def land_after_failure(self) -> None:
        if self.master is not None and self.snapshot().armed:
            print("Failure detected. Requesting PX4 AUTO.LAND.")
            for _ in range(20):
                self.request_mode((6 << 24) | (4 << 16))
                if "LAND" in self.snapshot().mode.upper():
                    break
                time.sleep(0.1)

    def stop(self) -> None:
        self.running = False
        if self.receiver is not None:
            self.receiver.join(timeout=1.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the simple PX4 SITL mission")
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--heading-deg", type=float, default=0.0, help="0=north, 90=east")
    parser.add_argument("--log", type=Path, default=Path("flight_log.csv"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    mission = PX4Mission(MissionConfig(connection=args.connection), args.heading_deg, args.log)
    try:
        mission.run()
    except BaseException:
        mission.land_after_failure()
        raise
    finally:
        mission.stop()


if __name__ == "__main__":
    main()
