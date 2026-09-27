#!/usr/bin/env python3

import math
import time

from PID_position_new import PositionController, clamp, wrapped_angle
from trajectory import TrajectoryPoint


class RandomizedPositionController(PositionController):
    """Airborne-start controller with an explicit, launcher-supplied ground model.

    PX4 LOCAL_POSITION_NED uses positive Z down.  If the estimator reports the
    randomized airborne start at ``airborne_z`` and the launcher supplied a
    positive height ``spawn_height_m``, the predicted ground is therefore
    ``airborne_z + spawn_height_m``.
    """

    def __init__(self, spawn_height_m, ground_reference=None, spawn_pose=None):
        super().__init__()
        if not math.isfinite(spawn_height_m) or spawn_height_m <= 0.0:
            raise ValueError("spawn_height_m must be finite and positive")

        self.spawn_height_m = float(spawn_height_m)
        self.ground_reference = ground_reference
        self.spawn_pose = dict(spawn_pose or {})
        self.ground_target_z = None
        self.ground_from_height_z = None
        self.ground_target_source = "uninitialized"
        self.ground_reference_error_m = math.nan

        self.trajectory_origin = self.trajectory.points[0]

        self.stabilize_position_tolerance_m = 0.50
        self.stabilize_horizontal_speed_m_s = 0.30
        self.stabilize_vertical_speed_m_s = 0.22
        self.stabilize_tilt_rad = math.radians(5.0)
        self.stabilize_rate_rad_s = math.radians(18.0)
        self.stabilize_hold_s = 1.25
        self.stabilize_timeout_s = 35.0
        self.stabilize_max_excursion_m = 2.0
        self.stabilize_max_horizontal_speed_m_s = 2.0
        self.stabilize_max_vertical_excursion_m = 1.0
        self.stabilize_max_vertical_speed_m_s = 1.5
        self.stable_since = None
        self.fixture_released = False
        self.release_ready = False
        self.held_ready_since = None
        self.held_ready_hold_s = 0.20

        # The final metre is deliberately slower. PX4 performs the final
        # contact detection and auto-disarm after the controlled handoff.
        self.land_slow_height_m = 1.0
        self.land_rate_gain = 0.45
        self.land_min_rate_m_s = 0.08
        self.land_handoff_height_m = 0.28
        self.land_handoff_xy_m = 0.30
        self.land_handoff_vz_m_s = 0.25
        self.land_handoff_tilt_rad = math.radians(8.0)
        self.land_handoff_hold_s = 0.35
        self.land_ready_since = None
        self.max_landing_time_s = max(45.0, 2.5 * self.spawn_height_m + 25.0)
        self.descent_decel = 0.65

    def initialize_airborne_target(self, timeout=8.0, freshness_limit=2.0):
        self.wait_for_fresh_telemetry(timeout, freshness_limit)

        with self.state_lock:
            self.x0 = self.state.x
            self.y0 = self.state.y
            self.z0 = self.state.z
            self.yaw0 = self.state.yaw

        self.target_x = self.x0
        self.target_y = self.y0
        self.target_z = self.z0
        self.land_x = self.x0
        self.land_y = self.y0
        self.land_yaw_unwrapped = self.yaw0

        # This is the backup landing estimate under test. It intentionally uses
        # the exact randomized height supplied by the launcher.
        self.ground_from_height_z = self.z0 + self.spawn_height_m
        if self.ground_reference is not None:
            reference_z = float(self.ground_reference[2])
            self.ground_target_z = reference_z
            self.ground_target_source = "stored_ground_reference"
            self.ground_reference_error_m = self.ground_from_height_z - reference_z
        else:
            self.ground_target_z = self.ground_from_height_z
            self.ground_target_source = "launcher_height_backup"

        self._reset_position_pids()
        self.pid_x.setpoint = self.target_x
        self.pid_y.setpoint = self.target_y
        self.pid_z.setpoint = self.target_z

        self.current_vz_cmd = 0.0
        self.stable_since = None
        self.fixture_released = False
        self.release_ready = False
        self.held_ready_since = None
        self.land_ready_since = None
        self.trajectory_start_time = None
        self.mission_time = 0.0
        self.phase = "STABILIZE"
        self.phase_enter_time = time.monotonic()

        print(
            "[randomized] Airborne origin: "
            f"x={self.x0:.3f} y={self.y0:.3f} z={self.z0:.3f} "
            f"yaw={self.yaw0:.3f}"
        )
        print(
            "[randomized] Realized launcher height / backup local ground: "
            f"height={self.spawn_height_m:.3f} "
            f"backup_ground_z={self.ground_from_height_z:.3f}"
        )
        print(
            "[randomized] Landing ground target: "
            f"z={self.ground_target_z:.3f} source={self.ground_target_source}"
        )
        if self.ground_reference is not None:
            print(
                "[randomized] Ground-reference cross-check: "
                f"reference_z={self.ground_reference[2]:.3f} "
                f"prediction_error={self.ground_reference_error_m:.3f}m"
            )

    def estimated_agl_m(self, z):
        if self.ground_target_z is None:
            return math.nan
        return self.ground_target_z - z

    def set_realized_spawn_height(self, height_m):
        """Use the verified Gazebo height actually achieved by the fixture."""
        if not math.isfinite(height_m) or height_m <= 0.0:
            raise ValueError("realized spawn height must be finite and positive")
        self.spawn_height_m = float(height_m)
        self.max_landing_time_s = max(45.0, 2.5 * self.spawn_height_m + 25.0)

    def metadata(self):
        return {
            "spawn_pose": self.spawn_pose,
            "spawn_height_m": self.spawn_height_m,
            "airborne_local_origin": {
                "x": self.x0,
                "y": self.y0,
                "z": self.z0,
                "yaw": self.yaw0,
            },
            "ground_reference_local": (
                None
                if self.ground_reference is None
                else {
                    "x": self.ground_reference[0],
                    "y": self.ground_reference[1],
                    "z": self.ground_reference[2],
                    "yaw": self.ground_reference[3],
                }
            ),
            "predicted_ground_local_z": self.ground_target_z,
            "height_backup_ground_local_z": self.ground_from_height_z,
            "ground_target_source": self.ground_target_source,
            "ground_reference_error_m": self.ground_reference_error_m,
            "fixture_released": self.fixture_released,
            "stabilization_policy": {
                "max_excursion_m": self.stabilize_max_excursion_m,
                "max_horizontal_speed_m_s": self.stabilize_max_horizontal_speed_m_s,
                "max_vertical_excursion_m": self.stabilize_max_vertical_excursion_m,
                "max_vertical_speed_m_s": self.stabilize_max_vertical_speed_m_s,
            },
            "landing_policy": {
                "slow_height_m": self.land_slow_height_m,
                "max_descent_rate_m_s": self.max_descent_rate,
                "handoff_height_m": self.land_handoff_height_m,
                "handoff_xy_m": self.land_handoff_xy_m,
            },
        }

    def notify_fixture_released(self, now_mono=None):
        """Begin free-flight stabilization after the Gazebo joint is detached."""
        now_mono = time.monotonic() if now_mono is None else now_mono
        with self.state_lock:
            self.target_x = self.state.x
            self.target_y = self.state.y
            self.target_z = self.state.z
            self.yaw0 = self.state.yaw
        self.x0 = self.target_x
        self.y0 = self.target_y
        self.z0 = self.target_z
        self.land_x = self.x0
        self.land_y = self.y0
        self.land_yaw_unwrapped = self.yaw0
        # Recompute the launcher-height backup in the free-flight frame. When a
        # pre-spawn ground reference exists it remains the final target; this
        # mirrors the real vehicle, which records ground before takeoff.
        self.ground_from_height_z = self.z0 + self.spawn_height_m
        if self.ground_reference is not None:
            reference_z = float(self.ground_reference[2])
            self.ground_target_z = reference_z
            self.ground_target_source = "stored_ground_reference"
            self.ground_reference_error_m = self.ground_from_height_z - reference_z
        else:
            self.ground_target_z = self.ground_from_height_z
            self.ground_target_source = "launcher_height_backup"
        self._reset_position_pids()
        self.pid_x.setpoint = self.target_x
        self.pid_y.setpoint = self.target_y
        self.pid_z.setpoint = self.target_z
        self.fixture_released = True
        self.release_ready = False
        self.stable_since = None
        self.phase_enter_time = now_mono
        print(
            "[randomized] Fixture released; free-flight origin: "
            f"x={self.x0:.3f} y={self.y0:.3f} z={self.z0:.3f}"
        )

    def _active_state(self):
        with self.state_lock:
            return {
                "x": self.state.x,
                "y": self.state.y,
                "z": self.state.z,
                "vx": self.state.vx,
                "vy": self.state.vy,
                "vz": self.state.vz,
                "roll": self.state.roll,
                "pitch": self.state.pitch,
                "p": self.state.roll_rate,
                "q": self.state.pitch_rate,
                "r": self.state.yaw_rate,
            }

    def _validate_runtime_health(self, snapshot):
        super()._validate_runtime_health(snapshot)
        if self.phase == "STABILIZE":
            if not snapshot["armed"]:
                raise RuntimeError("Unexpected disarm during STABILIZE")
            if snapshot["heartbeat_main_mode"] != 6:
                raise RuntimeError(
                    "PX4 left OFFBOARD during STABILIZE: "
                    f"main_mode={snapshot['heartbeat_main_mode']}"
                )

    def update_phase(self, x, y, z, now_mono):
        self.last_phase_transition = ""
        state = self._active_state()

        if self.phase == "STABILIZE":
            position_error = math.sqrt(
                (x - self.target_x) ** 2
                + (y - self.target_y) ** 2
                + (z - self.target_z) ** 2
            )
            if not self.fixture_released:
                held_quiet = (
                    math.hypot(state["vx"], state["vy"]) <= 0.40
                    and abs(state["vz"]) <= 0.30
                    and max(abs(state["p"]), abs(state["q"]), abs(state["r"]))
                    <= self.stabilize_rate_rad_s
                )
                if held_quiet:
                    if self.held_ready_since is None:
                        self.held_ready_since = now_mono
                    elif now_mono - self.held_ready_since >= self.held_ready_hold_s:
                        self.release_ready = True
                else:
                    self.held_ready_since = None
                if now_mono - self.phase_enter_time > self.stabilize_timeout_s:
                    raise RuntimeError("Held airborne initialization did not settle")
                return

            horizontal_excursion = math.hypot(x - self.x0, y - self.y0)
            horizontal_speed = math.hypot(state["vx"], state["vy"])
            vertical_excursion = abs(z - self.z0)
            vertical_speed = abs(state["vz"])
            if horizontal_excursion > self.stabilize_max_excursion_m:
                raise RuntimeError(
                    "Airborne initialization exceeded horizontal excursion limit: "
                    f"{horizontal_excursion:.3f}m > "
                    f"{self.stabilize_max_excursion_m:.3f}m"
                )
            if horizontal_speed > self.stabilize_max_horizontal_speed_m_s:
                raise RuntimeError(
                    "Airborne initialization exceeded horizontal speed limit: "
                    f"{horizontal_speed:.3f}m/s > "
                    f"{self.stabilize_max_horizontal_speed_m_s:.3f}m/s"
                )
            if vertical_excursion > self.stabilize_max_vertical_excursion_m:
                raise RuntimeError(
                    "Airborne initialization exceeded vertical excursion limit: "
                    f"{vertical_excursion:.3f}m > "
                    f"{self.stabilize_max_vertical_excursion_m:.3f}m"
                )
            if vertical_speed > self.stabilize_max_vertical_speed_m_s:
                raise RuntimeError(
                    "Airborne initialization exceeded vertical speed limit: "
                    f"{vertical_speed:.3f}m/s > "
                    f"{self.stabilize_max_vertical_speed_m_s:.3f}m/s"
                )

            stable = (
                position_error <= self.stabilize_position_tolerance_m
                and math.hypot(state["vx"], state["vy"])
                <= self.stabilize_horizontal_speed_m_s
                and abs(state["vz"]) <= self.stabilize_vertical_speed_m_s
                and max(abs(state["roll"]), abs(state["pitch"]))
                <= self.stabilize_tilt_rad
                and max(abs(state["p"]), abs(state["q"]), abs(state["r"]))
                <= self.stabilize_rate_rad_s
            )
            if stable:
                if self.stable_since is None:
                    self.stable_since = now_mono
                elif now_mono - self.stable_since >= self.stabilize_hold_s:
                    self._reset_position_pids()
                    self.trajectory.reset()
                    self.trajectory_start_time = now_mono
                    self.mission_time = 0.0
                    self._transition("TRAJECTORY", now_mono)
            else:
                self.stable_since = None

            if (
                self.phase == "STABILIZE"
                and now_mono - self.phase_enter_time > self.stabilize_timeout_s
            ):
                raise RuntimeError("Airborne initialization did not stabilize")

        elif self.phase == "TRAJECTORY":
            if self.trajectory.finished:
                final_target = self.trajectory.points[-1]
                self.current_vz_cmd = 0.0
                # Explicitly return to the randomized airborne X0/Y0 instead
                # of trusting the final CSV point to be exactly zero.
                self.land_x = self.x0
                self.land_y = self.y0
                self.land_yaw_unwrapped = (
                    self.yaw0
                    + final_target.yaw
                    - self.trajectory_origin.yaw
                )
                self.pid_x.setpoint = self.land_x
                self.pid_y.setpoint = self.land_y
                self.land_ready_since = None
                self._transition("LAND", now_mono)

        elif self.phase == "LAND":
            if now_mono - self.phase_enter_time > self.max_landing_time_s:
                raise RuntimeError("Controlled landing approach timed out")

            agl = self.estimated_agl_m(z)
            horizontal_error = math.hypot(x - self.land_x, y - self.land_y)
            ready = (
                -0.10 <= agl <= self.land_handoff_height_m
                and horizontal_error <= self.land_handoff_xy_m
                and abs(state["vz"]) <= self.land_handoff_vz_m_s
                and max(abs(state["roll"]), abs(state["pitch"]))
                <= self.land_handoff_tilt_rad
            )
            if ready:
                if self.land_ready_since is None:
                    self.land_ready_since = now_mono
                elif now_mono - self.land_ready_since >= self.land_handoff_hold_s:
                    self._transition("DONE", now_mono)
            else:
                self.land_ready_since = None

            if agl < -0.25:
                raise RuntimeError(
                    f"Vehicle passed predicted ground by {-agl:.3f}m"
                )

    def stabilize_controller(self, x, y, z, dt):
        if not self.fixture_released:
            target = TrajectoryPoint(
                time=0.0,
                x=0.0,
                y=0.0,
                z=0.0,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                yaw=0.0,
            )
            zeros = self._zero_pid_terms()
            return self._control_result(
                target=target,
                desired=(x, y, z),
                planned=(0.0, 0.0, 0.0),
                pid_terms=(zeros.copy(), zeros.copy(), zeros.copy()),
                command=(0.0, 0.0, 0.0),
                limited=(False, False, False),
                yaw_unwrapped=self.yaw0,
            )

        self.pid_x.setpoint = self.target_x
        self.pid_y.setpoint = self.target_y
        self.pid_z.setpoint = self.target_z
        correction_x = self.pid_x.update(x, dt)
        correction_y = self.pid_y.update(y, dt)
        correction_z = self.pid_z.update(z, dt)
        command, limited = self._limit_velocity_command(
            correction_x, correction_y, correction_z
        )
        target = TrajectoryPoint(
            time=0.0,
            x=0.0,
            y=0.0,
            z=0.0,
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
        origin = self.trajectory_origin

        desired_x = self.x0 + target.x - origin.x
        desired_y = self.y0 + target.y - origin.y
        desired_z = self.z0 + target.z - origin.z

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
        yaw_unwrapped = self.yaw0 + target.yaw - origin.yaw
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

    def descent_rate_target(self, agl_m):
        if not math.isfinite(agl_m) or agl_m <= 0.0:
            return 0.0
        if agl_m >= self.land_slow_height_m:
            return self.max_descent_rate
        return min(
            self.max_descent_rate,
            max(self.land_min_rate_m_s, self.land_rate_gain * agl_m),
        )

    def landing_controller(self, x, y, z, yaw, dt):
        agl = self.estimated_agl_m(z)
        target_vz = self.descent_rate_target(agl)
        delta = target_vz - self.current_vz_cmd
        if delta >= 0.0:
            delta = min(delta, self.descent_accel * dt)
        else:
            delta = max(delta, -self.descent_decel * dt)
        self.current_vz_cmd += delta

        correction_x = self.pid_x.update(x, dt)
        correction_y = self.pid_y.update(y, dt)
        command, limited = self._limit_velocity_command(
            correction_x,
            correction_y,
            self.current_vz_cmd,
        )
        target = TrajectoryPoint(
            time=0.0,
            x=0.0,
            y=0.0,
            z=self.ground_target_z - self.z0,
            vx=0.0,
            vy=0.0,
            vz=self.current_vz_cmd,
            yaw=wrapped_angle(self.land_yaw_unwrapped - self.yaw0),
        )
        zero_z = self._zero_pid_terms()
        return self._control_result(
            target=target,
            desired=(self.land_x, self.land_y, self.ground_target_z),
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
        if self.phase == "STABILIZE":
            return self.stabilize_controller(x, y, z, self.control_dt)
        return super().done_controller(x, y, z, yaw)
