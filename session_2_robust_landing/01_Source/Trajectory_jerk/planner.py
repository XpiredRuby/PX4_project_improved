from segments import MotionProfile
from commands import (
    LineCommand,
    TurnCommand,
    RotateCommand,
    HoverCommand
)

import math


class TrajectoryPlanner:

    def __init__(
        self,
        default_speed=2.0,
        default_acceleration=1.0,
        default_yaw_rate=30.0,
        default_yaw_acceleration=1.0,
        default_jerk=1.0,
        default_yaw_jerk=1.0,
    ):
        self.default_speed = default_speed
        self.default_acceleration = default_acceleration

        self.default_yaw_rate = default_yaw_rate
        self.default_yaw_acceleration = default_yaw_acceleration

        self.default_jerk = default_jerk
        self.default_yaw_jerk = default_yaw_jerk

    def create_linear_profile(
        self,
        command,
        start_speed,
        end_speed
    ):
        speed = (
            command.speed
            if command.speed is not None
            else self.default_speed
        )

        acceleration = (
            command.acceleration
            if command.acceleration is not None
            else self.default_acceleration
        )

        jerk = (
            command.jerk
            if getattr(command, "jerk", None) is not None
            else self.default_jerk
        )

        return MotionProfile(
            start_speed=start_speed,
            cruise_speed=speed,
            end_speed=end_speed,

            max_acceleration=acceleration,
            max_deceleration=acceleration,
            max_jerk=jerk,
        )

    def create_rotation_profile(self, command):
        yaw_rate = (
            command.yaw_rate
            if command.yaw_rate is not None
            else self.default_yaw_rate
        )

        yaw_acceleration = (
            command.yaw_acceleration
            if getattr(command, "yaw_acceleration", None) is not None
            else self.default_yaw_acceleration
        )

        yaw_jerk = (
            command.yaw_jerk
            if getattr(command, "yaw_jerk", None) is not None
            else self.default_yaw_jerk
        )

        return MotionProfile(
            start_speed=0.0,
            cruise_speed=0.0,
            end_speed=0.0,

            max_acceleration=0.0,
            max_deceleration=0.0,

            start_yaw_rate=0.0,
            cruise_yaw_rate=math.radians(yaw_rate),
            end_yaw_rate=0.0,

            max_yaw_acceleration=yaw_acceleration,
            max_yaw_jerk=yaw_jerk,
        )

    def _command_speed(self, command):
        return (
            command.speed
            if command.speed is not None
            else self.default_speed
        )

    @staticmethod
    def _same_direction(angle_degrees):
        angle = math.radians(angle_degrees)
        return (
            math.isclose(
                math.sin(angle),
                0.0,
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            and math.cos(angle) > 0.0
        )

    def _tangent_is_continuous(self, current, following):
        if isinstance(current, LineCommand) and isinstance(
            following,
            LineCommand,
        ):
            return self._same_direction(
                following.heading - current.heading
            )
        if isinstance(current, LineCommand) and isinstance(
            following,
            TurnCommand,
        ):
            return self._same_direction(current.heading)
        if isinstance(current, TurnCommand) and isinstance(
            following,
            LineCommand,
        ):
            return self._same_direction(following.heading)
        if isinstance(current, TurnCommand) and isinstance(
            following,
            TurnCommand,
        ):
            return True
        return False

    def _boundary_speed(self, current, following):
        """Preserve speed only across a continuous tangent and curvature."""
        linear_types = (LineCommand, TurnCommand)
        if not isinstance(current, linear_types):
            return 0.0
        if not isinstance(following, linear_types):
            return 0.0
        if not self._tangent_is_continuous(current, following):
            return 0.0

        # Fixed-overlay turns ramp curvature smoothly from and back to zero.
        # Their boundary yaw rate is therefore zero, matching a straight line
        # without requiring a stop.
        return min(
            self._command_speed(current),
            self._command_speed(following),
        )

    def plan_speeds(self, commands):
        boundary_speeds = [0.0] * (len(commands) + 1)
        for index in range(len(commands) - 1):
            boundary_speeds[index + 1] = self._boundary_speed(
                commands[index],
                commands[index + 1],
            )

        profiles = []
        for index, command in enumerate(commands):
            if isinstance(command, (LineCommand, TurnCommand)):
                profiles.append(
                    self.create_linear_profile(
                        command,
                        boundary_speeds[index],
                        boundary_speeds[index + 1],
                    )
                )
            elif isinstance(command, RotateCommand):
                profiles.append(
                    self.create_rotation_profile(command)
                )
            elif isinstance(command, HoverCommand):
                profiles.append(None)
            else:
                raise ValueError(
                    f"Unknown command type: {type(command)}"
                )

        return profiles
