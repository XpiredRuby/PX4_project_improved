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
        default_yaw_acceleration=1.0
    ):

        self.default_speed = default_speed
        self.default_acceleration = default_acceleration

        self.default_yaw_rate = default_yaw_rate
        self.default_yaw_acceleration = default_yaw_acceleration


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


        return MotionProfile(

            start_speed=start_speed,
            cruise_speed=speed,
            end_speed=end_speed,

            max_acceleration=acceleration,
            max_deceleration=acceleration
        )


    def create_rotation_profile(self, command):

        yaw_rate = (
            command.yaw_rate
            if command.yaw_rate is not None
            else self.default_yaw_rate
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

            max_yaw_acceleration=self.default_yaw_acceleration
        )


    def plan_speeds(self, commands):

        profiles = []

        current_speed = 0.0


        for i, command in enumerate(commands):

            # -------------------------
            # Linear motion
            # -------------------------

            if isinstance(
                command,
                (
                    LineCommand,
                    TurnCommand
                )
            ):

                if i < len(commands)-1:

                    next_command = commands[i+1]

                    if isinstance(
                        next_command,
                        (HoverCommand, RotateCommand)
                    ):
                        end_speed = 0.0

                    else:
                        end_speed = (
                            next_command.speed
                            if next_command.speed is not None
                            else self.default_speed
                        )

                else:
                    end_speed = 0.0


                profiles.append(
                    self.create_linear_profile(
                        command,
                        current_speed,
                        end_speed
                    )
                )


                current_speed = end_speed


            # -------------------------
            # Rotation
            # -------------------------

            elif isinstance(command, RotateCommand):

                profiles.append(
                    self.create_rotation_profile(command)
                )

                current_speed = 0.0


            # -------------------------
            # Hover
            # -------------------------

            elif isinstance(command, HoverCommand):

                profiles.append(None)

                current_speed = 0.0


        return profiles