import csv
import math
from dataclasses import dataclass


@dataclass
class TrajectoryPoint:

    time: float = 0.0

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    yaw: float = 0.0


class Trajectory:

    def __init__(self, filename):

        self.points = []
        self.index = 0
        self.finished = False

        with open(filename, "r") as file:

            reader = csv.DictReader(file)

            for row in reader:

                self.points.append(

                    TrajectoryPoint(

                        time=float(row["time"]),

                        x=float(row["x"]),
                        y=float(row["y"]),
                        z=float(row["z"]),

                        vx=float(row["vx"]),
                        vy=float(row["vy"]),
                        vz=float(row["vz"]),

                        yaw=float(row["yaw"])

                    )

                )

        # -------------------------------------
        # Trajectory duration
        # -------------------------------------
        if len(self.points) > 0:
            self.duration = self.points[-1].time
        else:
            self.duration = 0.0        


    def reset(self):

        self.index = 0
        self.finished = False


    def get_target(self, t):
        """
        Returns the desired trajectory point
        interpolated to time t.
        """

        # -------------------------------------
        # End of trajectory
        # -------------------------------------

        if t >= self.points[-1].time:

            self.finished = True

            return self.points[-1]


        # -------------------------------------
        # Advance index
        # -------------------------------------

        while (
            self.index < len(self.points) - 2
            and
            self.points[self.index + 1].time <= t
        ):

            self.index += 1


        p1 = self.points[self.index]
        p2 = self.points[self.index + 1]


        # -------------------------------------
        # Interpolation ratio
        # -------------------------------------

        ratio = (

            t - p1.time

        ) / (

            p2.time - p1.time

        )


        target = TrajectoryPoint()

        target.time = t


        # -------------------------------------
        # Position
        # -------------------------------------

        target.x = p1.x + ratio * (p2.x - p1.x)
        target.y = p1.y + ratio * (p2.y - p1.y)
        target.z = p1.z + ratio * (p2.z - p1.z)


        # -------------------------------------
        # Velocity
        # -------------------------------------

        target.vx = p1.vx + ratio * (p2.vx - p1.vx)
        target.vy = p1.vy + ratio * (p2.vy - p1.vy)
        target.vz = p1.vz + ratio * (p2.vz - p1.vz)


        # -------------------------------------
        # Yaw
        # -------------------------------------

        dyaw = p2.yaw - p1.yaw

        while dyaw > math.pi:
            dyaw -= 2.0 * math.pi

        while dyaw < -math.pi:
            dyaw += 2.0 * math.pi

        target.yaw = p1.yaw + ratio * dyaw

        return target