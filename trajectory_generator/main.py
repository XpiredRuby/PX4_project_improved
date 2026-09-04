from pose import Pose

from commands import (
    LineCommand,
    TurnCommand,
    RotateCommand,
    HoverCommand
)

from trajectory import TrajectoryGenerator
from plot import plot_trajectory
from planner import TrajectoryPlanner
from export import save_trajectory_csv



start = Pose(
    x=0,
    y=0,
    z=-8,
    yaw=0
)


generator = TrajectoryGenerator(
    start_pose=start,
    dt=0.05,
    profile_type="scurve",
    default_jerk=1.0,
    default_yaw_jerk=1.0,
)


commands = [

    LineCommand(
        distance=30,
        speed=2,
        heading = 0,
        acceleration=1,
    ),

    TurnCommand(
        angle=-180,
        radius=10,
        speed=1,
    ) ,
    LineCommand(
        distance=30,
        speed=2,
        heading = 0,
        acceleration=1,
        ),
    RotateCommand(
        angle=-90,
        yaw_rate=20
        ),
    LineCommand(
        distance=20,
        speed=2,
        heading = 0,
        acceleration=1,
        ),
    HoverCommand(
        duration=5
    ),    
]



planner = TrajectoryPlanner()

profiles = planner.plan_speeds(commands)


trajectory = generator.generate(
    commands,
    profiles
)


plot_trajectory(trajectory)
save_trajectory_csv(
    "trajectory.csv",
    trajectory
)

print(
    f"Generated {len(trajectory)} points"
)

print(
    f"Saved to trajetory.csv"
)

for i, (command, p) in enumerate(zip(commands, profiles)):

    if p is None:
        print(i, "Hover:", command.duration)
        continue

    print(
        i,
        "start:", p.start_speed,
        "cruise:", p.cruise_speed,
        "end:", p.end_speed
    )