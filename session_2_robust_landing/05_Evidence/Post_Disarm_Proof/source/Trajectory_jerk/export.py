import csv


FIELDS = [
    "time",
    "x",
    "y",
    "z",
    "yaw",
    "vx",
    "vy",
    "vz",
    "yaw_rate",
    "ax",
    "ay",
    "az",
    "jx",
    "jy",
    "jz",
    "yaw_acceleration",
    "yaw_jerk",
]


def save_trajectory_csv(filename, trajectory):
    """Export every position and derivative carried by TrajectoryPoint."""
    with open(filename, "w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader()
        for point in trajectory:
            writer.writerow(
                {
                    field: getattr(point, field)
                    for field in FIELDS
                }
            )
