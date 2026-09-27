import csv


def save_trajectory_csv(filename, trajectory):

    with open(filename, "w", newline="") as f:

        writer = csv.writer(f)

        writer.writerow([
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
        ])

        for p in trajectory:

            writer.writerow([
                p.time,

                p.x,
                p.y,
                p.z,

                p.yaw,

                p.vx,
                p.vy,
                p.vz,

                p.yaw_rate,

                p.ax,
                p.ay,
                p.az,
            ])