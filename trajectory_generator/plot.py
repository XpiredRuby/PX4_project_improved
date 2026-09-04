import matplotlib.pyplot as plt
import numpy as np


def plot_trajectory(points):

    # Extract data
    t = np.array([p.time for p in points])

    x = np.array([p.x for p in points])
    y = np.array([p.y for p in points])
    z = np.array([p.z for p in points])

    yaw = np.unwrap(
        np.array([p.yaw for p in points])
    )


    vx = np.array([p.vx for p in points])
    vy = np.array([p.vy for p in points])
    vz = np.array([p.vz for p in points])

    yaw_rate = np.array(
        [p.yaw_rate for p in points]
    )


    ax = np.array([p.ax for p in points])
    ay = np.array([p.ay for p in points])
    az = np.array([p.az for p in points])


    # =================================================
    # XY trajectory
    # =================================================

    plt.figure(figsize=(7,7))

    plt.plot(
        x,
        y,
        linewidth=2,
        label="Path"
    )

    plt.scatter(
        x[0],
        y[0],
        marker="o",
        label="Start"
    )

    plt.scatter(
        x[-1],
        y[-1],
        marker="x",
        label="End"
    )


    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("XY Trajectory")

    plt.axis("equal")
    plt.grid(True)
    plt.legend()



    # =================================================
    # Position vs time
    # =================================================

    plt.figure()

    plt.plot(t, x, label="X")
    plt.plot(t, y, label="Y")
    plt.plot(t, z, label="Z")

    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Position")

    plt.grid(True)
    plt.legend()



    # =================================================
    # Velocity vs time
    # =================================================

    plt.figure()

    plt.plot(t, vx, label="Vx")
    plt.plot(t, vy, label="Vy")
    plt.plot(t, vz, label="Vz")

    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Velocity")

    plt.grid(True)
    plt.legend()



    # =================================================
    # Acceleration vs time
    # =================================================

    plt.figure()

    plt.plot(t, ax, label="Ax")
    plt.plot(t, ay, label="Ay")
    plt.plot(t, az, label="Az")

    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.title("Acceleration")

    plt.grid(True)
    plt.legend()



    # =================================================
    # Yaw
    # =================================================

    plt.figure()

    plt.plot(
        t,
        np.rad2deg(yaw),
        label="Yaw"
    )

    plt.xlabel("Time (s)")
    plt.ylabel("Yaw (deg)")
    plt.title("Yaw")

    plt.grid(True)
    plt.legend()



    # =================================================
    # Yaw rate
    # =================================================

    plt.figure()

    plt.plot(
        t,
        np.rad2deg(yaw_rate),
        label="Yaw rate"
    )

    plt.xlabel("Time (s)")
    plt.ylabel("Yaw rate (deg/s)")
    plt.title("Yaw Rate")

    plt.grid(True)
    plt.legend()


    plt.show()