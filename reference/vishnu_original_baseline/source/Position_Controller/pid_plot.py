#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# ==========================
# Configuration
# ==========================

CSV_FILE = "pid_log_fri2.csv"


# ==========================
# Load CSV
# ==========================

df = pd.read_csv(CSV_FILE)

print(df.head())
print(df.columns)


# ==========================
# Time
# ==========================

t = df["time"] - df["time"].iloc[0]


# ==========================
# 3D Trajectory Plot
# ==========================

fig = plt.figure(figsize=(8,6))

ax = fig.add_subplot(111, projection="3d")

ax.plot(
    df["target_x"],
    df["target_y"],
    df["target_z"],
    label="Desired trajectory"
)

ax.plot(
    df["x"],
    df["y"],
    df["z"],
    label="Actual trajectory"
)


ax.scatter(
    df["x"].iloc[0],
    df["y"].iloc[0],
    df["z"].iloc[0],
    marker="o",
    label="Start"
)

ax.scatter(
    df["x"].iloc[-1],
    df["y"].iloc[-1],
    df["z"].iloc[-1],
    marker="x",
    label="End"
)


ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")

ax.set_title("3D Position Tracking")

ax.legend()

plt.grid()


# ==========================
# Position Plot
# ==========================

fig, axs = plt.subplots(
    3,
    1,
    figsize=(10,8),
    sharex=True
)


positions = [
    ("x", "target_x", "X position"),
    ("y", "target_y", "Y position"),
    ("z", "target_z", "Z position")
]


for ax, (actual, target, name) in zip(axs, positions):

    ax.plot(
        t,
        df[target],
        label="Desired"
    )

    ax.plot(
        t,
        df[actual],
        label="Actual"
    )

    ax.set_ylabel(name)

    ax.grid()


axs[0].legend()

axs[-1].set_xlabel("Time (s)")

plt.suptitle("Position Tracking")



# ==========================
# Velocity Plot
# ==========================

fig, axs = plt.subplots(
    3,
    1,
    figsize=(10,8),
    sharex=True
)


velocities = [
    ("vx", "cmd_vx", "Velocity X"),
    ("vy", "cmd_vy", "Velocity Y"),
    ("vz", "cmd_vz", "Velocity Z")
]


for ax, (actual, target, name) in zip(axs, velocities):

    ax.plot(
        t,
        df[target],
        label="Desired"
    )

    ax.plot(
        t,
        df[actual],
        label="Actual"
    )

    ax.set_ylabel(name)

    ax.grid()


axs[0].legend()

axs[-1].set_xlabel("Time (s)")

plt.suptitle("Velocity Tracking")



# ==========================
# Velocity Command Plot
# ==========================

fig, axs = plt.subplots(
    3,
    1,
    figsize=(10,8),
    sharex=True
)


commands = [
    ("cmd_vx", "vx", "VX command"),
    ("cmd_vy", "vy", "VY command"),
    ("cmd_vz", "vz", "VZ command")
]


for ax, (cmd, actual, name) in zip(axs, commands):

    ax.plot(
        t,
        df[cmd],
        label="Command"
    )

    ax.plot(
        t,
        df[actual],
        label="Actual"
    )

    ax.set_ylabel(name)

    ax.grid()


axs[0].legend()

axs[-1].set_xlabel("Time (s)")

plt.suptitle("Velocity Commands")



# ==========================
# Yaw Plot
# ==========================

plt.figure(figsize=(10,4))

plt.plot(
    t,
    np.rad2deg(df["yaw"]),
    label='actual'
)
plt.plot(
    t,
    np.rad2deg(df["target_yaw"]),
    label='target'
)
plt.legend()

plt.xlabel("Time (s)")
plt.ylabel("Yaw (deg)")

plt.title("Yaw")

plt.grid()



# ==========================
# Flight Phase
# ==========================

plt.figure(figsize=(10,3))

plt.plot(
    t,
    df["phase"]
)

plt.xlabel("Time (s)")
plt.ylabel("Phase")

plt.title("Flight Phase")

plt.grid()



plt.show()
