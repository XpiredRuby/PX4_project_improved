"""CMU DJI Matrice 100 delivery dataset (Rodrigues et al., Sci Data 2021, CC BY 4.0, doi:10.1184/R1/12683453).

Velocity columns are body-frame; rotating by the logged quaternion reproduces GPS-derived east/north motion
(r > 0.99). The anemometer reports the uncorrected relative airflow; its frame and sign convention are chosen
by physical consistency (the derived wind must stay steady while the aircraft changes heading).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

G = 9.80665


def rotation(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q.T
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    ).transpose(2, 0, 1)


def load(path: Path) -> dict:
    f = pd.read_csv(path)
    lat0, lon0 = f.position_y.iloc[0], f.position_x.iloc[0]
    east = (f.position_x - lon0).to_numpy() * np.cos(np.radians(lat0)) * 111_320.0
    north = (f.position_y - lat0).to_numpy() * 111_320.0
    q = f[
        ["orientation_x", "orientation_y", "orientation_z", "orientation_w"]
    ].to_numpy()
    rot = rotation(q)
    v_body = f[["velocity_x", "velocity_y", "velocity_z"]].to_numpy()
    v_world = np.einsum("nij,nj->ni", rot, v_body)
    tilt = np.arccos(np.clip(rot[:, 2, 2], -1.0, 1.0))
    yaw = np.arctan2(rot[:, 1, 0], rot[:, 0, 0])
    return {
        "t": f.time.to_numpy(),
        "east": east,
        "north": north,
        "alt": f.position_z.to_numpy() - f.position_z.iloc[0],
        "v": v_world[:, :2],
        "tilt": tilt,
        "yaw": yaw,
        "air_speed": f.wind_speed.to_numpy(),
        "air_angle": np.radians(f.wind_angle.to_numpy()),
        "power": (f.battery_voltage * f.battery_current).to_numpy(),
    }


CONVENTIONS = [
    (frame, sense, sign)
    for frame in ("world", "body")
    for sense in ("cw", "ccw")
    for sign in (1, -1)
]


def airflow(d: dict, convention) -> np.ndarray:
    """Relative air velocity (air minus vehicle), east/north, for one candidate anemometer convention."""
    frame, sense, sign = convention
    ang = d["air_angle"] * (1 if sense == "cw" else -1)
    if frame == "body":
        ang = ang + (
            np.pi / 2 - d["yaw"]
        )  # body x forward -> compass bearing of heading
    return sign * d["air_speed"][:, None] * np.column_stack([np.sin(ang), np.cos(ang)])


def wind(d: dict, convention) -> np.ndarray:
    return airflow(d, convention) + d["v"]


def cruise_mask(d: dict, min_speed=2.0, min_alt=10.0) -> np.ndarray:
    return (np.linalg.norm(d["v"], axis=1) > min_speed) & (d["alt"] > min_alt)


def best_convention(flights) -> tuple:
    """Convention whose derived wind is most stable during cruise (sum of per-flight std)."""

    def score(c):
        s = 0.0
        for d in flights:
            m = cruise_mask(d)
            if m.sum() > 50:
                s += float(np.sum(np.std(wind(d, c)[m], axis=0)))
        return s

    return min(CONVENTIONS, key=score)
