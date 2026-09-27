#!/usr/bin/env python3

import argparse
import math
import os
import random
import secrets
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class SpawnPose:
    x: float
    y: float
    z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    mode: str
    seed: int

    def as_dict(self):
        return asdict(self)


def _env_float(name, default=None):
    value = os.environ.get(name)
    return default if value in (None, "") else float(value)


def _env_int(name, default=None):
    value = os.environ.get(name)
    return default if value in (None, "") else int(value, 0)


def parser():
    p = argparse.ArgumentParser(
        description="Bounded random/manual airborne initialization for PX4 SITL"
    )
    p.add_argument(
        "--mode",
        choices=("random", "manual"),
        default=os.environ.get("PX4_INIT_MODE", "random"),
    )
    p.add_argument("--seed", type=int, default=_env_int("PX4_SPAWN_SEED"))
    p.add_argument("--x", type=float, default=_env_float("PX4_SPAWN_X"))
    p.add_argument("--y", type=float, default=_env_float("PX4_SPAWN_Y"))
    p.add_argument("--z", type=float, default=_env_float("PX4_SPAWN_Z"))
    p.add_argument(
        "--roll-deg", type=float, default=_env_float("PX4_SPAWN_ROLL_DEG", 0.0)
    )
    p.add_argument(
        "--pitch-deg", type=float, default=_env_float("PX4_SPAWN_PITCH_DEG", 0.0)
    )
    p.add_argument(
        "--yaw-deg", type=float, default=_env_float("PX4_SPAWN_YAW_DEG", 0.0)
    )
    p.add_argument("--z-min", type=float, default=_env_float("PX4_SPAWN_Z_MIN", 3.0))
    p.add_argument("--z-max", type=float, default=_env_float("PX4_SPAWN_Z_MAX", 10.0))
    p.add_argument(
        "--tilt-max-deg",
        type=float,
        default=_env_float("PX4_SPAWN_TILT_MAX_DEG", 2.0),
    )
    p.add_argument(
        "--model", default=os.environ.get("PX4_GZ_MODEL_NAME", "x500_0")
    )
    p.add_argument(
        "--world", default=os.environ.get("PX4_GZ_WORLD_NAME", "default")
    )
    return p


def validate_pose(pose):
    values = (
        pose.x,
        pose.y,
        pose.z,
        pose.roll_deg,
        pose.pitch_deg,
        pose.yaw_deg,
    )
    if not all(math.isfinite(v) for v in values):
        raise ValueError("All spawn values must be finite")
    if not (-2.0 <= pose.x <= 2.0 and -2.0 <= pose.y <= 2.0):
        raise ValueError("X and Y must remain inside the configured +/-2 m envelope")
    if not (2.0 <= pose.z <= 15.0):
        raise ValueError("Z must be a positive airborne height from 2 m to 15 m")
    if abs(pose.roll_deg) > 15.0 or abs(pose.pitch_deg) > 15.0:
        raise ValueError("Roll and pitch are safety-limited to +/-15 degrees")
    if abs(pose.yaw_deg) > 180.0:
        raise ValueError("Yaw must be within +/-180 degrees")
    return pose


def build_pose(args):
    seed = args.seed if args.seed is not None else secrets.randbits(32)
    rng = random.Random(seed)
    if args.mode == "manual":
        missing = [name for name in ("x", "y", "z") if getattr(args, name) is None]
        if missing:
            raise ValueError(
                "Manual mode requires --x, --y, and --z; missing "
                + ", ".join(missing)
            )
        pose = SpawnPose(
            x=args.x,
            y=args.y,
            z=args.z,
            roll_deg=args.roll_deg,
            pitch_deg=args.pitch_deg,
            yaw_deg=args.yaw_deg,
            mode="manual",
            seed=seed,
        )
    else:
        if not (2.0 <= args.z_min <= args.z_max <= 15.0):
            raise ValueError("Random Z bounds must satisfy 2 <= z-min <= z-max <= 15")
        if not (0.0 <= args.tilt_max_deg <= 15.0):
            raise ValueError("tilt-max-deg must be between 0 and 15")
        pose = SpawnPose(
            x=rng.uniform(-2.0, 2.0),
            y=rng.uniform(-2.0, 2.0),
            z=rng.uniform(args.z_min, args.z_max),
            roll_deg=rng.uniform(-args.tilt_max_deg, args.tilt_max_deg),
            pitch_deg=rng.uniform(-args.tilt_max_deg, args.tilt_max_deg),
            yaw_deg=rng.uniform(-180.0, 180.0),
            mode="random",
            seed=seed,
        )
    return validate_pose(pose)


def quaternion_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
