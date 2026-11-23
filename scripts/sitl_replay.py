"""Replay real X500 V2 setpoint streams through PX4 SITL + Gazebo x500 and record the simulated response.

For each selected 20 s window from a public Flight Review log, the sim first flies a 5 s lead-in of the
same setpoints (to approach the real state), then the scored window. Setpoints are translated so the
window starts at the sim's current position; pacing uses the simulator's own clock, so any speed factor
works. Each window is flown twice: with the SITL airframe defaults ("default") and with the real log's
own multicopter controller tuning ("log_tuning").

SITL_ONLY. Usage:
  python scripts/sitl_replay.py --autopilot ~/PX4-Autopilot --logs ~/validation-data/x500 --out DIR
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from fidelity import x500_ident as xi  # noqa: E402

LEAD_S, WINDOW_S, RATE_HZ = 5.0, 20.0, 50.0
ANCHOR_ALT_M = 15.0
TUNING_KEYS = [
    "MPC_XY_P",
    "MPC_XY_VEL_P_ACC",
    "MPC_XY_VEL_I_ACC",
    "MPC_XY_VEL_D_ACC",
    "MPC_Z_P",
    "MPC_Z_VEL_P_ACC",
    "MPC_Z_VEL_I_ACC",
    "MPC_Z_VEL_D_ACC",
    "MPC_TILTMAX_AIR",
    "MPC_ACC_HOR_MAX",
    "MPC_ACC_UP_MAX",
    "MPC_ACC_DOWN_MAX",
    "MPC_XY_VEL_MAX",
    "MPC_Z_VEL_MAX_UP",
    "MPC_Z_VEL_MAX_DN",
    "MC_ROLL_P",
    "MC_PITCH_P",
    "MC_ROLLRATE_P",
    "MC_PITCHRATE_P",
    "MC_ROLLRATE_I",
    "MC_PITCHRATE_I",
    "MC_ROLLRATE_D",
    "MC_PITCHRATE_D",
    "MC_ROLLRATE_K",
    "MC_PITCHRATE_K",
    "MPC_THR_HOVER",
    "MPC_USE_HTE",
]
SITL_PARAMS = {
    "COM_RC_IN_MODE": 4,
    "NAV_RCL_ACT": 0,
    "NAV_DLL_ACT": 0,
    "COM_OF_LOSS_T": 2.0,
    "COM_OBL_RC_ACT": 0,
    "COM_DISARM_PRFLT": 60,
    "EKF2_GPS_DELAY": 0.0,
}


def select_windows(log_dir: Path, max_windows: int, per_vehicle: int = 2):
    picked, used = [], {}
    for log_id, vehicle, npz in xi.unique_logs(log_dir):
        d = dict(np.load(npz))
        params = json.loads((log_dir / "npz" / f"{log_id}.params.json").read_text())
        for seg in xi.offboard_segments(log_id, vehicle, d):
            if not xi.position_tracked(seg):
                continue
            n_lead, n_win = int(LEAD_S * RATE_HZ), int(WINDOW_S * RATE_HZ)
            for start in range(n_lead, len(seg.t) - n_win, n_win):
                sl = slice(start - n_lead, start + n_win)
                sp = seg.sp_pos[sl]
                rel = sp - seg.pos[start - n_lead]
                speed = np.linalg.norm(seg.vel[start : start + n_win, :2], axis=1)
                if (
                    np.max(np.linalg.norm(rel[:, :2], axis=1)) > 60
                    or np.ptp(rel[:, 2]) > 10
                ):
                    continue
                if (
                    np.percentile(
                        np.linalg.norm(seg.sp_pos[sl] - seg.pos[sl], axis=1), 95
                    )
                    > 20
                ):
                    continue
                if speed.max() < 0.5:  # skip pure hover windows: nothing to compare
                    continue
                if used.get(seg.vehicle, 0) >= per_vehicle:
                    break
                used[seg.vehicle] = used.get(seg.vehicle, 0) + 1
                picked.append(
                    {
                        "log_id": log_id,
                        "vehicle": seg.vehicle,
                        "t0": float(seg.t[start]),
                        "sp_pos": rel.tolist(),
                        "sp_vel": np.nan_to_num(seg.sp_vel[sl]).tolist(),
                        "real_pos": (seg.pos[sl] - seg.pos[start - n_lead]).tolist(),
                        "tuning": {k: params[k] for k in TUNING_KEYS if k in params},
                    }
                )
                break
    picked.sort(key=lambda w: w["vehicle"])
    return picked[:max_windows]


def parse_variant(text: str) -> dict:
    """'thrust=1.38,tau_up=0.08,tau_down=0.1,drag=2.5' -> scale/override dict ({} = stock x500)."""
    out = {}
    for item in filter(None, (text or "").split(",")):
        key, _, value = item.partition("=")
        if key not in ("thrust", "tau_up", "tau_down", "drag"):
            raise ValueError("unknown variant key " + key)
        out[key] = float(value)
    return out


def write_variant_model(autopilot: Path, dest: Path, variant: dict) -> str:
    """Copy the stock x500 model to DEST/x500_cal with calibrated motor coefficients; returns the model name."""
    import re

    model = dest / "x500_cal"
    if model.exists():
        shutil.rmtree(model)
    shutil.copytree(autopilot / "Tools/simulation/gz/models/x500", model)
    sdf = (model / "model.sdf").read_text()
    # Included models (e.g. x500_base) must sit beside the variant so Gazebo can resolve them.
    for dep in re.findall(r"model://([A-Za-z0-9_\-]+)", sdf):
        target = dest / dep
        if not target.exists():
            shutil.copytree(autopilot / "Tools/simulation/gz/models" / dep, target)
    for tag, key, mode in (
        ("motorConstant", "thrust", "scale"),
        ("rotorDragCoefficient", "drag", "scale"),
        ("timeConstantUp", "tau_up", "set"),
        ("timeConstantDown", "tau_down", "set"),
    ):
        if key not in variant:
            continue
        value = variant[key]
        sdf = re.sub(
            rf"<{tag}>([^<]+)</{tag}>",
            lambda m, v=value, md=mode, t=tag: (
                f"<{t}>{(float(m.group(1)) * v if md == 'scale' else v):.8g}</{t}>"
            ),
            sdf,
        )
    sdf = re.sub(r"<model name=['\"]x500['\"]", "<model name='x500_cal'", sdf, count=1)
    (model / "model.sdf").write_text(sdf)
    cfg = model / "model.config"
    if cfg.exists():
        cfg.write_text(
            re.sub(
                r"<name>[^<]*</name>", "<name>x500_cal</name>", cfg.read_text(), count=1
            )
        )
    return "x500_cal"


class Sitl:
    def __init__(
        self, autopilot: Path, workdir: Path, speed: float, variant: dict | None = None
    ):
        build = autopilot / "build/px4_sitl_default"
        runtime = workdir / "runtime"
        if runtime.exists():
            shutil.rmtree(runtime)
        runtime.mkdir(parents=True)
        shutil.copy2(build / "rootfs/gz_env.sh", runtime / "gz_env.sh")
        env = os.environ.copy()
        for key in ("PX4_GZ_STANDALONE", "GZ_PARTITION", "PX4_GZ_MODEL_NAME"):
            env.pop(key, None)
        env.update(
            PX4_SYS_AUTOSTART="4001",
            PX4_SIM_MODEL="gz_x500",
            PX4_GZ_WORLD="default",
            PX4_GZ_MODEL_POSE="0,0,0,0,0,0",
            PX4_SIM_SPEED_FACTOR=str(speed),
            GZ_IP="127.0.0.1",
            HEADLESS="1",
        )
        env.update({"PX4_PARAM_" + k: str(v) for k, v in SITL_PARAMS.items()})
        if variant:
            name = write_variant_model(autopilot, workdir / "models", variant)
            env["PX4_SIM_MODEL"] = "gz_" + name
            models = (workdir / "models").resolve()
            # PX4 spawns file://$PX4_GZ_MODELS/<name>/model.sdf; point this run's private gz_env.sh at the variant.
            with open(runtime / "gz_env.sh", "a") as fh:
                fh.write(f"\nexport PX4_GZ_MODELS={models}\nexport GZ_SIM_RESOURCE_PATH={models}:$GZ_SIM_RESOURCE_PATH\n")
        self.log_path = workdir / "simulator.log"
        self.proc = subprocess.Popen(
            [
                str((build / "bin/px4").resolve()),
                "-d",
                str(build / "etc"),
                "-w",
                str(runtime),
            ],
            cwd=runtime,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=self.log_path.open("w"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.build, self.runtime = build, runtime
        deadline = time.monotonic() + 90
        while "Startup script returned successfully" not in self.log_path.read_text(
            errors="replace"
        ):
            if self.proc.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("SITL failed to start; see " + str(self.log_path))
            time.sleep(0.5)
        subprocess.run(
            [
                str(build / "bin/px4-mavlink"),
                "start",
                "-u",
                "14600",
                "-o",
                "14601",
                "-t",
                "127.0.0.1",
                "-m",
                "onboard",
                "-r",
                "400000",
            ],
            capture_output=True,
            timeout=10,
            check=True,
        )
        self.gcs = mavutil.mavlink_connection(
            "udpin:127.0.0.1:14550", source_system=254, source_component=190
        )
        self._stop = threading.Event()
        threading.Thread(target=self._heartbeat, daemon=True).start()
        self.m = mavutil.mavlink_connection(
            "udpin:127.0.0.1:14601", source_system=255, source_component=0
        )
        self.m.wait_heartbeat(timeout=30)
        self.interval(32, 50)  # LOCAL_POSITION_NED

    def _heartbeat(self):
        while not self._stop.is_set():
            while self.gcs.recv_match(blocking=False) is not None:
                pass
            self.gcs.mav.heartbeat_send(6, 8, 0, 0, 4)
            self._stop.wait(0.5)

    def interval(self, msg_id, hz):
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            511,
            0,
            msg_id,
            1e6 / hz,
            0,
            0,
            0,
            0,
            0,
        )

    def param(self, name, value=None):
        if value is not None:
            self.m.mav.param_set_send(
                self.m.target_system,
                self.m.target_component,
                name.encode(),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
                if float(value).is_integer() and name in ("MPC_USE_HTE",)
                else mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        else:
            self.m.mav.param_request_read_send(
                self.m.target_system, self.m.target_component, name.encode(), -1
            )
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is not None and msg.param_id == name:
                return float(msg.param_value)
        return None

    def state(self, timeout=1.0):
        return self.m.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=timeout
        )

    def send(self, p, v):
        mask = (
            0b0000_1100_0000_0000 | 0b0000_0001_1100_0000
        )  # ignore accel + yaw rate, use pos+vel+yaw
        self.m.mav.set_position_target_local_ned_send(
            0,
            self.m.target_system,
            self.m.target_component,
            1,
            mask,
            *p,
            *v,
            0,
            0,
            0,
            0,
            0,
        )

    def mode_offboard(self):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component, 176, 0, 1, 6, 0, 0, 0, 0, 0
        )

    def arm(self, on=True):
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            400,
            0,
            1 if on else 0,
            0 if on else 21196,
            0,
            0,
            0,
            0,
            0,
        )

    def land(self):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component, 21, 0, 0, 0, 0, 0, 0, 0, 0
        )

    def close(self):
        self._stop.set()
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            self.proc.wait(15)
        except Exception:  # noqa: BLE001
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        subprocess.run(["pkill", "-f", "gz sim"], capture_output=True)


def hold(sitl: Sitl, target, seconds, tol=None):
    """Stream a position hold for `seconds` of sim time; return final LOCAL_POSITION_NED."""
    msg = sitl.state()
    t_end = msg.time_boot_ms / 1000 + seconds
    while True:
        sitl.send(target, (0, 0, 0))
        msg = sitl.state()
        if msg is None:
            continue
        done = msg.time_boot_ms / 1000 >= t_end
        if tol is not None:
            done = (
                done
                and np.hypot(msg.x - target[0], msg.y - target[1]) < tol
                and abs(msg.vx) + abs(msg.vy) < 0.3
            )
        if done:
            return msg


def fly_window(sitl: Sitl, w: dict):
    sp_pos, sp_vel = np.array(w["sp_pos"]), np.array(w["sp_vel"])
    msg = sitl.state()
    anchor = np.array([msg.x, msg.y, msg.z])
    t0 = msg.time_boot_ms / 1000
    n = len(sp_pos)
    out_t, out_p = [], []
    while True:
        msg = sitl.state()
        if msg is None:
            continue
        t = msg.time_boot_ms / 1000 - t0
        i = int(t * RATE_HZ)
        if i >= n:
            break
        out_t.append(t)
        out_p.append([msg.x - anchor[0], msg.y - anchor[1], msg.z - anchor[2]])
        sitl.send(anchor + sp_pos[i], sp_vel[i])
    t, p = np.array(out_t), np.array(out_p)
    grid = np.arange(n) / RATE_HZ
    return np.column_stack([np.interp(grid, t, p[:, k]) for k in range(3)])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--autopilot", type=Path, required=True)
    ap.add_argument("--logs", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--max-windows", type=int, default=40)
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument(
        "--variant",
        default="",
        help="calibrated model, e.g. thrust=1.38,tau_up=0.08,drag=2.5",
    )
    ap.add_argument("--configs", default="default,log_tuning")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    windows = select_windows(args.logs, args.max_windows)
    (args.out / "windows.json").write_text(
        json.dumps(
            [{k: w[k] for k in ("log_id", "vehicle", "t0", "tuning")} for w in windows],
            indent=1,
        )
    )
    print("windows", len(windows), flush=True)
    results_path = args.out / "replay.jsonl"
    done = set()
    if results_path.exists():
        done = {
            (r["log_id"], r["t0"], r["config"])
            for r in map(json.loads, results_path.read_text().splitlines())
        }
    variant = parse_variant(args.variant)
    configs = [c for c in args.configs.split(",") if c in ("default", "log_tuning")]
    sitl = Sitl(args.autopilot, args.out, args.speed, variant)
    try:
        defaults = {k: sitl.param(k) for k in TUNING_KEYS}
        (args.out / "sitl_defaults.json").write_text(json.dumps(defaults, indent=1))
        home = np.array([0.0, 0.0, -ANCHOR_ALT_M])
        for _ in range(50):
            sitl.send(home, (0, 0, 0))
            time.sleep(0.02)
        for _attempt in range(20):
            sitl.mode_offboard()
            sitl.arm(True)
            hb = sitl.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            sitl.send(home, (0, 0, 0))
            if hb and hb.base_mode & 128:
                break
        hold(sitl, home, 8, tol=0.5)
        with results_path.open("a") as out:
            for w in windows:
                for config in configs:
                    if (w["log_id"], w["t0"], config) in done:
                        continue
                    values = (
                        defaults if config == "default" else {**defaults, **w["tuning"]}
                    )
                    for k, v in values.items():
                        if (
                            v is not None and k != "MPC_THR_HOVER"
                        ):  # hover thrust is physics, not tuning
                            sitl.param(k, v)
                    msg = sitl.state()
                    here = np.array([msg.x, msg.y, -ANCHOR_ALT_M])
                    if np.hypot(*here[:2]) > 60:
                        here = home
                    hold(sitl, here, 3, tol=0.5)
                    sim = fly_window(sitl, w)
                    out.write(
                        json.dumps(
                            {
                                "log_id": w["log_id"],
                                "vehicle": w["vehicle"],
                                "t0": w["t0"],
                                "config": config,
                                "variant": args.variant,
                                "sim_pos": np.round(sim, 4).tolist(),
                            }
                        )
                        + "\n"
                    )
                    out.flush()
                    print(w["log_id"][:8], config, "flown", flush=True)
        sitl.land()
        time.sleep(5)
    finally:
        sitl.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
