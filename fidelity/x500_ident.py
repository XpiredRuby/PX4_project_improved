"""Identify X500 V2 dynamics from public Flight Review logs and score tracking predictions.

Referent: the onboard EKF2 estimate (GNSS-aided). Nothing here is scored against independent truth.
All rates in SI: rad/s, m, s. Body/NED conventions follow PX4.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

G = 9.80665
RPM_TO_RADS = 2.0 * np.pi / 60.0

# Default PX4 Gazebo x500 model (PX4-gazebo-models x500_base / x500 SDF).
GAZEBO_X500 = {
    "mass_kg": 2.0,
    "motor_constant": 8.54858e-06,  # N/(rad/s)^2
    "time_constant_up_s": 0.0125,
    "time_constant_down_s": 0.025,
    "rotor_drag_coefficient": 8.06428e-05,
}


def gazebo_hover_rad_s(p=GAZEBO_X500) -> float:
    return float(np.sqrt(p["mass_kg"] * G / (4.0 * p["motor_constant"])))


def gazebo_linear_drag_per_s(p=GAZEBO_X500) -> float:
    """Gazebo rotor drag: F = c * omega * v per rotor, so D/m = 4 c omega_hover / m (1/s)."""
    return 4.0 * p["rotor_drag_coefficient"] * gazebo_hover_rad_s(p) / p["mass_kg"]


def unique_logs(log_dir):
    """(log_id, vehicle, npz_path) for each distinct flight; byte-identical re-uploads are dropped."""
    import json
    from pathlib import Path

    log_dir = Path(log_dir)
    manifest = json.loads((log_dir / "manifest.json").read_text())
    seen, out = set(), []
    for log_id in sorted(manifest):
        meta = manifest[log_id]
        npz = log_dir / "npz" / f"{log_id}.npz"
        if meta.get("status") != "ok" or not npz.exists() or meta.get("sha256") in seen:
            continue
        seen.add(meta.get("sha256"))
        out.append((log_id, meta.get("vehicle", log_id), npz))
    return out


def resample(t_src, values, t_new):
    values = np.asarray(values, float)
    if values.ndim == 1:
        return np.interp(t_new, t_src, values)
    return np.column_stack(
        [np.interp(t_new, t_src, values[:, k]) for k in range(values.shape[1])]
    )


def quat_tilt(q: np.ndarray) -> np.ndarray:
    """Tilt angle (rad) between body z and vertical from PX4 quaternion (w, x, y, z)."""
    w, x, y, z = q.T
    cos_tilt = 1.0 - 2.0 * (x * x + y * y)
    return np.arccos(np.clip(cos_tilt, -1.0, 1.0))


@dataclass
class Segment:
    log_id: str
    vehicle: str
    t: np.ndarray
    pos: np.ndarray  # EKF2 NED position
    vel: np.ndarray
    sp_pos: np.ndarray  # setpoint position (NaN where velocity-only)
    sp_vel: np.ndarray


def offboard_segments(
    log_id: str, vehicle: str, d: dict, rate_hz: float = 50.0, min_s: float = 15.0
) -> list[Segment]:
    from .px4_logs import segments

    if "sp" not in d or "nav" not in d:
        return []
    out = []
    for a, b in segments(d["nav_t"], d["nav"], float(d["lpos_t"][-1]), min_s=min_s):
        a, b = (
            max(a, d["sp_t"][0], d["lpos_t"][0]) + 1.0,
            min(b, d["sp_t"][-1], d["lpos_t"][-1]) - 0.5,
        )
        if b - a < min_s:
            continue
        t = np.arange(a, b, 1.0 / rate_hz)
        lp = resample(d["lpos_t"], d["lpos"], t)
        # Setpoints are zero-order-hold signals: sample the most recent one.
        idx = np.clip(
            np.searchsorted(d["sp_t"], t, side="right") - 1, 0, len(d["sp_t"]) - 1
        )
        sp = d["sp"][idx]
        if "lpos_xy_valid" in d:
            valid = resample(d["lpos_t"], d["lpos_xy_valid"], t) > 0.5
            if valid.mean() < 0.99:
                continue
        if not np.all(np.isfinite(lp)):
            continue
        out.append(
            Segment(log_id, vehicle, t, lp[:, :3], lp[:, 3:], sp[:, :3], sp[:, 3:6])
        )
    return out


def position_tracked(seg: Segment) -> bool:
    return bool(np.isfinite(seg.sp_pos).all())


# --- closed-loop tracking model (PX4 position controller + airframe as seen from setpoints) ---------


@dataclass(frozen=True)
class AxisFit:
    kp: float  # 1/s^2 on position error
    kd: float  # 1/s on velocity error (setpoint velocity feed-forward minus velocity)
    bias: float
    delay_s: float


def _delay(x, steps):
    return (
        x
        if steps <= 0
        else np.concatenate([np.repeat(x[:1], steps, axis=0), x[:-steps]])
    )


def _rows(segs, axis, steps):
    X, y = [], []
    for s in segs:
        dt = s.t[1] - s.t[0]
        spp, spv = (
            _delay(s.sp_pos[:, axis], steps),
            _delay(np.nan_to_num(s.sp_vel[:, axis]), steps),
        )
        p, v = s.pos[:, axis], s.vel[:, axis]
        a = np.convolve(np.gradient(v, dt), np.ones(5) / 5, mode="same")
        k = slice(5, len(p) - 5)
        X.append(np.column_stack([spp - p, spv - v, np.ones_like(p)])[k])
        y.append(a[k])
    return np.vstack(X), np.concatenate(y)


def fit_axis(segs, axis, max_delay_s=0.3) -> AxisFit:
    dt = segs[0].t[1] - segs[0].t[0]
    best = None
    for steps in range(int(max_delay_s / dt) + 1):
        X, y = _rows(segs, axis, steps)
        th, *_ = np.linalg.lstsq(X, y, rcond=None)
        r = float(np.mean((X @ th - y) ** 2))
        if best is None or r < best[0]:
            best = (r, steps, th)
    _, steps, th = best
    return AxisFit(float(th[0]), float(th[1]), float(th[2]), steps * dt)


def fit_tracking(segs) -> tuple[AxisFit, AxisFit, AxisFit]:
    return tuple(fit_axis(segs, k) for k in range(3))


def simulate(fits, seg: Segment, start: int, n: int) -> np.ndarray:
    """Forward-simulate n samples from the true state at index start, driven only by setpoints."""
    dt = seg.t[1] - seg.t[0]
    out = np.empty((n, 3))
    for k, f in enumerate(fits):
        steps = int(round(f.delay_s / dt))
        spp = _delay(seg.sp_pos[:, k], steps)[start : start + n]
        spv = _delay(np.nan_to_num(seg.sp_vel[:, k]), steps)[start : start + n]
        p, v = seg.pos[start, k], seg.vel[start, k]
        for i in range(n):
            out[i, k] = p
            a = f.kp * (spp[i] - p) + f.kd * (spv[i] - v) + f.bias
            v += a * dt
            p += v * dt
    return out


def window_errors(fits, segs, window_s=20.0):
    """Open-loop multi-step prediction from the true state at each window start; returns per-sample errors."""
    model_err, base_err, groups = [], [], []
    for s in segs:
        dt = s.t[1] - s.t[0]
        n = int(window_s / dt)
        for start in range(0, len(s.t) - n, n):
            pred = simulate(fits, s, start, n)
            truth = s.pos[start : start + n]
            model_err.append(np.linalg.norm(pred - truth, axis=1))
            base_err.append(np.linalg.norm(s.sp_pos[start : start + n] - truth, axis=1))
            groups.append(s.vehicle)
    return model_err, base_err, groups


# --- propulsion -----------------------------------------------------------------------------------


def hover_samples(d: dict, rate_hz: float = 20.0):
    """Samples where the vehicle is airborne, slow, and not accelerating: mean thrust ~= weight."""
    if "esc_rpm" not in d:
        return None
    t = np.arange(d["lpos_t"][0], d["lpos_t"][-1], 1.0 / rate_hz)
    lp = resample(d["lpos_t"], d["lpos"], t)
    vel = lp[:, 3:]
    acc = np.gradient(vel, 1.0 / rate_hz, axis=0)
    rpm = resample(d["esc_t"], d["esc_rpm"][:, :4], t)
    airborne = -lp[:, 2] > 1.0
    calm = (np.linalg.norm(vel, axis=1) < 0.3) & (np.linalg.norm(acc, axis=1) < 0.3)
    spinning = np.all(rpm > 1000, axis=1)
    keep = airborne & calm & spinning
    if keep.sum() < rate_hz * 5:
        return None
    return t[keep], rpm[keep]


def hover_omega(d: dict) -> float | None:
    hs = hover_samples(d)
    if hs is None:
        return None
    _, rpm = hs
    # Mean of squares: thrust is proportional to omega^2.
    return float(np.sqrt(np.median(np.mean((rpm * RPM_TO_RADS) ** 2, axis=1))))


def motor_time_constant(d: dict, rate_hz: float = 100.0, max_tau=0.25) -> float | None:
    """First-order lag from motor command to measured RPM: omega' = (f(u) - omega)/tau, f affine in u."""
    if "esc_rpm" not in d or "mot" not in d:
        return None
    t0 = max(d["esc_t"][0], d["mot_t"][0])
    t1 = min(d["esc_t"][-1], d["mot_t"][-1])
    if t1 - t0 < 20:
        return None
    t = np.arange(t0, t1, 1.0 / rate_hz)
    u = resample(d["mot_t"], d["mot"], t)
    w = resample(d["esc_t"], d["esc_rpm"][:, :4], t) * RPM_TO_RADS
    on = np.all(w > 100, axis=1) & np.all(u > 0.05, axis=1)
    if on.mean() < 0.3:
        return None
    esc_rate = 1.0 / np.median(np.diff(d["esc_t"]))
    if esc_rate < 20:
        return None  # too coarse to resolve tens of milliseconds
    best = None
    for tau in np.arange(0.0, max_tau + 1e-9, 0.005):
        alpha = 1.0 if tau == 0 else 1.0 - np.exp(-1.0 / (rate_hz * tau))
        sse = 0.0
        for k in range(4):
            # Affine static map identified with least squares on the filtered command.
            f = np.empty_like(u[:, k])
            acc = u[0, k]
            for i, x in enumerate(u[:, k]):
                acc += alpha * (x - acc)
                f[i] = acc
            A = np.column_stack([f[on], np.ones(on.sum())])
            coef, *_ = np.linalg.lstsq(A, w[on, k], rcond=None)
            sse += float(np.sum((A @ coef - w[on, k]) ** 2))
        if best is None or sse < best[0]:
            best = (sse, tau)
    return float(best[1])


def drag_per_mass(d: dict, rate_hz: float = 20.0):
    """Steady level flight: horizontal drag acceleration = g*tan(tilt). Returns (speeds, drag_acc)."""
    if "att_q" not in d:
        return None
    t = np.arange(d["lpos_t"][0], d["lpos_t"][-1], 1.0 / rate_hz)
    lp = resample(d["lpos_t"], d["lpos"], t)
    q = resample(d["att_t"], d["att_q"], t)
    vh = np.linalg.norm(lp[:, 3:5], axis=1)
    acc = np.gradient(lp[:, 3:], 1.0 / rate_hz, axis=0)
    steady = (
        (vh > 1.5)
        & (np.linalg.norm(acc, axis=1) < 0.25)
        & (np.abs(lp[:, 5]) < 0.3)
        & (-lp[:, 2] > 2.0)
    )
    if steady.sum() < rate_hz * 5:
        return None
    return vh[steady], G * np.tan(quat_tilt(q[steady]))


def fit_drag(speeds, drag_acc):
    """D/m = k1 V + k2 V^2 (k >= 0 not enforced; reported as identified)."""
    A = np.column_stack([speeds, speeds**2])
    coef, *_ = np.linalg.lstsq(A, drag_acc, rcond=None)
    return float(coef[0]), float(coef[1])


# --- output-error fit (always-stable second-order closed loop) -------------------------------------


@dataclass(frozen=True)
class Loop:
    wn: float  # rad/s
    zeta: float
    delay_s: float

    def to_dict(self):
        return {"wn_rad_s": self.wn, "zeta": self.zeta, "delay_s": self.delay_s}


def windows(segs, axes, window_s=20.0, rate_hz=25.0):
    """Stack fixed-length windows: setpoint position/velocity, initial state, truth. Shape (W, N, len(axes))."""
    spp, spv, pos, vel, groups = [], [], [], [], []
    for s in segs:
        step = max(1, int(round((1.0 / (s.t[1] - s.t[0])) / rate_hz)))
        t = s.t[::step]
        n = int(window_s * rate_hz)
        for start in range(0, len(t) - n, n):
            sl = slice(start * step, (start + n) * step, step)
            spp.append(s.sp_pos[sl][:, axes])
            spv.append(np.nan_to_num(s.sp_vel[sl][:, axes]))
            pos.append(s.pos[sl][:, axes])
            vel.append(s.vel[sl][:, axes])
            groups.append(s.vehicle)
    return {
        k: np.array(v)
        for k, v in (("spp", spp), ("spv", spv), ("pos", pos), ("vel", vel))
    } | {"groups": np.array(groups), "dt": 1.0 / rate_hz}


def simulate_windows(w, loop: Loop, rows=None) -> np.ndarray:
    rows = slice(None) if rows is None else rows
    spp, spv, pos, vel, dt = (
        w["spp"][rows],
        w["spv"][rows],
        w["pos"][rows],
        w["vel"][rows],
        w["dt"],
    )
    d = int(round(loop.delay_s / dt))
    if d:
        spp = np.concatenate([np.repeat(spp[:, :1], d, 1), spp[:, :-d]], 1)
        spv = np.concatenate([np.repeat(spv[:, :1], d, 1), spv[:, :-d]], 1)
    kp, kd = loop.wn**2, 2.0 * loop.zeta * loop.wn
    p, v = pos[:, 0].copy(), vel[:, 0].copy()
    out = np.empty_like(pos)
    for i in range(pos.shape[1]):
        out[:, i] = p
        a = kp * (spp[:, i] - p) + kd * (spv[:, i] - v)
        v = v + a * dt
        p = p + v * dt
    return out


def fit_loop(w, rows=None, wn_grid=None, zeta_grid=None, delay_grid=None) -> Loop:
    wn_grid = np.arange(0.5, 6.01, 0.25) if wn_grid is None else wn_grid
    zeta_grid = np.arange(0.4, 1.61, 0.1) if zeta_grid is None else zeta_grid
    delay_grid = np.arange(0.0, 0.41, 0.04) if delay_grid is None else delay_grid
    rows = slice(None) if rows is None else rows
    truth = w["pos"][rows]
    best = None
    for wn in wn_grid:
        for zeta in zeta_grid:
            for delay in delay_grid:
                loop = Loop(float(wn), float(zeta), float(delay))
                err = float(np.mean((simulate_windows(w, loop, rows) - truth) ** 2))
                if best is None or err < best[0]:
                    best = (err, loop)
    return best[1]


def as_dict(fits):
    return {ax: asdict(f) for ax, f in zip("xyz", fits, strict=True)}
