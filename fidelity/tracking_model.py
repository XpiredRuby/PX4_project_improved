"""Closed-loop tracking model: per-axis second-order response to a delayed setpoint.

    a = wn^2 * (sp(t - tau) - p) - 2*zeta*wn * v + c

Parameters are identified by least squares on training flights only; predictions are
open-loop forward simulations from the initial truth state, so error is not reset by
truth along the way (multi-step prediction, the honest test for a preflight tool).
"""

from __future__ import annotations

from dataclasses import dataclass, asdict

import numpy as np

from .nanobench import Flight


@dataclass(frozen=True)
class AxisModel:
    wn2: float  # wn^2, 1/s^2
    damping: float  # 2*zeta*wn, 1/s
    bias: float  # m/s^2
    delay_s: float

    @property
    def wn(self) -> float:
        return float(np.sqrt(max(self.wn2, 0.0)))

    @property
    def zeta(self) -> float:
        return float(self.damping / (2.0 * self.wn)) if self.wn > 0 else float("nan")


@dataclass(frozen=True)
class TrackingModel:
    axes: tuple[AxisModel, AxisModel, AxisModel]

    def to_dict(self) -> dict:
        names = ("x", "y", "z")
        return {
            n: {**asdict(a), "wn": a.wn, "zeta": a.zeta}
            for n, a in zip(names, self.axes, strict=True)
        }


def _delayed(signal: np.ndarray, delay_steps: int) -> np.ndarray:
    if delay_steps <= 0:
        return signal
    return np.concatenate([np.full(delay_steps, signal[0]), signal[:-delay_steps]])


def _acceleration(velocity: np.ndarray, dt: float) -> np.ndarray:
    acc = np.gradient(velocity, dt)
    kernel = (
        np.ones(5) / 5.0
    )  # light smoothing; Vicon velocity is already SG-differentiated
    return np.convolve(acc, kernel, mode="same")


def _regression_rows(flights, axis, delay_steps):
    rows, targets = [], []
    for f in flights:
        sp = _delayed(f.setpoint[:, axis], delay_steps)
        p, v = f.position[:, axis], f.velocity[:, axis]
        a = _acceleration(v, f.dt)
        trim = slice(5, len(p) - 5)
        rows.append(np.column_stack([sp - p, -v, np.ones_like(p)])[trim])
        targets.append(a[trim])
    return np.vstack(rows), np.concatenate(targets)


def fit_axis(flights, axis: int, max_delay_s: float = 0.30) -> AxisModel:
    if not flights:
        raise ValueError("no training flights")
    dt = float(np.median([f.dt for f in flights]))
    best = None
    for steps in range(int(round(max_delay_s / dt)) + 1):
        X, y = _regression_rows(flights, axis, steps)
        theta, *_ = np.linalg.lstsq(X, y, rcond=None)
        resid = float(np.mean((X @ theta - y) ** 2))
        if best is None or resid < best[0]:
            best = (resid, steps, theta)
    _, steps, (wn2, damping, bias) = best
    return AxisModel(float(wn2), float(damping), float(bias), steps * dt)


def fit(flights) -> TrackingModel:
    return TrackingModel(tuple(fit_axis(flights, k) for k in range(3)))


def simulate(model: TrackingModel, flight: Flight) -> np.ndarray:
    """Forward-simulate positions from the flight's setpoints and initial truth state."""
    n, dt = len(flight.t), flight.dt
    out = np.empty((n, 3))
    for k, axis in enumerate(model.axes):
        sp = _delayed(flight.setpoint[:, k], int(round(axis.delay_s / dt)))
        p, v = flight.position[0, k], flight.velocity[0, k]
        for i in range(n):
            out[i, k] = p
            a = axis.wn2 * (sp[i] - p) - axis.damping * v + axis.bias
            v += a * dt
            p += v * dt
    return out


def plan_is_flight(flight: Flight) -> np.ndarray:
    """Baseline prediction: the vehicle flies exactly the commanded setpoint."""
    return flight.setpoint.copy()
