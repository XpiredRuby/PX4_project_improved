import numpy as np
import pytest

from fidelity import metrics, tracking_model
from fidelity.nanobench import Flight, active_segment, repair_spikes
from fidelity.quality import Gate, check, load_clean


def _synthetic_flight(model, name="synthetic_rep1", seconds=30.0, dt=0.01, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, seconds, dt)
    sp = np.column_stack(
        [
            0.8 * np.sin(0.6 * t + rng.uniform(0, 3)),
            0.8 * np.sin(0.9 * t + rng.uniform(0, 3)),
            1.0 + 0.3 * np.sin(0.4 * t + rng.uniform(0, 3)),
        ]
    )
    pos = np.empty_like(sp)
    vel = np.empty_like(sp)
    for k, axis in enumerate(model.axes):
        delayed = tracking_model._delayed(sp[:, k], int(round(axis.delay_s / dt)))
        p, v = sp[0, k], 0.0
        for i in range(len(t)):
            pos[i, k], vel[i, k] = p, v
            a = axis.wn2 * (delayed[i] - p) - axis.damping * v + axis.bias
            v += a * dt
            p += v * dt
    return Flight(name, "synthetic", "synthetic", 1, t, sp, pos, vel)


TRUE = tracking_model.TrackingModel(
    (
        tracking_model.AxisModel(4.0, 2.0, 0.0, 0.05),
        tracking_model.AxisModel(4.0, 2.0, 0.0, 0.05),
        tracking_model.AxisModel(9.0, 4.0, 0.0, 0.02),
    )
)


def test_fit_recovers_known_closed_loop_dynamics():
    flights = [_synthetic_flight(TRUE, f"s_rep{i}", seed=i) for i in range(3)]
    fitted = tracking_model.fit(flights)
    for got, want in zip(fitted.axes, TRUE.axes, strict=True):
        assert got.wn2 == pytest.approx(want.wn2, rel=0.10)
        assert got.damping == pytest.approx(want.damping, rel=0.15)
        assert got.delay_s == pytest.approx(want.delay_s, abs=0.02)
    held_out = _synthetic_flight(TRUE, "held_out", seed=99)
    err = metrics.error_norm(
        tracking_model.simulate(fitted, held_out), held_out.position
    )
    assert metrics.summarize(err)["rmse_m"] < 0.02


def test_model_beats_plan_is_flight_baseline_on_lagged_system():
    flight = _synthetic_flight(TRUE, seed=5)
    model_err = metrics.error_norm(
        tracking_model.simulate(TRUE, flight), flight.position
    )
    base_err = metrics.error_norm(
        tracking_model.plan_is_flight(flight), flight.position
    )
    assert (
        metrics.summarize(model_err)["rmse_m"]
        < 0.1 * metrics.summarize(base_err)["rmse_m"]
    )


def test_repair_spikes_fixes_isolated_sample_but_keeps_real_steps():
    sp = np.zeros((10, 3))
    sp[:, 2] = 1.0
    sp[4] = [0.6, 0.6, 0.4]  # one-sample logging spike
    sp[7:, 0] = 1.0  # genuine commanded step that persists
    fixed, n = repair_spikes(sp)
    assert n == 1
    assert np.allclose(fixed[4], [0.0, 0.0, 1.0])
    assert np.allclose(fixed[7:, 0], 1.0)


def test_active_segment_excludes_ground_and_landing_tail():
    sp = np.zeros((10, 3))
    sp[2:8, 2] = 1.0
    truth_z = np.array([0.0, 0.0, 0.1, 0.5, 0.9, 1.0, 1.0, 1.0, 0.3, 0.0])
    seg = active_segment(sp, truth_z)
    assert (seg.start, seg.stop) == (3, 8)


def test_quality_gate_rejects_setpoint_outside_arena():
    t = np.arange(0, 5, 0.01)
    sp = np.column_stack([np.zeros_like(t), np.zeros_like(t), np.ones_like(t)])
    sp[300:, 0] = 20.0
    flight = Flight(
        "bad", "x", "x", 1, t, sp, np.zeros_like(sp) + [0, 0, 1], np.zeros_like(sp)
    )
    assert any("leaves arena" in r for r in check(flight, Gate()))


def _write_csv(path, x_offset=0.0):
    t = np.arange(0, 6, 0.01) + 1000.0
    n = len(t)
    cols = {
        "t": t,
        "px": np.full(n, x_offset),
        "py": np.zeros(n),
        "pz": np.ones(n),
        "vx": np.zeros(n),
        "vy": np.zeros(n),
        "vz": np.zeros(n),
        "sp_ctrltarget_x": np.full(n, x_offset),
        "sp_ctrltarget_y": np.zeros(n),
        "sp_ctrltarget_z": np.ones(n),
    }
    header = ",".join(cols)
    rows = np.column_stack(list(cols.values()))
    np.savetxt(path, rows, delimiter=",", header=header, comments="", fmt="%.6f")


def test_load_clean_drops_byte_identical_duplicates(tmp_path):
    _write_csv(tmp_path / "A_shape_rep1.csv")
    (tmp_path / "A_shape_rep2.csv").write_bytes(
        (tmp_path / "A_shape_rep1.csv").read_bytes()
    )
    _write_csv(tmp_path / "A_shape_rep3.csv", x_offset=0.1)
    accepted, log = load_clean(tmp_path)
    assert [f.name for f in accepted] == ["A_shape_rep1", "A_shape_rep3"]
    assert any("duplicate of A_shape_rep1.csv" in r["reason"] for r in log)


def test_envelope_and_coverage():
    train = np.linspace(0, 1, 101)
    bound = metrics.envelope(train, 0.95)
    assert bound == pytest.approx(0.95)
    assert metrics.coverage(np.array([0.1, 0.5, 2.0, 0.9]), bound) == pytest.approx(
        0.75
    )
