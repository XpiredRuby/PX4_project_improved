import json

import numpy as np
import pytest

from fidelity import x500_ident as xi
from fidelity.px4_logs import NAV_OFFBOARD, segments, select_candidates


def _windows(loop, n_win=12, seconds=20.0, rate=25.0, seed=0):
    rng = np.random.default_rng(seed)
    n = int(seconds * rate)
    t = np.arange(n) / rate
    spp = np.stack(
        [
            np.column_stack(
                [
                    3 * np.sin(rng.uniform(0.2, 0.6) * t + rng.uniform(0, 6))
                    for _ in range(2)
                ]
            )
            for _ in range(n_win)
        ]
    )
    spv = np.gradient(spp, 1 / rate, axis=1)
    w = {
        "spp": spp,
        "spv": spv,
        "pos": np.zeros_like(spp),
        "vel": np.zeros_like(spp),
        "dt": 1 / rate,
    }
    w["pos"][:, 0], w["vel"][:, 0] = spp[:, 0], spv[:, 0]
    w["pos"] = xi.simulate_windows(w, loop)
    w["vel"] = np.gradient(w["pos"], 1 / rate, axis=1)
    w["groups"] = np.array(["v%d" % (i % 4) for i in range(n_win)])
    return w


def test_output_error_fit_recovers_closed_loop():
    true = xi.Loop(1.5, 0.8, 0.08)
    fit = xi.fit_loop(_windows(true))
    assert fit.wn == pytest.approx(1.5, abs=0.26)
    assert fit.zeta == pytest.approx(0.8, abs=0.11)
    assert fit.delay_s == pytest.approx(0.08, abs=0.041)


def test_simulated_loop_is_stable_and_tracks():
    w = _windows(xi.Loop(1.25, 0.7, 0.0), seed=3)
    err = np.linalg.norm(w["pos"] - w["spp"], axis=2)
    assert np.isfinite(err).all() and err.max() < 3.0


def test_drag_fit_recovers_linear_and_quadratic_terms():
    v = np.linspace(1.5, 10, 400)
    a = 0.3 * v + 0.02 * v**2
    k1, k2 = xi.fit_drag(v, a)
    assert (k1, k2) == pytest.approx((0.3, 0.02), rel=1e-6)


def test_gazebo_reference_numbers():
    assert xi.gazebo_hover_rad_s() == pytest.approx(757.35, abs=0.5)
    assert xi.quat_tilt(np.array([[1.0, 0, 0, 0]]))[0] == 0.0


def test_offboard_segments_split_on_mode_changes():
    nav_t = np.array([0.0, 5.0, 40.0, 50.0, 90.0])
    nav = np.array([2, NAV_OFFBOARD, 2, NAV_OFFBOARD, 2])
    assert segments(nav_t, nav, 100.0, min_s=10) == [(5.0, 40.0), (50.0, 90.0)]


def test_candidate_selection_filters_sitl_versions_and_caps_per_vehicle():
    base = {
        "airframe_name": "Holybro X500 V2",
        "sys_hw": "PX4_FMU_V6X",
        "ver_sw_release": "v1.15.4",
        "download_url": "/download?log=x",
        "flight_mode_durations": [[NAV_OFFBOARD, 60]],
    }
    entries = [dict(base, log_id=f"a{i}", vehicle_uuid="same") for i in range(5)]
    entries += [
        dict(base, log_id="sitl", sys_hw="PX4_SITL"),
        dict(base, log_id="old", ver_sw_release="v1.12.3"),
        dict(base, log_id="short", flight_mode_durations=[[NAV_OFFBOARD, 5]]),
    ]
    picked = select_candidates(entries, per_vehicle=3)
    assert [p["log_id"] for p in picked] == ["a0", "a1", "a2"]


def test_unique_logs_drops_reuploads(tmp_path):
    (tmp_path / "npz").mkdir()
    for name in ("a", "b", "c"):
        np.savez(tmp_path / "npz" / f"{name}.npz", x=np.zeros(1))
    manifest = {
        "a": {"status": "ok", "sha256": "1", "vehicle": "v"},
        "b": {"status": "ok", "sha256": "1", "vehicle": "v"},
        "c": {"status": "ok", "sha256": "2", "vehicle": "w"},
    }
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    assert [x[0] for x in xi.unique_logs(tmp_path)] == ["a", "c"]
