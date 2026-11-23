"""Score real Holybro X500 V2 flights (public PX4 Flight Review logs) against the Studio models and Gazebo.

Produces OUT/results.json, OUT/README.md, OUT/*.png, and the Studio envelope file config/fidelity/x500_envelope.json.
Referent everywhere: the real vehicle's onboard EKF2 estimate (GNSS-aided), not independent truth.
Usage: python scripts/score_x500_logs.py --logs ~/validation-data/x500 --sitl ~/validation-data/sitl \
           --replay ~/validation-data/replay --out validation/2026-11-23-x500-public-logs
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from fidelity import metrics  # noqa: E402
from fidelity import x500_ident as xi  # noqa: E402

SPEED_BANDS = ((0.0, 2.0, "slow"), (2.0, 99.0, "fast"))
HORIZONS_S = (1, 2, 5, 10, 15, 20)


def band_of(speed):
    return next(name for lo, hi, name in SPEED_BANDS if lo <= speed < hi)


def tracking(log_dir: Path):
    segs = []
    for log_id, vehicle, npz in xi.unique_logs(log_dir):
        d = dict(np.load(npz))
        segs += [
            s
            for s in xi.offboard_segments(log_id, vehicle, d)
            if xi.position_tracked(s)
        ]
    segs = [
        s
        for s in segs
        if np.percentile(np.linalg.norm(s.sp_pos - s.pos, axis=1), 95) <= 20
    ]
    wh, wz = xi.windows(segs, [0, 1]), xi.windows(segs, [2])
    speeds = np.linalg.norm(wh["vel"], axis=2).max(axis=1)
    bands = np.array([band_of(v) for v in speeds])
    vehicles = sorted(set(wh["groups"]))
    rng = np.random.default_rng(0)
    rng.shuffle(vehicles)
    folds = [set(vehicles[i::5]) for i in range(5)]
    err_model = np.zeros(wh["pos"].shape[:2])
    err_base = np.zeros_like(err_model)
    fold_rows = []
    for fv in folds:
        te = np.array([g in fv for g in wh["groups"]])
        lh, lz = (
            xi.fit_loop(wh, ~te),
            xi.fit_loop(
                wz,
                ~te,
                wn_grid=np.arange(0.25, 4.01, 0.25),
                zeta_grid=np.arange(0.4, 3.01, 0.2),
            ),
        )
        pred = np.concatenate(
            [xi.simulate_windows(wh, lh, te), xi.simulate_windows(wz, lz, te)], 2
        )
        truth = np.concatenate([wh["pos"][te], wz["pos"][te]], 2)
        err_model[te] = np.linalg.norm(pred - truth, axis=2)
        spp = np.concatenate([wh["spp"][te], wz["spp"][te]], 2)
        err_base[te] = np.linalg.norm(spp - truth, axis=2)
        fold_rows.append(
            {
                "test_vehicles": len(fv),
                "horizontal": lh.to_dict(),
                "vertical": lz.to_dict(),
            }
        )
    # Final model on all data (what Studio uses).
    lh_all = xi.fit_loop(wh)
    lz_all = xi.fit_loop(
        wz, wn_grid=np.arange(0.25, 4.01, 0.25), zeta_grid=np.arange(0.4, 3.01, 0.2)
    )
    # Envelope per band and horizon: split-conformal across vehicle folds (each window's error is out-of-fold).
    dt = wh["dt"]
    envelope, coverage = {}, {}
    for _, _, name in SPEED_BANDS:
        rows = bands == name
        if rows.sum() < 20:
            envelope[name] = None
            continue
        envelope[name] = {}
        coverage[name] = {}
        for h in HORIZONS_S:
            k = min(int(h / dt), err_model.shape[1]) - 1
            e = err_model[rows, : k + 1].max(axis=1)  # worst error up to horizon h
            # Calibrate on half the vehicles, test coverage on the other half, then publish the full-data bound.
            g = wh["groups"][rows]
            cal = np.isin(g, sorted(set(g))[::2])
            bound_half = float(
                np.quantile(e[cal], min(1.0, 0.95 * (1 + 1 / max(cal.sum(), 1))))
            )
            coverage[name][str(h)] = float(np.mean(e[~cal] <= bound_half))
            envelope[name][str(h)] = float(
                np.quantile(e, min(1.0, 0.95 * (1 + 1 / len(e))))
            )
    return {
        "segments": len(segs),
        "windows": int(len(wh["groups"])),
        "vehicles": len(vehicles),
        "minutes": round(sum(s.t[-1] - s.t[0] for s in segs) / 60, 1),
        "band_windows": {n: int(np.sum(bands == n)) for _, _, n in SPEED_BANDS},
        "heldout_model": metrics.summarize(err_model.ravel()),
        "heldout_plan_is_flight": metrics.summarize(err_base.ravel()),
        "folds": fold_rows,
        "loop_all": {"horizontal": lh_all.to_dict(), "vertical": lz_all.to_dict()},
        "envelope_95_m": envelope,
        "envelope_heldout_coverage": coverage,
        "calibrated_speed_max_m_s": float(np.percentile(speeds, 99)),
    }, (err_model, err_base, dt)


def hover_and_drag(log_dir: Path, sitl_dir: Path):
    real_hover, real_v, real_a, mdl = [], [], [], []
    for log_id, _vehicle, npz in xi.unique_logs(log_dir):
        d = dict(np.load(npz))
        if "hover" in d and (d["hover"] > 0.05).any():
            real_hover.append(float(np.median(d["hover"][d["hover"] > 0.05])))
        dg = xi.drag_per_mass(d)
        if dg:
            real_v.append(dg[0])
            real_a.append(dg[1])
        params = json.loads((log_dir / "npz" / f"{log_id}.params.json").read_text())
        if "THR_MDL_FAC" in params:
            mdl.append(params["THR_MDL_FAC"])
    sitl_hover, sv, sa = [], [], []
    man = json.loads((sitl_dir / "manifest.json").read_text())
    for meta in man.values():
        if not isinstance(meta, dict) or meta.get("autostart") != 4001:
            continue
        d = dict(np.load(sitl_dir / "npz" / f"{meta['key']}.npz"))
        if "hover" in d and (d["hover"] > 0.05).any():
            sitl_hover.append(float(np.median(d["hover"][d["hover"] > 0.05])))
        dg = xi.drag_per_mass(d)
        if dg:
            sv.append(dg[0])
            sa.append(dg[1])
    rv, ra = np.concatenate(real_v), np.concatenate(real_a)
    sv, sa = np.concatenate(sv), np.concatenate(sa)

    def binned(v, a):
        edges = np.arange(1.5, 12.01, 1.0)
        rows = []
        for lo, hi in zip(edges[:-1], edges[1:], strict=True):
            m = (v >= lo) & (v < hi)
            if m.sum() > 200:
                rows.append(
                    {
                        "speed_m_s": float((lo + hi) / 2),
                        "drag_acc_m_s2_p50": float(np.median(a[m])),
                        "p25": float(np.percentile(a[m], 25)),
                        "p75": float(np.percentile(a[m], 75)),
                        "samples": int(m.sum()),
                    }
                )
        return rows

    real_h = np.array(real_hover)
    return {
        "hover_thrust_real": {
            "flights": len(real_h),
            "p10": float(np.percentile(real_h, 10)),
            "p50": float(np.median(real_h)),
            "p90": float(np.percentile(real_h, 90)),
        },
        "hover_thrust_sitl": {
            "flights": len(sitl_hover),
            "p50": float(np.median(sitl_hover)),
        },
        "thr_mdl_fac_real": {
            "p50": float(np.median(mdl)) if mdl else None,
            "flights": len(mdl),
        },
        "motor_constant_scale_to_match_fleet_median": float(
            (np.median(sitl_hover) / np.median(real_h)) ** 2
        ),
        "drag_real_linear_quadratic": xi.fit_drag(rv, ra),
        "drag_sitl_linear_quadratic": xi.fit_drag(sv, sa),
        "drag_real_binned": binned(rv, ra),
        "drag_sitl_binned": binned(sv, sa),
        "drag_note": "D/m from steady-level tilt, g*tan(tilt). Real values include unmeasured wind (bias and spread).",
    }


def propulsion(esc_dir: Path):
    """Hover rotor speed and motor lag from public logs that carry ESC RPM (deduplicated by content)."""
    import hashlib

    rows, seen = [], set()
    for npz in sorted(Path(esc_dir).glob("*.npz")):
        d = dict(np.load(npz))
        key = hashlib.sha256(np.round(d["esc_rpm"][:4000], 0).tobytes()).hexdigest()
        if key in seen:
            continue
        seen.add(key)
        w = xi.hover_omega(d)
        if w is None:
            continue
        rows.append(
            {
                "log_id": npz.stem,
                "hover_rpm": w / xi.RPM_TO_RADS,
                "k_t_per_mass": xi.G / (4 * w * w),
                "motor_time_constant_s": xi.motor_time_constant(d),
            }
        )
    taus = [
        r["motor_time_constant_s"]
        for r in rows
        if r["motor_time_constant_s"] is not None
    ]
    return {
        "flights": rows,
        "hover_rpm_range": [
            min(r["hover_rpm"] for r in rows),
            max(r["hover_rpm"] for r in rows),
        ]
        if rows
        else None,
        "motor_time_constant_range_s": [min(taus), max(taus)] if taus else None,
        "gazebo_hover_rpm": xi.gazebo_hover_rad_s() / xi.RPM_TO_RADS,
        "gazebo_k_t_per_mass": xi.GAZEBO_X500["motor_constant"]
        / xi.GAZEBO_X500["mass_kg"],
        "gazebo_motor_time_constant_up_s": xi.GAZEBO_X500["time_constant_up_s"],
    }


def replay(replay_dir: Path, log_dir: Path, loop):
    path = replay_dir / "replay.jsonl"
    if not path.exists():
        return None
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "sitl_replay", ROOT / "scripts/sitl_replay.py"
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    windows = {(w["log_id"], w["t0"]): w for w in mod.select_windows(log_dir, 10_000)}
    lead = int(mod.LEAD_S * mod.RATE_HZ)
    rows = {"default": [], "log_tuning": [], "studio_model": [], "plan_is_flight": []}
    per_window = []
    for rec in map(json.loads, path.read_text().splitlines()):
        w = windows.get((rec["log_id"], rec["t0"]))
        if w is None:
            continue
        sim, real, sp = (
            np.array(rec["sim_pos"]),
            np.array(w["real_pos"]),
            np.array(w["sp_pos"]),
        )
        s = sim[lead:] - sim[lead]
        r = real[lead:] - real[lead]
        e = np.linalg.norm(s - r, axis=1)
        rows[rec["config"]].append(e)
        if rec["config"] == "default":
            rows["plan_is_flight"].append(
                np.linalg.norm((sp[lead:] - real[lead]) - r, axis=1)
            )
            # Studio loop model driven by the same setpoints from the real state at window start.
            seg = xi.Segment(
                rec["log_id"],
                rec["vehicle"],
                np.arange(len(real)) / mod.RATE_HZ,
                real,
                np.gradient(real, 1 / mod.RATE_HZ, axis=0),
                sp,
                np.array(w["sp_vel"]),
            )
            wn = {
                "spp": sp[None, lead:],
                "spv": np.array(w["sp_vel"])[None, lead:],
                "pos": real[None, lead:],
                "vel": seg.vel[None, lead:],
                "dt": 1 / mod.RATE_HZ,
            }
            pred = np.concatenate(
                [
                    xi.simulate_windows(
                        {k: (v[..., :2] if k != "dt" else v) for k, v in wn.items()},
                        loop[0],
                    ),
                    xi.simulate_windows(
                        {k: (v[..., 2:] if k != "dt" else v) for k, v in wn.items()},
                        loop[1],
                    ),
                ],
                2,
            )[0]
            rows["studio_model"].append(np.linalg.norm((pred - real[lead]) - r, axis=1))
        per_window.append(
            {
                "log_id": rec["log_id"],
                "config": rec["config"],
                "rmse_m": float(np.sqrt(np.mean(e**2))),
            }
        )
    out = {k: metrics.summarize(np.concatenate(v)) for k, v in rows.items() if v}
    out["windows"] = len(rows["default"])
    out["per_window"] = per_window
    return out


def figures(out: Path, track_err, hd):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    err_model, err_base, dt = track_err
    t = np.arange(err_model.shape[1]) * dt
    fig, ax = plt.subplots(figsize=(7, 4))
    for e, label, c in (
        (err_base, "Plan = flight (no model)", "#9aa0a6"),
        (err_model, "Studio closed-loop model", "#e8a33d"),
    ):
        ax.plot(t, np.median(e, 0), color=c, label=label + " · median")
        ax.fill_between(
            t, np.percentile(e, 5, 0), np.percentile(e, 95, 0), color=c, alpha=0.18
        )
    ax.set_xlabel("Seconds into prediction")
    ax.set_ylabel("Position error vs real EKF2 (m)")
    ax.set_title("Held-out real X500 V2 flights: prediction error growth")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "x500_prediction_error.png", dpi=130)
    fig, ax = plt.subplots(figsize=(7, 4))
    for rows, label, c in (
        (hd["drag_real_binned"], "Real X500 V2 fleet", "#e8a33d"),
        (hd["drag_sitl_binned"], "Gazebo x500 (SITL)", "#5b8def"),
    ):
        v = [r["speed_m_s"] for r in rows]
        ax.plot(v, [r["drag_acc_m_s2_p50"] for r in rows], "o-", color=c, label=label)
        ax.fill_between(
            v, [r["p25"] for r in rows], [r["p75"] for r in rows], color=c, alpha=0.2
        )
    ax.set_xlabel("Horizontal speed (m/s)")
    ax.set_ylabel("Drag per mass, g·tan(tilt) (m/s²)")
    ax.set_title("Steady-flight drag: real fleet vs simulator")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "x500_drag_real_vs_sim.png", dpi=130)
    plt.close("all")


def evidence(track, hd, rp):
    """Plain-language summary consumed by the Studio 'Accuracy evidence' panel."""
    m, b = track["heldout_model"], track["heldout_plan_is_flight"]
    slow = track["envelope_95_m"]["slow"]
    cov = track["envelope_heldout_coverage"]["slow"]
    head = [
        {
            "label": "Real X500 V2 aircraft",
            "value": str(track["vehicles"]),
            "note": f"{track['minutes']:.0f} min of Offboard flight",
        },
        {
            "label": "Median prediction error",
            "value": f"{m['p50_m']:.2f} m",
            "note": f"vs {b['p50_m']:.2f} m if the plan is assumed exact",
        },
        {
            "label": "95% bound, 20 s, < 2 m/s",
            "value": f"±{slow['20']:.2f} m",
            "note": f"held-out coverage {100 * cov['20']:.0f}%",
        },
        {
            "label": "Calibrated speed range",
            "value": f"≤ {track['calibrated_speed_max_m_s']:.1f} m/s",
            "note": "faster plans are flagged",
        },
    ]
    if rp and "default" in rp and "log_tuning" in rp:
        head.append(
            {
                "label": "Gazebo vs real, same setpoints (p95)",
                "value": f"{rp['default']['p95_m']:.1f} m → {rp['log_tuning']['p95_m']:.1f} m",
                "note": "default tuning → the real aircraft's own PX4 tuning",
            }
        )
    head.append(
        {
            "label": "Hover thrust, real vs sim",
            "value": f"{100 * hd['hover_thrust_real']['p50']:.0f}% / {100 * hd['hover_thrust_sitl']['p50']:.0f}%",
            "note": "default sim x500 has less thrust margin",
        }
    )
    cred = [
        {
            "factor": "Verification",
            "score": 3,
            "why": "Unit, regression and lint suites; deterministic replay; envelope maths tested.",
        },
        {
            "factor": "Validation",
            "score": 2,
            "why": "Held-out real X500 flights, but referent is onboard EKF2, not independent truth.",
        },
        {
            "factor": "Input pedigree",
            "score": 2,
            "why": "Public user-uploaded logs, deduplicated and quality-gated; payloads and props unknown.",
        },
        {
            "factor": "Results uncertainty",
            "score": 2,
            "why": "Per-speed-band 95% envelopes with held-out coverage; wind term is modelled.",
        },
        {
            "factor": "Results robustness",
            "score": 2,
            "why": "Five vehicle-grouped folds; fast flight and strong wind are sparse.",
        },
        {
            "factor": "Use history",
            "score": 1,
            "why": "No operational use yet; own RTK + anemometer flights would raise this.",
        },
    ]
    limits = [
        "Absolute touchdown accuracy for your aircraft in real wind (no public X500 data with measured wind).",
        "Gust response: wind term is modelled from identified dynamics, not validated against measured wind.",
        "GNSS-outage behaviour beyond PX4's documented failsafe logic.",
        f"Speeds above {track['calibrated_speed_max_m_s']:.1f} m/s: few public X500 Offboard flights exist.",
    ]
    sources = [
        {
            "name": "PX4 Flight Review public logs",
            "url": "https://review.px4.io/browse",
        },
        {
            "name": "NanoBench (Crazyflie, Vicon)",
            "url": "https://github.com/syediu/nanobench-iros2026",
        },
        {"name": "Open-Meteo", "url": "https://open-meteo.com/"},
        {
            "name": "NOAA/NWS",
            "url": "https://www.weather.gov/documentation/services-web-api",
        },
    ]
    return {"headline": head, "credibility": cred, "limits": limits, "sources": sources}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", type=Path, required=True)
    ap.add_argument("--sitl", type=Path, required=True)
    ap.add_argument("--replay", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--esc", type=Path)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    track, track_err = tracking(args.logs)
    print("tracking done", flush=True)
    hd = hover_and_drag(args.logs, args.sitl)
    print("hover/drag done", flush=True)
    loop = (
        xi.Loop(
            **{
                k.replace("_rad_s", ""): v
                for k, v in track["loop_all"]["horizontal"].items()
            }
        ),
        xi.Loop(
            **{
                k.replace("_rad_s", ""): v
                for k, v in track["loop_all"]["vertical"].items()
            }
        ),
    )
    rp = replay(args.replay, args.logs, loop)
    prop = propulsion(args.esc) if args.esc else None
    results = {
        "referent": "Real vehicle onboard EKF2 estimate (GNSS-aided). Not independent truth.",
        "source": "PX4 Flight Review public logs, Holybro X500 V2, PX4 v1.14+, real hardware, deduplicated by SHA-256",
        "gazebo_x500_defaults": xi.GAZEBO_X500,
        "tracking": track,
        "propulsion_and_drag": hd,
        "propulsion_esc": prop,
        "sitl_replay": rp,
    }
    (args.out / "results.json").write_text(json.dumps(results, indent=1))
    figures(args.out, track_err, hd)
    env = {
        "schema": "px4-studio-fidelity-envelope/1",
        "aircraft": "Holybro X500 V2",
        "referent": results["referent"],
        "loop": track["loop_all"],
        "horizons_s": list(HORIZONS_S),
        "envelope_95_m": track["envelope_95_m"],
        "heldout_coverage": track["envelope_heldout_coverage"],
        "speed_bands_m_s": [[lo, hi, n] for lo, hi, n in SPEED_BANDS],
        "flights": track["segments"],
        "vehicles": track["vehicles"],
        "calibrated_speed_max_m_s": track["calibrated_speed_max_m_s"],
        "hover_thrust_real_p50": hd["hover_thrust_real"]["p50"],
        "drag_per_mass_linear_s": hd["drag_real_linear_quadratic"][0],
    }
    (ROOT / "config/fidelity").mkdir(parents=True, exist_ok=True)
    (ROOT / "config/fidelity").mkdir(parents=True, exist_ok=True)
    (ROOT / "config/fidelity/x500_evidence.json").write_text(
        json.dumps(evidence(track, hd, rp), indent=1) + "\n"
    )
    (ROOT / "config/fidelity/x500_envelope.json").write_text(
        json.dumps(env, indent=1) + "\n"
    )
    print(
        json.dumps(
            {
                k: track[k]
                for k in (
                    "windows",
                    "vehicles",
                    "heldout_model",
                    "heldout_plan_is_flight",
                    "envelope_95_m",
                    "envelope_heldout_coverage",
                )
            },
            indent=0,
        )[:1500]
    )
    print(
        json.dumps(
            {
                k: hd[k]
                for k in (
                    "hover_thrust_real",
                    "hover_thrust_sitl",
                    "drag_real_linear_quadratic",
                    "drag_sitl_linear_quadratic",
                    "motor_constant_scale_to_match_fleet_median",
                )
            }
        )
    )
    if rp:
        print(json.dumps({k: v for k, v in rp.items() if k != "per_window"})[:1200])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
