"""Measured-wind checks on the CMU DJI Matrice 100 dataset (anemometer on board, 180+ route flights).

1. Drag per mass from steady-flight tilt against MEASURED airspeed vs against ground speed
   (how much unmeasured wind contaminates the ground-speed estimate used for the X500 fleet).
2. Cross-track error on straight route legs against measured crosswind turbulence, giving an empirical
   wind-to-position coefficient for Studio's accuracy corridor.
Usage: python scripts/score_measured_wind.py DATA_DIR OUT_DIR   (DATA_DIR holds parameters.csv and flights/)
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from fidelity import cmu_m100 as cm  # noqa: E402


def r2(y, yhat):
    return float(1 - np.sum((y - yhat) ** 2) / np.sum((y - y.mean()) ** 2))


def legs(d, min_s=15.0, turn_deg=15.0):
    """Straight cruise legs: contiguous cruise samples whose course stays within turn_deg of the leg median."""
    m = cm.cruise_mask(d, min_speed=1.5)
    course = np.degrees(np.arctan2(d["v"][:, 0], d["v"][:, 1]))
    out, start = [], None
    for i in range(len(m) + 1):
        if (
            i < len(m)
            and m[i]
            and (
                start is None
                or abs(((course[i] - np.median(course[start:i]) + 180) % 360) - 180)
                < turn_deg
            )
        ):
            start = i if start is None else start
            continue
        if start is not None and d["t"][i - 1] - d["t"][start] >= min_s:
            out.append(slice(start + 5, i - 5))  # trim leg-entry and exit transients
        start = i if (i < len(m) and m[i]) else None
    return out


def main(data: Path, out: Path) -> int:
    params = pd.read_csv(data / "parameters.csv")
    flights = params[params.route.str.startswith("R")]
    loaded = {
        int(r.flight): cm.load(data / "flights" / f"{int(r.flight)}.csv")
        for r in flights.itertuples()
    }
    conv = cm.best_convention(list(loaded.values()))
    # 1. Drag vs airspeed and vs ground speed (steady, level cruise).
    air, gnd, acc_d = [], [], []
    rows = []
    for fid, d in loaded.items():
        a = cm.airflow(d, conv)
        dv = np.gradient(d["v"], d["t"], axis=0)
        steady = cm.cruise_mask(d) & (np.linalg.norm(dv, axis=1) < 0.3)
        air.append(np.linalg.norm(a, axis=1)[steady])
        gnd.append(np.linalg.norm(d["v"], axis=1)[steady])
        acc_d.append(cm.G * np.tan(d["tilt"][steady]))
        # 2. Cross-track vs measured crosswind turbulence on straight legs.
        w = cm.wind(d, conv)
        for sl in legs(d):
            p = np.column_stack([d["east"][sl], d["north"][sl]])
            if len(p) < 40:
                continue
            u = np.linalg.svd(p - p.mean(0), full_matrices=False)[2][0]  # leg direction
            n = np.array([-u[1], u[0]])
            xt = (p - p.mean(0)) @ n
            xt = xt - np.polyval(
                np.polyfit(np.arange(len(xt)), xt, 1), np.arange(len(xt))
            )  # remove planned-line offset/slope
            cross = w[sl] @ n
            rows.append(
                {
                    "flight": fid,
                    "duration_s": float(d["t"][sl][-1] - d["t"][sl][0]),
                    "speed_m_s": float(np.median(np.linalg.norm(d["v"][sl], axis=1))),
                    "crosswind_mean_m_s": float(abs(cross.mean())),
                    "crosswind_sigma_m_s": float(cross.std()),
                    "xtrack_p95_m": float(np.percentile(np.abs(xt), 95)),
                    "xtrack_rms_m": float(np.sqrt(np.mean(xt**2))),
                }
            )
    air, gnd, acc_d = map(np.concatenate, (air, gnd, acc_d))
    A_air, A_gnd = np.column_stack([air, air**2]), np.column_stack([gnd, gnd**2])
    c_air, *_ = np.linalg.lstsq(A_air, acc_d, rcond=None)
    c_gnd, *_ = np.linalg.lstsq(A_gnd, acc_d, rcond=None)
    drag = {
        "samples": int(len(acc_d)),
        "fit_vs_airspeed": {
            "k1": float(c_air[0]),
            "k2": float(c_air[1]),
            "r2": r2(acc_d, A_air @ c_air),
        },
        "fit_vs_groundspeed": {
            "k1": float(c_gnd[0]),
            "k2": float(c_gnd[1]),
            "r2": r2(acc_d, A_gnd @ c_gnd),
        },
    }
    L = pd.DataFrame(rows)
    X = np.column_stack([np.ones(len(L)), L.crosswind_sigma_m_s])
    coef, *_ = np.linalg.lstsq(X, L.xtrack_p95_m.to_numpy(), rcond=None)
    resid = L.xtrack_p95_m.to_numpy() - X @ coef
    # Conservative published coefficient: 95th percentile of p95 cross-track per unit turbulence above the GPS floor.
    floor = float(
        np.percentile(
            L.xtrack_p95_m[L.crosswind_sigma_m_s < L.crosswind_sigma_m_s.quantile(0.2)],
            50,
        )
    )
    ratio = (
        (L.xtrack_p95_m - floor).clip(lower=0) / L.crosswind_sigma_m_s.clip(lower=0.2)
    ).to_numpy()
    xtrack = {
        "legs": int(len(L)),
        "flights": int(L.flight.nunique()),
        "intercept_m": float(coef[0]),
        "slope_m_per_m_s": float(coef[1]),
        "r2": r2(L.xtrack_p95_m.to_numpy(), X @ coef),
        "resid_sd_m": float(resid.std()),
        "gps_floor_p95_m": floor,
        "coef95_m_per_m_s": float(np.percentile(ratio, 95)),
        "coef50_m_per_m_s": float(np.median(ratio)),
        "crosswind_sigma_range_m_s": [
            float(L.crosswind_sigma_m_s.min()),
            float(L.crosswind_sigma_m_s.max()),
        ],
        "speed_range_m_s": [float(L.speed_m_s.min()), float(L.speed_m_s.max())],
    }
    res = {
        "dataset": "CMU DJI Matrice 100 package delivery (doi:10.1184/R1/12683453, CC BY 4.0)",
        "anemometer_convention": {"frame": conv[0], "sense": conv[1], "sign": conv[2]},
        "flights": len(loaded),
        "drag": drag,
        "crosstrack_vs_measured_wind": xtrack,
        "caveats": [
            "Different airframe (DJI M100, ~3.6 kg) and autopilot (DJI A3/N1), not PX4.",
            "Position is standard GNSS (metre-level); the GPS floor is reported separately.",
            "Anemometer data is uncorrected for rotor-induced flow; convention chosen by wind stability.",
        ],
    }
    env_path = ROOT / "config/fidelity/x500_envelope.json"
    if env_path.exists():
        env = json.loads(env_path.read_text())
        env["measured_wind"] = {"coef95_m_per_m_s": xtrack["coef95_m_per_m_s"], "gps_floor_p95_m": floor,
                                "legs": xtrack["legs"], "flights": xtrack["flights"],
                                "drag_airspeed_k1_s": drag["fit_vs_airspeed"]["k1"], "drag_airspeed_k2_per_m": drag["fit_vs_airspeed"]["k2"],
                                "source": "CMU DJI Matrice 100, onboard anemometer"}
        env_path.write_text(json.dumps(env, indent=1) + "\n")
    ev_path = ROOT / "config/fidelity/x500_evidence.json"
    if ev_path.exists():
        ev = json.loads(ev_path.read_text())
        ev["headline"] = [h for h in ev["headline"] if not h["label"].startswith("Measured wind")]
        ev["headline"].append({"label": "Measured wind, route deviation", "value": f"{xtrack['intercept_m']:.2f} m p95",
                               "note": f"{xtrack['legs']} legs, {xtrack['flights']} flights with an onboard anemometer"})
        for c in ev["credibility"]:
            if c["factor"] == "Results uncertainty":
                c.update(score=3, why="Speed-band envelopes with held-out coverage; gust term calibrated on measured wind (other airframe).")
        ev["limits"] = [x for x in ev["limits"] if not x.startswith("Gust response")]
        ev["limits"].append("Gust response of this aircraft: calibrated on measured wind from a DJI M100, not an X500.")
        ev["sources"] = [x for x in ev["sources"] if "Matrice" not in x["name"]] + [
            {"name": "CMU DJI Matrice 100 (anemometer)", "url": "https://doi.org/10.1184/R1/12683453"}]
        ev_path.write_text(json.dumps(ev, indent=1) + "\n")
    out.mkdir(parents=True, exist_ok=True)
    (out / "results.json").write_text(json.dumps(res, indent=1) + "\n")
    L.to_csv(out / "legs.csv", index=False)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    idx = np.random.default_rng(0).choice(
        len(acc_d), min(6000, len(acc_d)), replace=False
    )
    ax[0].scatter(
        gnd[idx],
        acc_d[idx],
        s=2,
        alpha=0.25,
        color="#9aa0a6",
        label=f"vs ground speed (R² {drag['fit_vs_groundspeed']['r2']:.2f})",
    )
    ax[0].scatter(
        air[idx],
        acc_d[idx],
        s=2,
        alpha=0.25,
        color="#e8a33d",
        label=f"vs measured airspeed (R² {drag['fit_vs_airspeed']['r2']:.2f})",
    )
    ax[0].set_xlabel("Speed (m/s)")
    ax[0].set_ylabel("g·tan(tilt) (m/s²)")
    ax[0].set_title("Drag: measured airspeed vs ground speed")
    ax[0].legend(markerscale=4, fontsize=8)
    ax[1].scatter(
        L.crosswind_sigma_m_s, L.xtrack_p95_m, s=10, color="#5b8def", alpha=0.6
    )
    s = np.linspace(0, L.crosswind_sigma_m_s.max(), 50)
    ax[1].plot(
        s,
        coef[0] + coef[1] * s,
        color="#e8a33d",
        label=f"fit: {coef[0]:.2f} + {coef[1]:.2f}·σ",
    )
    ax[1].set_xlabel("Measured crosswind σ on leg (m/s)")
    ax[1].set_ylabel("Cross-track p95 (m)")
    ax[1].set_title("Route deviation vs measured turbulence")
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "measured_wind.png", dpi=130)
    print(json.dumps({"convention": conv, "drag": drag, "xtrack": xtrack}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1]), Path(sys.argv[2])))
