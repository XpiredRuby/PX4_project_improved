# Real Holybro X500 V2 flights vs Studio and Gazebo (public data)

Date: 2026-11-23 · Branch `claude/validation-harness` · Release class: analysis only, no flight.

## What was compared
Public PX4 Flight Review logs of **real Holybro X500 V2** aircraft (PX4 v1.14+, real hardware, at least 30 s of Offboard flight).
Byte-identical re-uploads were removed (27 of the first 120 logs were duplicates). After quality gates:
**27 aircraft, 47 Offboard segments, 194 minutes, 556 twenty-second windows.**

**Referent:** each aircraft's own EKF2 estimate (GNSS-aided). These logs have no independent truth and no measured wind,
so every number below is "agreement with the onboard estimate", not "accuracy against truth".

## 1. Predicting real flights from setpoints (held-out aircraft)
Five folds grouped by aircraft: the model never saw the aircraft it is scored on.
Model: second-order closed loop fitted by output error (horizontal wn 1.25 rad/s, zeta 0.7).

| Predictor | RMSE (m) | Median (m) | p95 (m) |
|---|---|---|---|
| Plan = flight (no model) | 0.88 | 0.22 | 1.09 |
| Studio closed-loop model | 0.72 | 0.13 | 0.96 |

95% envelope (worst error up to the horizon), split-conformal, coverage measured on aircraft not used to set it:

| Speed band | windows | 1 s | 5 s | 20 s | held-out coverage at 20 s |
|---|---|---|---|---|---|
| < 2 m/s | 518 | 0.37 m | 0.97 m | 1.26 m | 99% |
| >= 2 m/s | 38 | 1.94 m | 5.11 m | 5.13 m | 90% |

Public X500 Offboard flying is mostly slow: the 99th-percentile window speed is **3.3 m/s**.
Studio marks faster plans as *extrapolating* or *uncalibrated*.

## 2. Same real setpoints flown in Gazebo (PX4 v1.17 SITL, gz x500)
30 real windows replayed through SITL (`scripts/sitl_replay.py`): 5 s lead-in, 20 s scored, displacement compared with the real flight.

| Predictor | RMSE (m) | Median (m) | p95 (m) |
|---|---|---|---|
| Plan = flight | 2.01 | 0.25 | 3.75 |
| Gazebo x500, default airframe tuning | 1.53 | 0.31 | 3.14 |
| Studio closed-loop model | 1.55 | 0.18 | 2.66 |
| Gazebo x500 with the real aircraft's own PX4 tuning | 0.67 | 0.30 | 1.23 |

**Main finding: the largest sim-to-real gap is controller configuration, not physics.** Loading the real aircraft's
multicopter tuning into SITL cuts RMSE by 56% and p95 by 61%.
Simulate with the parameters your aircraft will actually fly.

## 3. Physics: real fleet vs the Gazebo x500 model
| Quantity | Real X500 V2 | Gazebo x500 default |
|---|---|---|
| Hover thrust command (p10 / p50 / p90) | 0.48 / 0.62 / 0.70 (85 flights) | 0.73 (188 SITL logs) |
| Hover rotor speed | 4655-5901 rpm (4 unique ESC-RPM logs) | 7232 rpm |
| Motor time constant | 55-190 ms | 12.5 ms (up) |
| Steady drag per mass, g·tan(tilt) (m/s²) | 2 m/s: 0.91, 3 m/s: 1.10 | 2 m/s: 0.23, 3 m/s: 0.34, 5 m/s: 0.60 |

- The default sim x500 hovers at higher throttle than the median real X500: less thrust margin. Matching the fleet median needs motorConstant x1.38.
- Simulated motors respond several times faster than real ones; braking and stop overshoot will be optimistic in SITL.
- Real aircraft tilt 3-4x more than SITL at the same speed. Part of this is unmeasured wind, but the gap is larger than wind alone explains; SITL under-predicts drag and wind sensitivity.

## 4. How Studio uses this
`config/fidelity/x500_envelope.json` drives the accuracy corridor drawn around every route and the header confidence badge.
`config/fidelity/x500_evidence.json` feeds the **Accuracy evidence** panel. The wind term (gust excess plus 2-sigma Dryden turbulence
through the identified loop and the real-fleet drag coefficient) is model-based and labelled as such.

## Not claimed
- Absolute accuracy against independent truth, touchdown accuracy in real wind, or gust response.
- Anything above 3.3 m/s or in wind above 5 m/s at flight altitude.
- Behaviour of any single aircraft: payloads, propellers and batteries in the public fleet are unknown.

## Reproduce
```
python scripts/mine_x500_logs.py ~/validation-data/x500 --max-logs 120
python scripts/hunt_esc_logs.py ~/validation-data/x500 --max-downloads 400
python scripts/sitl_replay.py --autopilot ~/PX4-Autopilot --logs ~/validation-data/x500 --out ~/validation-data/replay --max-windows 40 --speed 2
python scripts/score_x500_logs.py --logs ~/validation-data/x500 --sitl ~/validation-data/sitl --replay ~/validation-data/replay \
    --esc ~/validation-data/x500/esc --out validation/2026-11-23-x500-public-logs
```
`~/validation-data/sitl` holds earlier Studio SITL logs extracted with `fidelity.px4_logs.extract`.

## 5. Calibrated Gazebo model (`x500_cal`) and what it changes
Variants of the stock x500 were flown on the same real windows, all with each aircraft's own PX4 tuning
(`scripts/sitl_replay.py --variant`). The drag scale was chosen on half of the aircraft and scored on the other half
(`scripts/score_calibrated_x500.py`, `calibrated_model.json`).

| Model (real tuning in all runs) | held-out RMSE | median | p95 |
|---|---|---|---|
| Stock x500 | 0.49 m | 0.27 m | 1.10 m |
| **x500_cal**: motorConstant x1.38, motor time constant 80/100 ms | 0.49 m | **0.24 m** | 1.12 m |
| x500_cal + rotor drag x2.5 | 0.61 m | 0.24 m | 1.55 m |

- Matching hover throttle and motor lag to the real fleet gives a small median gain and no tail change. Controller
  tuning (section 2) matters far more than airframe physics for setpoint tracking.
- Raising rotor drag makes the simulator worse, so the extra tilt seen in the public fleet (section 3) is mostly
  unmeasured wind, not airframe drag. Studio treats real-fleet drag as an upper bound.
- Install: `python scripts/install_x500_cal.py --autopilot ~/PX4-Autopilot` (adds `x500_cal`; stock `x500` untouched),
  then register it with `mission_planner.simulation_binding --model x500_cal`.

## 6. GNSS outages longer than EKF2_NOAID_TOUT
Studio predicts PX4's documented sequence (dead reckoning, then altitude-only Descend drifting with the wind until
re-acquisition) instead of a touchdown point. Fitted on the SITL gust + 12 s case (29.2 m recorded, 32 m modelled),
checked on the calm 12 s (0.69 m) and gust 4 s (0.63 m) cases. Real-aircraft drift for that case: 32-55 m (upper bound).
