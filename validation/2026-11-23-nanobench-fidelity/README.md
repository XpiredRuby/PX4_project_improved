# NanoBench real-flight fidelity check (v1)

Date: 2026-11-23. Branch `claude/validation-harness`. Status: **method validation on real flights, not an x500 claim.**

## Question
Given only the commanded setpoints, how well can a fitted closed-loop model predict where a real
quadrotor actually flies, scored against independent motion-capture truth on flights it never saw?
And does an error bound set on training flights hold on unseen flights?

## Data
NanoBench (Ullah & Baca, arXiv:2603.09908): Crazyflie 2.1 (40.85 g instrumented), Vicon truth, 100 Hz.
Truth = Vicon `px,py,pz`. The onboard EKF is fed Vicon pose during collection, so it is not independent and is not used.

The public repository (cloned 2026-11-23, depth 1) contains 107 CSV files, not the 172 the paper and README state.

## Data-quality gate (`fidelity/quality.py`)
25 of 107 files rejected before any fitting:
- 20 byte-identical duplicates (same SHA-256, different repetition names; mostly `B9_trefoil_*`).
- Setpoint logs that leave the arena (up to 17.7 m away while Vicon shows the vehicle inside it), e.g. `B9_trefoil_fast_rep2`.
- Files with no airborne segment.
Isolated one-sample setpoint spikes are repaired by neighbour interpolation and logged; flights needing more than 10 repairs are rejected.
Every decision is in `nanobench_fidelity.json` under `quality_log`.

Scored set: 80 flights, 42.33 min of airborne tracking (battery-drain hover excluded).

## Model (`fidelity/tracking_model.py`)
Per axis: `a = wn^2 (sp(t - tau) - p) - 2 zeta wn v + c`, fitted by least squares on training flights only,
delay chosen on a 10 ms grid. Predictions are open-loop forward simulations from the initial truth state.
Baseline: "the vehicle flies exactly the plan" (setpoint = position), which is what a planner implicitly assumes.

## Results (held-out flights only)

| Protocol | train / test flights | Baseline RMSE (m) | Model RMSE (m) | Model p95 (m) | 95% bound from train (m) | Held-out coverage |
|---|---|---|---|---|---|---|
| A_heldout_repetitions | 59 / 21 | 0.434 | 0.226 | 0.482 | 0.530 | 97.1% |
| B_heldout_shapes | 29 / 51 | 0.406 | 0.323 | 0.637 | 0.530 | 87.1% |

- **A (new repetitions of known shapes):** model halves the plan-vs-flight error and the 95% bound holds (coverage >= 95%).
- **B (shapes never seen in training):** model still beats the baseline, but the 95% bound **fails** (87.1% coverage).
  Worst cases are 3-D trefoils; the fitted vertical axis is poorly damped (zeta ~ 0.10), consistent with
  training shapes that barely excite vertical motion.

## What this does and does not show
- Shows: the scoring method works on real data; a simple identified model predicts real tracking far better than "plan = flight";
  an error envelope is only valid inside the class of manoeuvres it was calibrated on.
- Does not show: anything about the x500, outdoor wind, GNSS, or speeds above ~2 m/s. Crazyflie at 41 g indoors is a different vehicle.
- v1 results were recorded before any tuning. The test flights have now been seen, so any model change must be scored on a new,
  untouched split and reported as v2.

## Reproduce
```
git clone --depth 1 https://github.com/syediu/nanobench-iros2026.git ~/validation-data/nanobench
python scripts/score_nanobench.py ~/validation-data/nanobench/datasets/dataset validation/2026-11-23-nanobench-fidelity
```
