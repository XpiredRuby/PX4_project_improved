# Session 2 — Robust Randomized-Start Landing

PX4 v1.17 SITL and Gazebo Harmonic x500 implementation for randomized airborne
initialization, stabilization, trajectory tracking, controlled descent, PX4
LAND, and automatic disarm.

## Results

- Five randomized/boundary trials: 5/5 successful
- PX4 LAND and automatic disarm: 5/5
- Maximum terminal XY error: 0.065 m
- Maximum absolute terminal Z error: 0.065 m
- Maximum handoff vertical speed: 0.105 m/s
- Maximum trajectory Z RMSE: 0.027 m
- Post-disarm proof: 10-second hold, disarmed state, zero motor outputs
- Tests: 19/19 randomized and 11/11 baseline

## Contents

- `01_Source/` — active controller and trajectory generator
- `02_Reproduction/` — analysis and regression scripts
- `03_Validation/` — validation reports and metrics
- `04_Launchers/` — visible test launchers
- `05_Evidence/` — trial summaries, exact source snapshots, and proof logs
- `06_Test_Results/` — test summary

## Reproduction

```bash
bash /mnt/f/PX4/research/analysis/run_randomized_regression.sh random 42
bash /mnt/f/PX4/internal/run-visible-five.sh
bash /mnt/f/PX4/internal/run-visible-proof.sh
```

Simulation use only. Physical testing requires approved supervision.
