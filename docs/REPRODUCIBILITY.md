# Reproducibility

## Validated reference run

- Archive: `20260903-125015-research-fixed-jerk`
- Result: mission DONE, PX4 native LAND, native auto-disarm
- Combined XY RMSE: 0.20739 m
- Z RMSE: 0.01033 m
- Yaw RMSE: 0.04046 rad
- Median/P99 loop interval: 50.000/50.147 ms
- Maximum loop interval: 100.600 ms
- Missed periods: 1
- Watchdog resends: 0

## Five-pair campaign

- XY RMSE: 0.20607 → 0.20332 m (−1.3%)
- Paired 95% CI for XY difference: [−0.00462, −0.00088] m
- Landing drift: −50.1% by group means
- Takeoff transition step: −95.0%
- Landing transition step: −99.2%
- Nine intervals >75 ms across 26,763 iterations
- Maximum event controller compute time: 0.269 ms

## Reproduction rules

1. Record PX4 commit/version, simulator, model, parameters, OS/WSL version, and
   Python dependency versions.
2. Generate `trajectory.csv` from the checked-in generator; do not hand-edit it.
3. Record SHA-256 hashes for the controller source and generated trajectory.
4. Run paired trials with counterbalanced order where practical.
5. Preserve failures and exclusions; do not silently drop unsuccessful attempts.
6. Keep raw logs immutable and generate plots/reports into separate directories.
7. Report unavailable fields as unavailable rather than estimating them.

