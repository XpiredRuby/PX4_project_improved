# PX4 Research Controller

PX4 controller development for GPS/IMU navigation, trajectory tracking, and
autonomous landing without a supplied ground height.

## Highlights

- Minimum-jerk takeoff and velocity-continuous return-to-home motion
- GPS-confidence speed adaptation and estimator health gates
- Command slew limits and flight-envelope guards
- Stable position/attitude gate before landing
- Gap-free PX4 landing handoff and automatic-disarm verification
- Setpoint watchdog, command acknowledgement, and safe timeouts

## Repository layout

| Path | Purpose |
|---|---|
| `controller/` | Current unknown-ground landing controller |
| `trajectory_generator/` | Jerk-limited trajectory tools |
| `analysis/` | Flight-log analysis and plots |
| `tests/` | Regression tests |
| `reference/original_baseline/` | Original baseline source and archive |
| `session_2_robust_landing/` | Validated Session 2 implementation and evidence |

## Session 2 validated results

- Five randomized/boundary trials: 5/5 successful
- PX4 LAND and automatic disarm: 5/5
- Maximum terminal XY error: 0.065 m
- Maximum absolute terminal Z error: 0.065 m
- Maximum handoff vertical speed: 0.105 m/s
- Session 2 tests: 19/19 randomized and 11/11 baseline

The current unknown-ground revision has not yet been simulation-tested.

## Local validation

```bash
python -m pip install -r requirements.txt
python scripts/validate.py
```

This repository contains software-in-the-loop research evidence, not flight
certification. Physical testing requires approved supervision.
