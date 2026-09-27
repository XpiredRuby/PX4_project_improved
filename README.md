# PX4 Research Controller

PX4 SITL controller development for trajectory tracking, randomized-start
stabilization, and reliable autonomous landing.

## Highlights

- MAVLink outer-loop position control
- Jerk-limited trajectory generation
- Randomized airborne initialization
- Controlled handoff to PX4 LAND
- Automatic-disarm verification
- Analysis, regression tests, and reproducible evidence

## Repository layout

| Path | Purpose |
|---|---|
| `controller/` | Initial improved controller |
| `trajectory_generator/` | Jerk-limited trajectory tools |
| `analysis/` | Flight-log analysis and plots |
| `tests/` | Regression tests |
| `reference/original_baseline/` | Original baseline source and archive |
| `session_2_robust_landing/` | Current landing implementation and validation |

## Validated results

- Five randomized/boundary trials: 5/5 successful
- PX4 LAND and automatic disarm: 5/5
- Maximum terminal XY error: 0.065 m
- Maximum absolute terminal Z error: 0.065 m
- Maximum handoff vertical speed: 0.105 m/s
- Current tests: 19/19 randomized and 11/11 baseline

## Local validation

```bash
python -m pip install -r requirements.txt
python scripts/validate.py
```

This repository contains software-in-the-loop research evidence, not flight
certification. Physical testing requires approved supervision.
