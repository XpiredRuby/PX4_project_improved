# PX4 Research Controller

PX4 controller development for GPS/IMU navigation, trajectory tracking, and
autonomous landing without a supplied ground height.

## Highlights

- Minimum-jerk takeoff and velocity-continuous return-to-home motion
- GPS-confidence speed adaptation and estimator health gates
- Averaged home reference and pre-arm trajectory checks
- Cruise height includes GPS vertical uncertainty and tracking margin
- Hold/recover on short GPS degradation; timeout requests return home
- Command slew limits and flight-envelope guards
- Stable position/attitude gate before landing
- Continuous logs through PX4 landing and automatic-disarm verification
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

The current unknown-ground revision has passed local regression checks but
has not yet been simulation-tested. It assumes terrain stays within the
configured height range and does not identify obstacles from GPS/IMU data.
The runner checks PX4 EKF2 source settings before arming. For PX4 v1.17,
configure GPS position/height/velocity (`EKF2_GPS_CTRL=7`, or 15 with dual-GPS
heading), GPS height reference (`EKF2_HGT_REF=1`), no magnetometer
(`EKF2_MAG_TYPE=5`), and disable `EKF2_BARO_CTRL`, `EKF2_OF_CTRL`,
`EKF2_EV_CTRL`, `EKF2_RNG_CTRL`, and `EKF2_AGP_CTRL`; reboot PX4. Single-GPS
yaw may require movement to initialize, so preflight can legitimately reject
an unaligned vehicle. Verify actual sensor fusion in PX4 logs.

## Local validation

```bash
python -m pip install -r requirements.txt
python scripts/validate.py
```

This repository contains software-in-the-loop research evidence, not flight
certification. Physical testing requires approved supervision.
