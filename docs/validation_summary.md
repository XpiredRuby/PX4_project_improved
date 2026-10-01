# Validation Summary

PX4 v1.17 / Gazebo 8.15 SITL. Results updated October 1, 2026.

| Scenario | Result | Evidence |
|---|---|---|
| Five nominal/yaw variants | Pass | 5/5 completed PX4 LAND and automatic disarm |
| Four-second GPS outage during takeoff | Pass | Recovered after HOLD; 0.112 m XY drift and 0.563 m Z drift |
| Twelve-second GPS outage | Pass | Failsafe LAND; 0.061 m/s touchdown vertical speed; automatic disarm and zero propulsion |
| Four-second GPS outage during trajectory | Pass | Estimator loss triggered failsafe LAND; 0.019 m HOLD XY drift, 0.021 m/s touchdown vertical speed, 0.065 m touchdown XY error; disarm in 2.787 s |
| Wind: 5 m/s east, 2 m/s north | Pass | 0.063 m touchdown XY error; 0.020 m/s touchdown vertical speed; 2,790 bounded commands |
| Six-core CPU contention | Pass | 50.77 ms p99 loop period, 54.57 ms maximum; zero missed periods or watchdog resends |
| Landing surface 2 m above takeoff | Pass | Surface added after 8 m climb; 0.077 m XY error, 0.012 m/s vertical speed; disarm in 2.202 s and zero propulsion |
| Landing surface 2 m below takeoff | Pass | Launch pad removed after 8 m climb; 0.077 m XY error, 0.022 m/s vertical speed; disarm in 2.356 s and zero propulsion |
| Abrupt controller-process loss | Pass | Independent observer: PX4 LAND in 0.990 s, touchdown in 13.790 s, disarm 2.200 s after touchdown; no bounce and fresh zero propulsion |
| Automated regression suite | Pass | 77 tests; Ruff and deterministic validation passed |
| Targeted logic coverage | Pass | 84% combined statement/branch coverage; CI minimum 80% |

The coverage gate applies to PID, minimum-jerk motion, mission configuration,
mission state, PX4 policy, run records, and trajectory loading. It does not
represent coverage of the complete controller or live MAVLink integration.

A short GPS outage can recover or require failsafe landing depending on whether
PX4 retains a valid estimate. Duration alone does not determine the outcome.
Both outcomes require actual touchdown, automatic disarm, and zero propulsion.

Evidence and reproduction details are in `validation/2026-10-01/`.
These are SITL results; slope, obstacle contact, combined failures, and physical
flight remain separate validation work.
