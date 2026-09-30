# Validation Summary

PX4 v1.17 Gazebo SITL validation completed on September 30, 2026.

| Scenario | Result | Key evidence |
|---|---:|---|
| Five nominal/yaw variants | Pass | 5/5 completed PX4 LAND and automatic disarm |
| Four-second GPS outage | Pass | `HEALTHY -> DEGRADED -> HOLD -> HEALTHY`; 0.112 m XY drift and 0.563 m Z drift during HOLD |
| Twelve-second GPS outage | Pass | PX4 failsafe LAND; 0.109 m XY drift before takeover; 0.061 m/s touchdown vertical speed; automatic disarm and zero propulsion confirmed |
| Windy world | Pass | 0.063 m touchdown error; 0.020 m/s touchdown vertical speed; all 2,790 commands bounded |
| Six-core CPU contention | Pass | 50.77 ms p99 loop period; 54.57 ms maximum; zero missed periods and zero watchdog resends |
| Automated regression suite | Pass | 68/68 tests, Ruff clean, validation script passed |

The evidence is software-in-the-loop research validation. Supervised hardware
tests are still required before autonomous physical operation.
