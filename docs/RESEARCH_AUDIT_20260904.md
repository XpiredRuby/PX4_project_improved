# Research completion audit — 2026-09-04

## Verdict

The assigned code and SITL scope is complete. The controller is ready for mentor
review and further supervised research. It is not yet cleared for physical flight.

| Requested item | Status | Evidence |
|---|---|---|
| Check bugs/issues | Complete | Initialization, masks, timing, stale telemetry, limits, watchdog, landing, and heartbeat-source fixes tested |
| Actual vs desired for all states | Complete | Position, velocity, yaw, attitude, body-rate, and acceleration exports/plots |
| Outer vs inner loops | Complete | Outer velocity command, PX4 target echo, attitude targets, body-rate targets, and actual responses |
| Simplify code | Complete | Shared controller result contract and PID reset path |
| Initialization | Complete | Fresh telemetry and shared origin captured after OFFBOARD and arming |
| No jumps/jitters | Complete for SITL | Smooth transition reductions, jerk-limited turns, timing-outlier classification, watchdog coverage |
| Repeated evidence | Complete | Five paired trials plus exact-source final regression |
| Presentation package | Complete | Plot-ready CSVs, technical gallery, and six mentor-facing summary plots |
| Repository readiness | Complete | Clean source layout, portable tests, CI, safety/reproducibility docs, ignore rules |

The final validator passed 25/25 tests and verified the fixed source against
`20260903-125015-research-fixed-jerk`, including byte-identical regenerated
trajectory data and a 1,803-row derivative/timing audit.

## Independent standards check

The design aligns with current PX4 documentation in the following ways:

- Offboard setpoints are streamed before mode entry and continuously afterward.
- Runtime monitoring is much faster than PX4's minimum 2 Hz proof-of-life rule.
- Velocity commands use the MAVLink local-NED setpoint message and an explicit
  ignore mask.
- The custom outer loop feeds PX4 velocity control; PX4 then produces
  acceleration, attitude, and rate targets, matching the documented cascade.
- Velocity saturation and anti-windup match PX4 controller design principles.
- Trajectory position, velocity, acceleration, jerk, yaw, and yaw-rate signals are
  exported consistently, with jerk identified as a logged smoothness quantity.

## Remaining work before physical flight

1. Obtain supervisor authorization and confirm publication/flight-test ownership.
2. Run airframe-specific parameter and failsafe reviews.
3. Complete propeller-off interface tests and explicit Offboard-loss injection.
4. Run HITL with the actual flight controller and telemetry chain.
5. Use a written test card and staged envelope expansion.

## Authoritative references

- PX4 Offboard mode: https://docs.px4.io/main/en/flight_modes/offboard
- PX4 controller diagrams: https://docs.px4.io/main/en/flight_stack/controller_diagrams
- PX4 TrajectorySetpoint: https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint
- MAVLink SET_POSITION_TARGET_LOCAL_NED: https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
- GitHub Python build/test guidance: https://docs.github.com/en/actions/tutorials/build-and-test-code/python
