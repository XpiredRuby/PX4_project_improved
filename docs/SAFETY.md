# Safety boundary

## Non-negotiable rule

Do not run this code on a physical aircraft unless Dr. Moble Benedict or Vishnu
Saj is physically present and has explicitly authorized the test.

## Before any hardware test

- Freeze and record the exact source revision and generated trajectory hash.
- Review the vehicle, estimator, coordinate frames, limits, and parameter file.
- Confirm Offboard-loss, data-link-loss, geofence, RC/manual override, and battery
  failsafe actions on the specific airframe.
- Perform propeller-off command, mode, arming, watchdog, and telemetry-loss tests.
- Validate SITL, then HITL, before a restrained or free-flight test.
- Establish a test card, abort criteria, exclusion area, spotter, and emergency
  mode-switch procedure.
- Preserve PX4 ULog, companion-computer logs, parameter exports, and timestamps.

## Runtime protections already implemented

- Fresh position, attitude, and heartbeat requirements.
- Armed-state and Offboard-mode guards during active phases.
- Continuous setpoint watchdog and measured setpoint gaps.
- Horizontal vector and vertical velocity limits.
- PID output limits and conditional anti-windup.
- Common-origin initialization after fresh telemetry, mode entry, and arming.
- Native PX4 LAND handoff and native auto-disarm.
- No forced airborne disarm in failure cleanup.

These controls reduce risk but do not replace airframe-specific validation.

