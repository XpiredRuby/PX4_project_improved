# RELLIS software work

This continuation prioritizes reusable controller logic. Aircraft and sensor
integration are deferred. Nothing here authorizes an unattended aircraft flight.

The controller uses a computed-command deadline of 0.25 seconds. A watchdog resend
does not renew it. If the worker stalls, one fresh zero-velocity command is allowed
only with a valid, fresh local estimate and confirmed armed Offboard ownership;
the client then stops sending and yields to PX4's configured failsafe. A resumed
worker cannot publish after handoff or overwrite a newer command with an old copy.

Arming, bootstrap, landing, and shutdown require telemetry from the selected PX4
autopilot component. Arming rechecks a fresh disarmed Offboard heartbeat after
acquiring the transport lock. Missing or old heartbeat data cannot establish disarm.
Pilot takeover is preserved during cleanup, including when an arming acknowledgement
was lost. Duplicate position and attitude timestamps preserve the original payload
and cannot renew measurement freshness.

Estimator status has the same source-clock rule. Duplicate packets may confirm that
the transport is alive, but cannot replace flags or renew measurement freshness;
regressed timestamps latch a navigation fault. Invalid local-estimate evidence causes
immediate command handoff instead of waiting through the degraded-navigation timer.

Mission geometry and navigation uncertainty are checked before arming. Required
trajectory fields cannot silently become zero. Backward trajectory queries reset
the interpolation cursor, and heading wrapping completes in bounded time even for
very large finite angles. Nonfinite commands and expired computations are rejected.

## Evidence writing

`controller/flight_logger.py` moves CSV writes, flushes, and routine runtime status
output to a dedicated thread. Control publication uses a bounded queue of 512
entries and never waits for the disk. Each row is copied before queuing. Flush
requests are coalesced; close drains the queue under a two-second deadline. A full
queue, write failure, or shutdown timeout remains an explicit failure. Evidence is
not silently discarded to make a run look successful.

The storage regression blocks the underlying writer for four seconds, after 100
writes. Its audit requires an exercised delay, control samples during the delay,
the unchanged setpoint-continuity limit, a completed nominal mission, and shutdown.
The separate control-stall regression pauses the control worker for two seconds and
requires failsafe handoff, one fresh brake, no subsequent Offboard sends, touchdown,
automatic disarm, and zero propulsion.

## Recovery and limits

Recovery and fallback are different results. A GPS-recovery case passes only when
navigation recovers and the mission completes. An invalid local estimate makes the
controller stop commanding; it does not pretend that telemetry remained valid.
Nominal audits require `SUCCESS`, so a safely landed timeout recovery cannot pass
as a completed mission. Touchdown quality failures remain failures after shutdown.

The default declared recovery budget is four seconds. The pre-arm PX4 policy derives
a minimum no-aiding timeout from that budget, PX4's configured post-outage GNSS
health window, and a one-second sampling/fusion reserve; the current SITL policy
therefore requires `EKF2_NOAID_TOUT >= 6 s`. The runner only verifies this setting
and never mutates persistent flight parameters.

Independent ULog truth is a hard offline gate for touchdown XY and every HOLD
episode's XYZ drift. Exact observations or interpolation brackets no wider than
0.25 seconds are accepted, and estimator-origin resets are treated as discrete
frame changes. An offline IMU propagation replay is retained only as feasibility
evidence: it never commands a vehicle and is explicitly not closed-loop validation.

Numerical software readiness is an engineering judgment, not a measured probability
of flight success. Tests and simulation cover specific configurations and failure
paths. They do not guarantee position accuracy after PX4 loses horizontal aiding.
See the validation evidence for passing runs, unsuccessful recovery trials, and
the unresolved prolonged GPS-loss/gust case.
