# October 4, 2026 status

**Research candidate: the requested 95+ flight-readiness target is not established.**

The current code passes Ruff, deterministic generation/compilation, **203 discovered
regression tests**, and five connector subprocess tests. Branch-aware coverage is
89% for the CI-gated helper modules, 72% for the complete `controller/` package,
64% for the large `PID_position_new.py` module, and 62% for all measured controller
and analysis code. These are coverage measurements, not a readiness score.

The controller now distinguishes estimator receipt time from estimator source time.
Duplicate status packets cannot replace flags or renew measurement freshness;
regressed source timestamps latch a fault. An invalid local estimate causes immediate
Offboard handoff. Independent truth alignment rejects interpolation brackets over
0.25 seconds, treats estimator-origin changes as discrete, and emits an explicit
pass/fail gate for touchdown and every HOLD episode.

The declared GNSS recovery budget is four seconds. For PX4 v1.17, the pre-arm policy
requires `EKF2_NOAID_TOUT >= 6 s`, covering the outage, the configured post-outage
GNSS health period, and one second of sampling/fusion reserve. Four independent x500
trials with gusts and early, middle, or late four-second outages completed the full
mission. Independent ULog truth measured **0.058–0.128 m touchdown XY error** and
**0.510–0.656 m physical HOLD XY drift**; every onboard and physical-position audit
passed. The runner verifies this policy but never writes persistent PX4 parameters.

The final-source live matrix also passed: a nominal mission completed successfully;
a four-second blocked evidence writer completed without blocking control; and a
two-second control-worker stall sent one eligible fresh brake, stopped Offboard
publication, and landed through PX4 failsafe. Independent touchdown errors were
0.053 m, 0.067 m, and 0.931 m respectively, all within the unchanged 1.5 m limit.
Every case confirmed contact, automatic disarm, fresh zero propulsion, and safe
simulator cleanup.

The operating-envelope boundary remains explicit. Combined gusts and a twelve-second
GPS outage produced `UNSAFE_TOUCHDOWN` with **28.204 m physical touchdown XY error**.
Ground contact and shutdown do not turn that result into a pass. A prior 10-second
no-aiding-timeout experiment also failed. PX4's native fallback cannot guarantee
bounded horizontal position after horizontal aiding is lost.

An offline pre-outage-bias IMU replay stayed within 0.373 m of independent truth over
the recorded twelve-second outage, but it uses asynchronous telemetry and is not a
closed-loop controller validation. No inertial bridge was added to flight code on
that evidence alone.

See `validation/2026-10-04-code-logic` for the current compact evidence and
`validation/2026-10-04` for the reproduced prolonged-outage failure.

## Deferred aircraft integration

The RELLIS continuation prioritizes code logic; the airframe, hover model,
GNSS update rate/accuracy, and IMU characteristics will be supplied later. Their
absence does not block software tests, controller lifecycle fixes, or simulation.
They limit what can be claimed about actual flight accuracy and allowable wind or
GPS outages. Independent aiding can be evaluated when aircraft integration begins.
See [code logic](code-logic.md) for the current software work.

Unknown terrain remains bounded by the configured ±10 m height range and verified
surface cases. GNSS/IMU cannot certify obstacle clearance or arbitrary landing slope.

---

# Historical October 3 candidate and trials

# October 3, 2026 validation

The GPS/GNSS + 6-DOF IMU mission uses PX4's inner loops for takeoff,
trajectory tracking, return to the recovered launch XY, native LAND, automatic
disarm, and propulsion-stop confirmation. No landing height is supplied to
the controller. Surface changes, wind, and simulator truth belong to the
test harness; truth is used only after flight.

This candidate improves braking and landing supervision, but the requested
90–95/100 readiness target has not been established. A numerical readiness
assessment is engineering judgment, not a probability of surviving flight.
Physical-flight readiness still needs the intended aircraft and supervised
hardware evidence.

## Changes

- Retimed cruise reference: at most 1.0 m/s horizontally, with a 1.25 m/s
  normal velocity-command cap. The phase clock and feedforward velocities
  scale together. The existing position PID gains remain unchanged.
- Navigation HOLD: freeze the path clock and last trusted reference; use
  fresh local-velocity damping (0.8) and bounded 3.0 m/s² horizontal braking.
  Normal horizontal slew stays 1.5 m/s² and vertical slew stays 1.0 m/s².
- Freeze an XY reference before native landing and check touchdown
  displacement at runtime, including failsafe landings. Preserve detected
  violations through ground/disarm/zero-output confirmation and report them.
- Use exact combined body tilt, `acos(cos(roll) * cos(pitch))`, for landing
  readiness and touchdown checks.
- Verify supported estimator timeout and native landing speed parameters
  before arming. Final terrain trials use `MPC_LAND_SPEED=0.4`,
  `MPC_LAND_CRWL=0.3`, `EKF2_NOAID_TOUT=5000000`, and
  `COM_DISARM_LAND=2`. The mission runner reads parameters; it does not alter
  persistent flight settings. The SITL harness sets landing speed while
  independently confirmed disarmed/on ground.
- Increase native landing timeout to 180 s, with a preflight descent-budget
  check for the configured maximum 40 m descent. The 240 s mission timeout
  governs the Offboard mission wait; it is not a whole-flight deadline.
- Differentiate emitted retimed reference velocities against real elapsed
  time for acceleration/jerk analysis. Do not bridge HOLD/phase transitions,
  native landing, regressed clocks, nonfinite samples, or telemetry gaps.
  Preserve raw path-clock derivatives as separate signals.
- Preserve the controller worker's actual exception when the thread stops.
  Trajectory-generator edits fix lint findings and retain CSV geometry.

## Acceptance criteria

HOLD XY/Z drift is at most 1 m and path-clock movement at most 0.1 s.
Touchdown XY error is at most 1.5 m relative to the frozen landing reference;
horizontal and vertical speed are each at most 0.5 m/s; combined tilt is at
most 10°. No post-touchdown bounce is allowed. Automatic disarm must occur
within 5 s and fresh propulsion outputs must be zero afterward. Thresholds
were not relaxed to make a failing trial pass.

`SUCCESS` means the full planned mission completed. `PX4_FAILSAFE` with a
passing audit means a bounded abort landed and stopped propulsion; it does
not mean the original trajectory completed. `UNSAFE_TOUCHDOWN` records a
quality failure even if PX4 subsequently disarms and outputs zero propulsion.
An expected-rejection test can verify reporting/cleanup while the landing
itself remains unsafe.

## Independent verification

Compact telemetry and aligned truth traces accompany each selected trial.
ULog filenames, original paths, and SHA-256 hashes identify the larger raw
logs retained on the test computer. Position truth is aligned using PX4's
logged position clock and the contemporaneous estimator origin, without
extrapolation. Physical HOLD drift is measured from the first HOLD truth
sample. Terrain height changes use absolute simulator truth altitude from
the first stable ground second to contact, excluding bootstrap ascent and
estimator-origin shifts.

The original implementation and unrelated working-tree changes are preserved.
Deployment records include guarded pre-update hashes, post-update hashes,
an exact backup of replaced files, and a reviewable patch. No git reset,
clean, commit, or push was used.

## Remaining blockers

Combined wind and prolonged GPS loss has produced excessive lateral
touchdown displacement. Increasing the EKF no-aiding timeout from 5 s to
the supported 10 s limit did not solve it; 5 s was restored. Ground contact,
automatic disarm, and zero propulsion alone do not make that landing pass.
Runtime quality reporting detects the displacement but does not prevent it.

The 10° slope boundary remains sensitive to actual contact/settling dynamics.
GPS/IMU cannot establish that an unknown surface is clear of obstacles or
within the allowed slope. The ±10 m tests verify specific bounded surfaces,
not arbitrary terrain or every possible initial attitude.

Further flight-readiness work needs the intended airframe, motor/thrust and
hover characteristics, GPS update/accuracy data, IMU noise/bias data, and the
required wind and GPS-outage operating envelope. A different navigation-loss
control design needs those inputs and separate validation; extending EKF
validity or accepting excessive drift is not a demonstrated solution.

The final code is a SITL research candidate. No physical aircraft was flown.

## October 3 results

GPS matrix: final control law and touchdown XY checks, before the added native landing speed policy. These trials used the previous PX4 landing speed (0.7 m/s); the final terrain/wind verdict trials use 0.4 m/s. Manifests identify exact source and configuration hashes.

| Scenario | Safety audit | Mission outcome | Physical HOLD XY drift | Physical touchdown XY error |
|---|---|---|---:|---:|
| calm-gps-12s-final | Pass | PX4_FAILSAFE | 0.604 m | 0.688 m |
| gust-gps-4s-final | Pass | SUCCESS | 0.630 m | 0.091 m |
| return-gps-4s-final | Pass | PX4_FAILSAFE | 0.178 m | 0.180 m |
| takeoff-gps-4s-final | Pass | SUCCESS | 0.097 m | 0.087 m |

Final parameter-policy trials:

| Scenario | Safety audit | Mission outcome | Physical touchdown XY error | Contact / max ground tilt |
|---|---|---|---:|---:|
| gust-gps-12s-final | FAIL | UNSAFE_TOUCHDOWN | 29.188 m | 0.38° / 0.40° |
| lowered-10m-final | Pass | SUCCESS | 0.109 m | 0.02° / 0.02° |
| offset-yaw-final | Pass | SUCCESS | 0.101 m | 0.01° / 0.02° |
| raised-10m-final | Pass | SUCCESS | 0.078 m | 0.04° / 0.04° |
| slope-10deg-verdict | Pass | SUCCESS | 0.071 m | 4.22° / 4.22° |
| slope-8deg-final | Pass | SUCCESS | 0.070 m | 4.42° / 4.42° |

The raised and lowered surfaces changed by +10 m and -10 m in independent absolute truth altitude. The final 10° slope landing passed, whereas an earlier trial failed at that boundary. The original terrain batch exited with an expectation mismatch because that trial succeeded when rejection was anticipated; no failed quality check was waived. The remaining wind/GPS trial ran separately after fresh ground/disarm/zero-output confirmation. See driver summaries and independent verification.

Earlier retiming experiments: two wind + 4 s GPS outages passed, with physical HOLD drift 0.632 m and 0.624 m. Wind + 12 s GPS loss failed with 29.119 m physical touchdown displacement. A 10 s EKF no-aiding-timeout experiment also failed at 8.165 m and was restored to 5 s. These results remain failures and do not describe the final landing-speed configuration.

Code checks: 131 regression tests, full Ruff checks including the trajectory generator, deterministic generation/compilation validation, and a real-run analysis/plot generation check passed in the staged candidate. The installed repository check is recorded separately. Earlier 84% targeted coverage belongs to the October 1 source and has not been re-measured for this candidate.

---

# Historical October 1 results

The following results predate this candidate and used simpler individual failures. The twelve-second GPS result does not establish performance with simultaneous wind.

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
