# Reusable-aircraft qualification record

`qualification.json` retains the profile-bound assessments, exact simulator
configuration, model/profile and budget hashes, ground confirmation, browser
results and validation-log digest. Full telemetry and ULogs remain at each
record's `evidence_directory` on the configured simulation machine. Decimated
track display samples are omitted from this consolidated record; each full
assessment's original digest is retained.

| Case | Result | Interpretation |
|---|---|---|
| Initial 0.10/0.10 m declarations | Failed profile-specific accuracy | Generic tracking checks passed, but navigation reached 0.134225 m and estimated tracking reached 0.340700 m. The failure remains in the record. |
| Separate nominal profile rerun | Passed sampled rehearsal | Predeclared 0.20 m navigation / 0.45 m tracking allowances; explicitly selected 1.0 m total path tolerance. Observed maxima were 0.149034 m / 0.352011 m. |
| Four-second GPS loss | Passed declared fault response; nominal accuracy failed | Navigation degraded, the mission recovered, and the landing/disarm/ground checks passed. This is not a nominal accuracy pass. |
| 0.4 s command-production stall | Passed declared fault response; trajectory incomplete | Command expiry/failsafe landing passed. The interrupted trajectory remains incomplete. |

The nominal mission used the installed x500 SDF configuration, fixed heading,
20 m launch-relative altitude and a straight 20 m route. Its planned duration
was 25.167786 s; observed route wall time was 25.199933 s. The whole recorded
lifecycle took 182.173635 s, inside its predeclared 217.751679 s nominal time
budget. The reserved contingency was additional. Generic sampled tracking
targets (0.5 m p95 / 1 m maximum) were unchanged between the diagnostic and rerun.

The browser exercised reusable setup, exact-artifact review, modified-hash
refusal, edit/refresh recovery, real Gazebo execution, truth overlay, assessment
download, and desktop/mobile layouts. No JavaScript errors were recorded.
Repository validation passed **350 tests**; the final SDF joint/axis restriction
also passed its focused binding regressions. The fault CLI itself was exercised
against both retained scenarios.

Offline regressions compile quad, hex, octo and custom profiles through the same
checker, test load/yaw/disturbance effects, missing inputs, energy exhaustion,
geometry rank and continuous allocation-box corners. Connected evidence here
is for x500 only. It does not qualify those other configurations or another
aircraft class.

These are limited sampled SITL results. The power envelope, torque-demand
reserve and accuracy/time bounds are declared assumptions with limited
evidence. No universal aerodynamic model, statistical reliability claim,
physical aircraft qualification or hardware-flight authorization is implied.
Every retained assessment has hardware approval false.

See [aircraft setup and supported classes](../../../../docs/aircraft_profiles.md)
for reproduction commands and the remaining physical qualification boundary.
