# October 4 code-logic evidence

This directory records the bounded GNSS-recovery and controller-lifecycle checks
for the current research candidate. Simulator truth was used only after each flight.
No physical aircraft was flown.

## Verified results

- Ruff, deterministic trajectory generation, compilation, 203 discovered tests,
  and five PowerShell-connector subprocess tests passed.
- Four separate x500 trials with gusts and a four-second GNSS outage completed the
  full mission with `EKF2_NOAID_TOUT=6 s`. Independent touchdown XY error was
  0.058–0.128 m and physical HOLD XY drift was 0.510–0.656 m.
- A nominal mission and a mission with a four-second blocked evidence writer both
  completed. A two-second control-worker stall produced the intended PX4 failsafe
  handoff and bounded landing. Every onboard and independent position audit passed.
- CI-gated helper coverage is 89%. Complete controller-package coverage is 72%;
  `PID_position_new.py` is 64%. Coverage is not a readiness score.

The four-second recovery budget requires at least a six-second PX4 no-aiding timeout:
the outage, PX4's post-outage GNSS health window, and one second of reserve. This is
a timing contract, not a guarantee for longer outages. The harness restored and saved
the test computer's five-second baseline after each isolated trial.

## Unresolved boundary

The recorded gust plus twelve-second outage remains a failure: independent truth
measured 28.204 m touchdown XY error. An offline IMU propagation replay is included
as feasibility evidence only. It never commanded the vehicle and does not establish
closed-loop containment.

`results.json` contains the selected measured results and ULog SHA-256 identifiers.
`inertial_replays.json` contains the diagnostic replay output. Raw ULogs remain on
the test computer; hashes bind these compact records to those files.

The deployment helper wrote the five modified controller files with CRLF endings,
while Git stores them with LF endings. A bytewise comparison ignoring only trailing
carriage returns passed for all five files. `source_provenance.json` records both raw
hash sets; Python semantics and source text were otherwise identical.
