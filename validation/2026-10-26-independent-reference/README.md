# Pre-simulator reference audit and performance follow-up

Scope: offline reference generation and browser planning. No simulator or physical
vehicle ran. Work starts from a8060fc on PR #7. Both GitHub workflows on that
starting commit were independently confirmed successful during this increment.

## Changes

1. Removed redundant geometry-search work. Candidates have strictly decreasing
   control spacing; selection minimizes `(excess_above_soft_goal, -spacing)`.
   After a candidate passes symmetric and ordered fidelity with zero excess,
   no later candidate can improve that key. The search now stops there. It does
   not skip either certificate, alter limits, or claim a global geometric optimum.
2. Added `analysis/reference_audit.py`, importing no planner modules. It reads
   exported states and checks sampled dynamics, NED force/thrust/tilt, operating
   envelope, clock, and integral consistency. Analytic fixtures and deliberately
   corrupted states exercise the independent path. This is an additional
   diagnostic, not a replacement for continuous verification or rotor allocation.
3. Added measured wall time, path length, stationary/turning time, average speed
   and a speed-only lower bound to the corpus record. The lower bound ignores
   acceleration, stops and yaw and is **not** an achievable mission duration.
4. The app exposes route length/time and stationary/turning estimates next to
   requested and planned speeds. Stationary intervals have both endpoint speeds
   below 0.0001 m/s; turning requires nonzero sampled yaw rate. Timing is sampled
   reference data, not measured flight. Physical and native-runner caps are unchanged.
5. Fixed stale benchmark provenance: removed a hard-coded old commit ID. The
   runner now records current HEAD plus exact working Python/config source hashes.

## Independent numerical method and limitations

Position is cross-checked against the quintic-Hermite velocity integral
`h*(v0+v1)/2 + h²*(a0-a1)/10 + h³*(j0+j1)/120` in each position/yaw component.
This is exact through degree-five velocity, not all planner curves. A 1e-6
position/radian residual is a declared diagnostic tolerance at <=50 ms spacing,
not a proof of between-sample continuity. Maximum observed position residual in
the final corpus is 1.21691e-8 m, yaw residual 6.33534e-12 rad.

A separate cubic-Hermite acceleration integral reports velocity residuals. Its
initial fixed 1e-6 gate falsely rejected nine valid spline cases, because no snap
bound had been supplied. That failed run is retained in `release-corpus/results.json`.
The final gate uses a necessary consistency condition derived from bounded jerk:
`norm(v1-v0 - h*(a0+a1)/2) <= J*h²/4 + 1e-9` (the integral of a Lipschitz tent).
Yaw uses its declared angular jerk. Cubic residuals remain visible diagnostics;
maximum translational value was 9.19259e-6 m/s. This check can detect inconsistent
exports but does not establish sufficiency or empirical aircraft validity.

## Evidence map

- `optimized-corpus/results.json`: authoritative final 23-case run, all pass,
  exact source hashes and independent metrics. All exported route CSV hashes are
  identical to the earlier exhaustive search (`optimization-comparison.json`).
- `results.json`: first position-integral audit, 23 pass, before the timing UI.
- `final-corpus/results.json`: intermediate pre-optimization 23-case run. It used
  the initial position-only audit; despite its directory name, it is not the final
  audit implementation. Its captured hashes identify that source state.
- `release-corpus/results.json`: deliberately retained failed fixed-tolerance
  experiment, 14 pass / 9 fail; superseded by the justified jerk-bound gate.
- `geometry-and-audit-tests.txt`: 24 focused tests pass, including a straight
  route that evaluates one candidate and corruption/constant-bias detection.
- `unit-tests.txt`: final 10 independent-audit/disclosure tests pass.
- `javascript-tests.txt`: 56 tests pass.
- `release-boundary-tests.txt`: 6 tests pass.
- `python-validation.txt`: intermediate full 441-test run passed before geometry
  optimization. Final full-suite result is separately retained in
  `final-python-validation.txt`; do not substitute the intermediate result.
- `profile.txt`: pre-optimization profile identifies geometry/fidelity work.
- `benchmarks.png`: final corpus plot; all values are planned, never telemetry.
- `environment.json`: runtime versions and final portable build manifest.

Wall-time observations include concurrent validation on this host, so they are
not controlled benchmarks. Straight 2 m/s compilation plus audits fell from
11.0613 s to 5.7227 s. The entire 23-case CSV equality comparison is stronger
correctness evidence than that timing ratio.

## Browser and publication

Final portable build: `3ce4f8187b8b187e`. Chrome/Pyodide desktop completed the
straight route after cancel/retry and draft restoration. Full mission 6,242 rows,
312.02 s, 20 m route, requested 1 m/s, sampled peak 0.90 m/s; new time breakdown
is visible. `app.jpg` and `browser-state.txt` retain the final view.

An evidence download returned a browser path but no readable/synchronized file;
a subsequent CSV download failed in the automation transport and reset its
kernel. No byte-integrity success is claimed for these attempts. The app stayed
usable afterward. File-picker imports and cross-browser/device qualification
remain open; previous failures are not silently counted as passes.

Sites discovery again returned no owned/editable projects. The existing public
URL remains the older deployment; this account cannot publish the prepared
changes there. The local preview is internal QA, not a public deployment.

The model still lacks empirical identification; stop/turn references remain
conservative. These changes strengthen reproducibility and independent data
checking, not industrial certification or connected SITL readiness.

## Reproduce

```sh
python -m unittest tests.test_reference_audit tests.test_planner_geometry -v
python scripts/validate.py --controller-coverage
node --test tests/*.test.cjs
python scripts/check_conference_planner.py --output /absolute/new/corpus --repeats 1
python scripts/plot_reference_benchmarks.py /absolute/new/corpus/results.json --output /absolute/new/benchmarks.png
python -m analysis.reference_audit /path/to/trajectory.csv --vehicle /path/to/vehicle_model.json --environment /path/to/environment.json
python web/build_public_studio.py dist
```

Final full validation passed: 443 Python tests; VALIDATION_PASSED. The 390 px Chrome iframe (375 px content area after scrollbar) completed planning with scrollWidth=clientWidth=375; no horizontal page overflow. This is responsive layout coverage, not a physical-phone or Safari/Firefox test. See mobile-state.txt.
