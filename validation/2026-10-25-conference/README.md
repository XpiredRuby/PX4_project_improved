# Executed engineering validation — 25 October 2026

Source baseline: PR #6 `72f7b03c26e4ca20e2d45a19f272e5b1d9c4961b`.
Changes: branch `ai/conference-planner-validation-20261025`.
Exact tested source hashes and runtime versions: `environment-and-source.json`.
The containing Git commit binds the source and evidence together.

## Results

| Executed check | Result | Evidence |
|---|---|---|
| `python scripts/validate.py`, final source | 426 tests passed; generation and deterministic waypoint export also passed | `release-python-validation.txt` |
| `node --test tests/*.test.cjs` | 45 passed | `javascript-tests.txt` |
| Repository Ruff check | Passed | `lint.txt` |
| Existing strict trajectory acceptance | 16/16 expected accept/refuse outcomes passed | `strict-acceptance/report.json` |
| New closest-mode repeated corpus | 23 cases × 2 = 46/46 accepted, invariants passed, each native pair byte-identical | `repeated-plans/results.json`, `repeated-plans.txt` |
| Constructive native adapter | 8 tests passed, including actual batch staging, digest/certificate tampering, full-mission refusal and interpolation | `adapter-tests.txt`; included in final full suite |
| Independent rehearsal audit | 6 tests passed, including polynomial reference comparison instead of linear chords | `rehearsal-tests.txt`; included in final full suite |
| Real Chrome/Pyodide desktop and 390 px iframe | Passed the checks listed below | `browser-test-record.json`, `browser-handwriting.jpg` |
| Native versus real-browser straight reference | Passed; 6,066 full mission rows in each runtime | `browser-native.json`, compressed original browser exports in `browser/` |
| Portable public bundle | Built 28 planner/configuration files | `public-build.txt` |
| New connected Gazebo campaign | FAILED BEFORE SIMULATOR START; zero new simulated flights | `sitl-final-attempt/status.json` |
| Public production update | Not performed: connected Sites account returned no owned/shared editable Site | Existing URL is not claimed to contain these changes |

The closest corpus includes speed preferences 0.5, 1, 1.25, 1.5, 2, 3 and 5 m/s;
a deliberately lower cap; 0.1/1/30/50 m height requests; reversal; five stops;
duplicate hover; tiny detail; figure-eight; 10,000 points; boundary repair; Vin;
an impossible deadline; and one/two-lap circles. Repaired requests retain their
original input and enumerate changes. Original-fit failures are disclosed, not
turned into fake fidelity passes. Largest diverse constructive serialized-state
reconstruction error in that corpus: `3.319e-12` (mixed state units; see per-case
reports). It is a numerical reconstruction metric, not flight accuracy.

## Speed regression

The old 1.25 m/s default hard cap was hidden in advanced settings. It clipped
higher requested cruise speeds. The fresh demo now shows a 3 m/s cap beside
cruise; explicit/restored lower caps remain binding and produce clear guidance.
The aircraft model is unchanged. The planner's existing 10% margin remains.

A real browser 20 m straight route at requested cruise 2.00 m/s produced a
sampled planned peak of 1.80 m/s. The separate native 35 m sweep produced sampled
peaks of 1.4513 m/s at requested 2 m/s and 1.7487 m/s at requested 3 m/s; acceleration,
jerk and timing conservatism can prevent the preference being reached. A 5 m/s
request was adjusted to the model's 3 m/s ceiling. Explicit 1.25 m/s cap remained
binding. See `speed-regression-checks.txt` and the unrounded result JSON.

Native flight control separately retains its intentional 1.0 m/s reference-progress
policy and 1.25 m/s command cap. This increment does not increase either.

## Cross-runtime and browser evidence

The downloaded browser CSV SHA-256 matches the downloaded JSON report. Comparison
against a fresh native compile uses mission timestamps and interpolates only the
native reference, not measurements. Maximum north-position difference was
`5.324e-11 m`, vertical `7.191e-12 m`, velocity-component difference at most
`2.657e-11 m/s`; duration difference `2.718e-11 s`. Per-channel differences,
including yaw/angular derivatives, are retained. These establish agreement for
this request and these runtimes, not universal numerical equivalence or tracking.

Reproduce using the actual saved browser files:

```bash
python scripts/compare_browser_reference.py \
  --bundle validation/2026-10-25-conference/browser/straight-2mps.json.gz \
  --csv validation/2026-10-25-conference/browser/straight-2mps.csv.gz \
  --output /absolute/new/browser-native.json
```

Chrome checks used the real Python/NumPy WebAssembly worker, not mocked planning.
The 481-point Vin example produced a 12-vertex adjusted path and 8,255 mission
samples at the tested 2 m/s preference. At 63.50 s the reference remained at
N=0/E=0/Up=20 m, speed=0, while heading was 337.7° and yaw rate −37.30°/s.
Exact next/previous stepping, alternate 3D/height views, final ground endpoint,
play/pause, cancellation/retry, stale edits, restored drafts, conflicting cap
feedback, incomplete-input error and recovery were exercised. 1x clock arithmetic
is additionally covered by JavaScript regressions; no sub-millisecond wall-clock
browser timing certificate is claimed. The 390 px iframe had a 375 px body scroll
width, without horizontal overflow. This is viewport testing, not a real phone test.

## Experimental native adapter and failed launch

The adapter binds every model/intent/certificate digest, reconstructs every route
state and exact phase boundary, checks diverse derivative extrema and reruns
continuous physical checks. Runner staging preserves all required sidecars.
The runner uses C3 septic interpolation for this family; the truth auditor uses
an independently solved phase evaluator. Native lifecycle ownership is unchanged.
Ordinary studio execution still refuses this unqualified family.

A real CLI launch attempt used:

```bash
python scripts/planner_sitl_campaign.py --confirm-sitl --experimental-stop-turn \
  --autopilot /home/xpire/PX4-Autopilot \
  --output validation/2026-10-25-conference/sitl-final-attempt \
  --cases stop-turn-nominal
```

The latest five-stop/reversal artifact passed its offline audit: 1,907 route
samples, five phases, maximum reconstruction error `4.450e-13`; root-extrema speed
`0.8816121 m/s`, acceleration `0.1902926 m/s²`, jerk `0.08362957 m/s³`.
It failed before creating a simulator process because the historical compiled
PX4 tree is absent, specifically `build/px4_sitl_default/rootfs/gz_env.sh`.
Neither `gz` nor `px4` is on PATH. The failed status and exact artifact are retained.
An earlier attempt is also preserved in `sitl-attempt/`. No new takeoff, tracking,
landing, disarm or motor-stop result is claimed.

## Historical measured-versus-reference plots

`archived-evidence/` re-analyzes the repository's retained October 3 offset-yaw
recording (`source_sha256=9831bcc5eac74a0a49ec3d5e8889c086705cf161f07d42d879ab2cb8eba2b559`).
It contains position, velocity, attitude, equal-scale 3D, ground-track/error plots
and phase-specific metrics. Route-phase 3D position error: RMS `0.15093 m`, p95
`0.29999 m`, maximum `0.51706 m` across 2,536 retained aligned truth samples.
The largest telemetry gap over the recording was `0.516952 s`. Phase shading and
gap markers are shown. Missing truth after contact remains missing.

Velocity/attitude plots use available PX4 estimates and are labeled accordingly;
raw ULog velocity/acceleration/quaternion truth and desired acceleration/yaw are
not present in this retained CSV. No missing channel is fabricated. Alignment is
inherited from the archived CSV; this reanalysis does not independently recreate
the original ULog alignment or qualify the latest implementation.

```bash
python analysis/retained_tracking_report.py \
  --source validation/2026-10-03/cases/offset-yaw-final/telemetry_and_truth.csv.gz \
  --recording-label 'Archived PX4/Gazebo recording, 3 October 2026' \
  --output /absolute/new/archived-evidence
```

## Failures and remaining work

The first broad test attempt encountered 21 import errors because pymavlink was
missing. Dependencies were installed, then full validation passed. An initial
adapter fixture expected 20 m while its source intent specified 17 m; the test
was corrected to bind to the actual declared intent. No production height was
changed to satisfy the assertion. Original motion regression and strict/fallback
acceptance policies remain binding. Existing lint failures in closest mode were
resolved with explicit zip-length semantics and removal of an unused test value.

The final local runtime is Python 3.12 with the exact package versions recorded
in `environment-and-source.json`; existing CI's Python 3.10 environment remains a
separate check. Physical-model primitive diversity, true connected fallback
tracking, heading-mode launch seams, explicit dwell in the drawing UI, general
legacy-native between-row C3 execution, higher-speed native policy qualification,
and hosted Gazebo remain open. Original named waypoint CLI dwell support is
preserved. No physical aircraft was tested or authorized.

Conclusion: offline planner/visualization and experimental artifact integration
have stronger reproducible evidence. **Reliable SITL research acceptance for the
new planner family is not yet established.** Run the declared connected campaigns
on an accessible compiled PX4/Gazebo environment before promoting the adapter or
presenting the animation as a newly simulated flight.

## CI scheduling correction

On commit `ce6eebccb6b03a2884b0b3016d4aa91e718f9504`, GitHub's Controller
validation passed on Python 3.10. Validation run `40170468274` passed static
checks, all deterministic tests and connector tests, then reached its 15-minute
job limit during a second complete test pass for branch coverage. This was an
incomplete coverage result, not an assertion failure or a coverage pass.

The follow-up workflow invokes `scripts/validate.py --controller-coverage` so
its one full deterministic test pass also collects controller coverage. The
critical 80% report gate and separate waypoint 90% gate are unchanged. Test
discovery, generator/determinism checks and the 15-minute limit are retained.
No test was removed to accommodate the timeout.
