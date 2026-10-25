# App audit and offline release candidate — 25 October 2026

Baseline Git commit: `7459067894ccfe1a3a7a19ef3a8c7bc45fee4021`.
Final portable build ID: `df15b1363bd45ece`.
Audit before implementation: `docs/app_release_audit.md` and
`source-inventory-before.json` (188 active source/config/test files).

No Gazebo/PX4 or physical aircraft was launched in this increment. Connected
validation was explicitly deferred by the user. The public production Site has
not been updated: owned/editable Sites lists returned no accessible Site and the
checkout has no retained project ID. The session preview is internal QA only;
`terminal.local` is not a permanent public application URL.

## Executed results

| Check | Result / retained evidence |
|---|---|
| Complete Python validation | 432 passed in 414.263 s; `python-validation.txt`, `VALIDATION_PASSED` |
| JavaScript logic | 56 passed; `javascript-tests.txt` |
| Release boundary/build tests | 6 passed; `release-boundary-tests.txt`; deterministic repeat builds, asset hashes, stale-file removal, failed-build rollback, forbidden source output, original/effective motion, malformed input, >48 marked points |
| Repeated closest-feasible corpus | 46/46 passed (23 cases twice), matching CSV hashes per pair; `repeated-plans/results.json` |
| Strict planner acceptance | 16/16 expected accepted/refused outcomes; `strict-acceptance/report.json` |
| Connector subprocess suite | 5 passed; `connector-tests.txt`; no connected simulator execution |
| Ruff | Passed across planner/web Python/controller/analysis/scripts/tests/generator/tools; `lint.txt` |
| Selected controller coverage | 89% combined statement/branch report with branch measurement enabled; retained 80% gate; `controller-coverage.txt`. Not a pure branch percentage or quality score. |
| Real Chrome/Pyodide | 2 m/s plus custom 1.25 cap explicitly refused before compile; Use aircraft model cap resolved it; straight plan peak 1.80 m/s. Final handwriting result below. |

Final browser observations, retained in `browser-final-record.json`:

- Requested cruise 2 m/s, Aircraft cap policy, Never exceed 3 m/s.
- Handwriting: 481 samples → 12 flight vertices; 8,255 mission CSV samples.
- Planning cap 1.80 m/s; sampled route peak 1.76 m/s; full duration 412.60 s.
- State: Adjusted reference, with explicit stops/geometry changes, no 1.25 cap message.
- Exact sample 1271 at 63.50 s: N=0, E=0, Up=20 m, speed=0;
  heading 337.7 degrees, yaw rate -37.30 degrees/s. Stationary yaw is visible.
- Height and 3D view controls operated. `final-app-summary.jpg` shows route,
  speed explanation and paused turn; `final-app.jpg` shows height/phase evidence from the preceding build.
- No genuine measured track was loaded. No new tracking error or simulated
  landing/disarm claim is made.

## Failures and limits retained

1. Initial refreshed-runtime baseline lacked pymavlink/pyulog/ruff and ended with
   22 import errors (`baseline-incomplete-python.txt`). Dependencies were installed
   from requirements-dev.txt. A later browser timeout reset the runtime; final
   validation used a dedicated scratch virtual environment, with exact versions
   recorded in `environment-and-build.json`.
2. First implementation full run: 432 tests, one error in the incomplete aircraft
   intake path (`python-first-pass-failure.txt`). Early route validation had hidden
   the missing-profile questions. Corrected the ordering and reran all 432 tests
   successfully. `runtime-reset-failure.txt` retains the environment-reset attempt.
3. Browser initialization caught a ReferenceError in the new speed policy module.
   Corrected its factory binding and added a browser-DOM initialization/legacy
   restoration regression. Final real browser speed and handwriting tests pass.
4. Two browser file-chooser calls timed out and reset their sessions. Live project
   import/export, telemetry file import, mobile browser and cross-browser behavior
   were not newly qualified. Parser/schema/legacy-migration/gap behavior are
   covered by deterministic tests; those are not an end-to-end browser pass.
5. The selected x500 model ceiling remains 3 m/s and the existing 10% timing margin
   remains. Cruise is an upper preference, not guaranteed constant speed. Native
   runner 1.0/1.25 m/s policies are unchanged and have not been requalified here.
6. This finite audit does not prove defect absence, global path optimality,
   industrial certification, physical-flight suitability or release readiness.
   Remaining public-release gates include actual Site deployment access, live
   file-picker/mobile/browser compatibility checks and deployment smoke testing.
   Connected execution remains a separate, later qualification task.

## Reproduce

```bash
python -m pip install -r requirements-dev.txt
python scripts/validate.py --controller-coverage
node --test tests/*.test.cjs
python -m unittest tests.test_app_release -v
python -m ruff check mission_planner web controller analysis scripts tests trajectory_generator tools
python scripts/check_conference_planner.py --output /absolute/new/repeated-plans --repeats 2
python scripts/check_trajectory_acceptance.py --output /absolute/new/strict-acceptance
python -m unittest discover -s tools/powershell_connector -p test_server.py -v
python web/build_public_studio.py dist
```

The repeated-path corpus includes requested speeds 0.5, 1, 1.25, 1.5, 2, 3, 5;
explicit lower cap; heights 0.1, 1, 30, 50; five stops, reversal, hover, tiny detail,
figure-eight, 10,000-point input, handwriting, envelope repair, impossible deadline,
and one/two-lap circles. Full generated CSVs remain reproducible intermediates;
tracked JSON results retain every reported metric and deterministic hash.

Final UI-only follow-up: incomplete custom-aircraft setup no longer throws during
speed-cap refresh. The hint shows the missing setup, clears an unavailable
automatic ceiling, and updates after profile checking/recovery. Browser verified
custom-unchecked → default-model recovery and handwriting again. The existing DOM
initialization regression now tests the missing-model and changed-ceiling paths;
all 56 JavaScript tests and six release boundary tests were rerun and passed.
The complete 432-test Python result above precedes this UI-only follow-up; no
Python planner/controller source changed afterward. Latest CI must be read from
the final commit, not inferred from the preceding green or pending runs.
