# GitHub comparison — 2026-10-26

This records source and CI observations, not approval of an untested combined
revision. No peer branch, native environment or protected branch was changed.

## Current sources inspected

| Source | Exact revision | Role in this review |
|---|---|---|
| Agreed PR #6 base | `72f7b03c26e4ca20e2d45a19f272e5b1d9c4961b` | Remains the development base |
| GPT A implementation, PR #8 | `d212903f3449139ce11db7bada699492e610f684` | Tested planner/browser corrections and fixtures |
| Existing PR #7 | `72cb875c31bfd1a9a0074e7884986792ad594910` | Separately inspected prior work; not assumed to be GPT B's agreed branch |

The published branch search returned `ai/planner-review-20261026` and
`ai/conference-planner-validation-20261025`. It did not return the agreed
`ai/native-sitl-20261026` branch at the time of this check. This does not establish
anything about unpublished laptop work. PR #8 had no posted comments or review
submissions at this check.

## Remote validation of the implementation commit

Both GitHub Actions runs completed successfully for
`d212903f3449139ce11db7bada699492e610f684`:

- [Controller validation, run 141](https://github.com/XpiredRuby/PX4_project_improved/actions/runs/40293103999).
- [Validation, run 60](https://github.com/XpiredRuby/PX4_project_improved/actions/runs/40293104103).

The validation job ran 428 Python tests, the connector checks and both configured
coverage gates. Aggregate coverage in the selected controller gate was 89%; the
selected waypoint-planner gate was 91%. These percentages describe the selected
files and configured coverage metric, not project accuracy. The complete job
step results and coverage extract are retained in `github-ci.json` and
`github-coverage-extract.txt` under `validation/2026-10-26-planner-review/`.

Subsequent GPT A changes add documentation and review evidence only. The tested
production source is unchanged; do not attribute these implementation-run IDs to
a later documentation commit.

## Overlap and integration choices

| Area | Observation | Integration treatment |
|---|---|---|
| Required hard/exact points | Both PRs preserve marks. PR #7 expands the simplifier's budget to retain 49 required points; PR #8 explicitly refuses above the configured 48-point budget. Reproduced with the same marked drawing. | This is a real resource-policy difference, not a reason to drop points. GPT A retains its explicit cap. Any combined revision must choose one declared, bounded policy and update corresponding positive/negative tests. |
| Phase vehicle cap and numeric validation | PR #8 changes `lifecycle_verification.py`; PR #7 does not change it from the agreed base. | Preserve the model-cap correction and malformed-input regressions when integrating. |
| Original-request browser certificate | PR #8 changes `web/comparison.js`; PR #7 does not change it from the agreed base. | Preserve the scope/evidence guard and its missing/invalid certificate regressions. |
| Playback and measurements | PR #7 adds a separate measurement-gap policy and rendering changes. Its reference interpolation passes all 123 queries in GPT A's rational fixture. | Keep reference and measurement semantics distinct. This offline cross-check does not qualify real DOM behavior, live telemetry or a combined application build. |
| Existing closest tests | PR #7 changes nonpositive-speed handling and other preview behavior. | Preserve explicit refusal/no-mutation tests and all new model-cap/mark tests; do not blindly replace either test file. |
| Native adapter and truth changes | PR #7 also touches native/controller/analysis paths and describes an experimental adapter. | These do not count as GPT B's reviewed native result. Exact combined-commit review and raw measured evidence remain in the separate blank review record. |

A non-checkout `git merge-tree --write-tree` inspection reproduced a content
conflict in `mission_planner/closest.py`. It did not merge branches or modify
the index/worktree. The conflict record is retained in `peer-merge-conflict.txt`.
The syntactic conflict is not an integrated test result; automatically merged
files would still require semantic review and combined-revision tests.

## Independent cross-check from the peer revision

The pinned `analysis/reference_audit.py` imports no planner code. Run
`python validation/2026-10-26-planner-review/reproduce-peer-audit.py` from this
repository after fetching the exact peer revision to reproduce the recorded
experiment. The auditor source hash and reviewed implementation SHA are retained.

- The 401-row route and 1,828-row full mission both pass the sampled data audit,
  including integral consistency, speed, acceleration, jerk, yaw derivatives,
  NED thrust/tilt and operating envelopes.
- An incorrect velocity, a nonfinite position and a duplicate clock are all
  rejected. Results are retained in `peer-data-audit.json` and `.txt`.
- The peer's browser reference module also agrees with the rational oracle;
  its exact source hash and maximum component error are retained in
  `peer-browser-reference.txt`.

This is a useful independent implementation check. It remains a sampled
diagnostic and supplements the continuous certificates; it is not an independent
physical flight, complete native-adapter review, empirical model validation or
formal certification. PR #7 has not been merged into this branch.
