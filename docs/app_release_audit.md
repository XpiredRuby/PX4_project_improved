# App release audit — 25 October 2026

Baseline: 7459067894ccfe1a3a7a19ef3a8c7bc45fee4021. This audit precedes implementation. Gazebo/PX4 execution is explicitly deferred by the user.

Scope: repository-wide active-source inventory, Python AST and JavaScript syntax checks, dangerous-operation and exception/input scans, existing tests and CI, and focused manual review of contracts, geometry/retiming/verification boundaries, closest-path construction, lifecycle composition, execution guards, browser worker, draft recovery, playback, export and static build. Historical research archives are retained unchanged. The inventory records exact source hashes. This does not mean every possible behavior has been proved correct.

| ID | Priority | Confirmed finding / reproduction | Planned correction |
|---|---|---|---|
| APP-01 | High | Version-1 saved drafts restore `maxSpeed=1.25`; changing fresh defaults does not resolve existing sessions. | Explicit aircraft/custom cap policy, legacy review notice and one-action model-limit reset. Persist the policy in version 2. |
| APP-02 | High | A custom cap below cruise silently rewrites cruise during closest preparation; message does not name binding setting. | Resolve custom conflicts before browser compilation; give requested/cap/model reasons in API adjustment metadata and prominent results. |
| APP-03 | High | Closest `_motion` accepts zero/negative speeds by flooring to 0.1. | Reject malformed speed commands with field-specific validation before planning. Ordinary finite geometry remains repairable. |
| APP-04 | High | `_simplify_sketch` abandons all mandatory marks if >48 are present; draft thinning removes marked points at alternating indices. | Always retain hard/exact points and endpoints, thin ordinary samples only; explicitly refuse storage budgets that cannot preserve mandatory marks. |
| APP-05 | Medium | Worker `postMessage` throwing leaves pending/timer/worker alive; message-deserialization errors unhandled. | Single settlement/cleanup path, error and messageerror recovery, stale-worker identity guards. |
| APP-06 | Medium | Missing/mistyped payload members can surface TypeError/KeyError inconsistently across browser and local server. | Validate request container shape before indexing; retain strict downstream contracts. |
| APP-07 | Medium | `requested_intent.motion` is constructed after clipping speeds, losing the original requested motion in that artifact. | Preserve original motion in requested artifact; keep effective intent separately bound to verification. |
| APP-08 | Medium | Measured playback interpolates across arbitrarily long telemetry gaps and fills missing derivative fields with inferred zero accelerations. | Gap-aware measurement interpolation; no unmeasured derivative fields fabricated. |
| APP-09 | Medium | Every animation frame filters full return/measured paths and redraws every height sample. | Cache display geometry; keep full-resolution rows and exact current-state sampling. |
| APP-10 | Medium | Existing output directories retain removed planner files; public build lacks visible source/build identity and mixed-version detection. | Deterministic clean portable build, dependency allowlist, source manifest identity, UI build label and runtime mismatch check. |
| APP-11 | High | CI runs only Python; JavaScript, static build integrity and public-source boundaries are not gated. | Add dedicated app validation job with JS and portable-build checks. |
| APP-12 | Medium | Result header can say Verified after effective-intent adjustments; speed explanation hidden in collapsed details. | Use Adjusted reference state and persistent requested/achieved speed summary. |

Release blockers outside this implementation: existing public Site is not editable in connected account (owned/editable list returned empty); no permanent update may be claimed. Connected behavior of the latest constructive family remains unqualified. Industrial/physical-flight readiness is not inferred from offline tests. No blanket “all issues fixed” claim is justified by a finite audit.

Baseline test attempt in the refreshed runtime encountered missing pymavlink/pyulog/ruff. Install the declared development requirements, record the incomplete attempt and run final checks with the complete environment. Historical controller/native speed policies and physical-flight guards will remain unchanged.

Reference guidance: OWASP Input Validation Cheat Sheet (https://cheatsheetseries.owasp.org/cheatsheets/Input_Validation_Cheat_Sheet.html) and MDN Using Web Workers / Worker messageerror (https://developer.mozilla.org/en-US/docs/Web/API/Web_Workers_API/Using_web_workers and https://developer.mozilla.org/en-US/docs/Web/API/Worker/messageerror_event). These support validation and worker cleanup practices; they do not certify this application.

## Implemented resolution and verification

APP-01/02: explicit Aircraft model / Custom cap policy, legacy version-1 notice,
version-2 saved policy, one-action reset and precompile conflict refusal. Model
ceilings never increase. Requested speed remains separate from the effective
intent and is visible beside the sampled planned peak. The existing 10% timing
margin is retained and explicitly shown before and after planning.

APP-03/06/07: boundary shape/positive finite speed validation; small positive
caps are no longer raised to 0.1; original requested motion is retained separately.
Incomplete aircraft intake still returns its missing questions before route
validation. A legacy test that expected negative speed to become positive was
replaced with an explicit refusal/no-mutation assertion; repairable finite
geometry continues through the closest-feasible planner.

APP-04: endpoints and all hard/exact marks survive both browser thinning and
Python simplification, including >48 mandatory marks. Too many mandatory marks
for browser storage fail explicitly without altering the previous route/history.
Malformed mark arrays are refused before thinning so they cannot cause a stalled
reduction loop.

APP-05/08/09: worker send/error/messageerror cleanup and stale identity checks;
measurement gaps >0.5 s display no inferred position, and missing derivatives
are not fabricated. This is a visualization gap policy, not a SITL acceptance
threshold. Cached display geometry retains the full-resolution state source and
terminal endpoint; visual line decimation is not a replacement for verification.

APP-10/11/12: staged deterministic builds replace only recognized build outputs,
remove stale executable files, preserve a failed build's previous output, prohibit
known execution modules in the portable import closure, hash emitted assets,
cache-key scripts/styles and bind worker manifest identity to the page build.
Build identity is visible in the footer and included in browser plan exports.
CI now includes JavaScript, public build and release-boundary tests. State labels
distinguish Model checked and Adjusted reference.

Added editable project export/import with strict route/settings parsing and
version-1 migration, invalidation of old checks and protection against edits
during file read. This transfers geometry/preferences only, not simulation or
flight authority. **The live file-chooser flow remains unverified:** two browser
automation calls timed out and reset their sessions. Parser, policy initialization,
legacy migration and draft-safety tests pass; these do not substitute for an
end-to-end file-picker pass on supported browsers. No production-readiness claim
is made for the whole app.

Final evidence is in `validation/2026-10-25-app-audit/README.md`. No new simulation
or physical flight was attempted. No plotting of synthetic measurements as truth.

### Final settings review follow-up (APP-13)

An unchecked custom model can make `selectedAircraftSetup` throw. The new speed
refresh originally accessed it outside its error boundary, and completed profile
checks did not immediately refresh the displayed automatic cap. Moved the entire
lookup inside the controlled refresh, clear unavailable automatic caps, and
refresh on editor invalidation. Browser verified the incomplete-model message
and default-model recovery; DOM regression covers a model changing its ceiling.
No physical limit or planner equation changed. This finding was discovered and
resolved during final review, after the initial audit table.

Final CI caught an incomplete mock DOM interface in the APP-13 regression
(classList.add missing). The test fixture was corrected; the application error
boundary was unchanged. A combined local shell command had hidden the failed
JavaScript exit status behind a later build success. Final JS validation is now
run as a separate command with its own exit status. The failed output and
correction are retained, rather than treating the earlier 55/56 run as a pass.
