# Trajectory studio: drawing and planning logic

Date: 2026-10-19. Status: implementation specification, not a new validation result.

## Scope and reviewed baseline

This increment covers drawing, mission interpretation, verified trajectory generation,
review, and PX4/Gazebo simulation. Physical aircraft setup and flight are outside this
increment. The default remains the reference simulator, 20 m route height, 1 m/s
preferred cap, 1.25 m/s hard cap, 0.5 m planning tolerance, and fixed launch heading.

GitHub baseline: PR #6, branch
`codex/trajectory-simulation-acceptance-20261019`, commit
`68665ef5d8360eb27228d7496f77df0cb06873ac`.
The preserved local editor also contains an unfinished visual increment: empty map,
START/END labels, arrows, adjacent controls, and a nearby planning button. Preserve
those changes when implementing; they are not part of the cited GitHub checkpoint.

Reviewed source: `mission_planner/editor.html`, `editor.py`, `contracts.py`,
`geometry.py`, `timing.py`, `retiming.py`, `plan_drawn_path.py`,
`studio_jobs.py`, and the controller trajectory-to-return transition.

Existing checks and earlier campaign results remain their own evidence. No new
compiler, browser, simulator, or runtime validation was performed for this document.

## Findings that drive the changes

| Observed behavior | Required correction |
|---|---|
| New drawing strokes always append to the previous point, including disconnected gestures. | Separate point placement from freehand strokes and preview every intentional connector. |
| Input points closer than 0.6 m are discarded and pointer release does not explicitly save the final location. | Preserve the released endpoint and use a bounded input reduction with recorded error. |
| Points beyond 39 m are silently ignored despite a separate 40 m environment contract. | Show the configured envelope; refuse or roll back an invalid gesture with a visible explanation. |
| The UI permits 240 points while the backend permits 256. | Publish one server-derived effective limit and reserve generated closure points explicitly. |
| Drawing has only unmarked points; hard stops and exact fly-throughs exist elsewhere. | Expose their distinct semantics without changing them into one another. |
| The local label says return to start but maps to periodic spline closure. | Distinguish closing a loop from stopping at the route end and subsequently returning home. |
| Every completed trajectory enters controller-generated RETURN_HOME. | Show the complete lifecycle and identify which legs are covered by the route certificate. |
| Tangent heading can be planned offline but cannot use the connected rehearsal. | Make connected capability visible before compilation, rather than at the launch button. |
| Geometric fidelity uses a symmetric distance bound. | Add independent traversal-order checks for crossings/retracing; distance alone cannot establish visit order. |
| Some documentation still opens with aircraft setup despite the simulator-first milestone. | Update quickstart and manual trial around the same default workflow. |

## 1. Drawing interaction contract

Use two explicit tools: **Draw stroke** and **Add points**. Both edit the same ordered
route. The immutable horizontal START is (east=0, north=0). Route height is a separate
value; START on the map represents the top of takeoff at that height, not ground level.

- A fresh map contains only START, orientation, scale, configured boundary and an
  instruction. Example routes require an explicit preset action.
- In Draw stroke, start near START for the first gesture, or END for continuation.
  A small screen-space hit target may snap only the beginning of a gesture to that
  exact endpoint. It must never move an existing endpoint.
- A disconnected freehand gesture is held as a pending proposal. Show its connector
  as a dashed preview and offer **Connect from END** or **Discard stroke**. Neither
  planning nor confirmation can use an unresolved proposal.
- In Add points, hover/focus previews the next connecting segment. A click or
  keyboard-confirmed coordinate explicitly commits that visible segment. First
  placement away from START therefore has a visible connection from START.
- Pointer capture follows one primary pointer. Ignore additional touches; handle
  pointercancel/lost capture by rolling back the unfinished gesture. Commit the
  final release position even when the last movement event was not recorded.
- Use one reversible history action per completed stroke, point, preset, annotation,
  or ending change. Undo/redo restores coordinates, annotations and ending together.
  New drawing resets the route to START and clears pending proposals.
- Invalid or oversized gestures leave the last valid route intact and explain the
  reason. Never silently truncate, clamp, drop an out-of-bounds section or fit a
  different shape to satisfy limits.
- Coordinate entry and selectable waypoint rows provide a keyboard alternative to
  drawing. Touch controls do not depend on hover or color alone.
- Highlight START, END, STOP and pass-through markers; arrows indicate travel order.
  Show drawn length separately from verified trajectory sample count.

Coordinates and hit testing must use the canvas display transform consistently on
desktop and mobile. Device pixel ratio must not change metre coordinates. A proposed
endpoint snap target is 12 CSS pixels, capped at 0.5 m in world space; display the
actual snapped location before commit and cover scale changes in tests.

## 2. Raw input, normalization and tolerance

Keep the ordered raw pointer-event polyline and annotations in the draft. Consecutive
identical ordinary samples may be deduplicated; nonconsecutive repeats, crossings,
lap counts, stop locations and exact points must remain meaningful.

Do not retain the fixed 0.6 m discard rule. Initial implementation may keep all samples
and refuse an oversized gesture. If reduction is needed, use an order-preserving
polyline reducer and independently bound the deviation from the retained raw input.
Protect all gesture endpoints and annotated points.

For a declared planning tolerance T, allocate explicit reduction error R and smoothing
error S with R + S <= T. Use R=0 when there is no reduction; a proposed reduction cap
is min(0.05 m, 0.1*T). Verify both bounds, retain the raw-to-normalized provenance and
show the complete budget. A chain of smoothing against an already-reduced path must
not silently gain another full T of deviation. Any optional navigation/tracking
allowance remains a separate budget and is not promised by a route-only certificate.

Derive effective input limits from the server. Count generated closing points before
submission. Validate finite coordinates, positive height/tolerance/speeds, nonzero
route length, unique point IDs, configured volume, and preferred cap <= hard cap.
Keep authoritative server validation even when the browser has already checked.

## 3. Route endings, points and loops

| User choice | Planning interpretation | Simulation behavior |
|---|---|---|
| Finish drawn route | Open, rest-to-rest route ending at END. | Stop at END, then existing controller-managed return, landing and disarm. |
| Closed loop | Explicit closing segment to START; periodic geometry, with start/stop at its seam. | Fly the reviewed loop and laps, then lifecycle landing. |
| Stop here | Interior hard point, split into rest-to-rest pieces. | Reach the specified location with the existing stop semantics. |
| Pass exactly here | Exact interpolation point, without an added hard stop. | Visit the point; no minimum fly-through speed guarantee is implied. |
| Ordinary point | Shape guidance within the accepted tolerance. | May be rounded; it does not require a stop or exact interpolation. |

A STOP does not yet imply a dwell duration. Do not expose a dwell control until the
exporter, mission clock, verifier and execution adapter all support it.

A loop must have a visible closure proposal; endpoint proximity alone must not silently
change its type. Preserve lap count and traversal order, including self-intersections.
Keep unsupported periodic + interior hard/exact combinations explicitly refused.
Do not label a periodic seam as an ordinary straight return leg.

The review always shows: takeoff, verified route, controller-managed return if needed,
landing and shutdown. Use a distinct dashed style for the nominal return indication.
The present return starts from measured state; a drawn straight connector is only an
indicative segment, not the certified reference or a timing guarantee. Route duration
must be labeled **Route duration**; lifecycle time is separate or unavailable. Do not
claim full-lifecycle certification without implementing and checking those trajectories.

## 4. Trajectory planning and independent acceptance

Preserve the existing mathematical pipeline:

1. Freeze and normalize the draft, model, environment, height, heading and ending.
2. Construct the versioned MissionIntent, including hard/exact annotations.
3. Split hard stops, fit smooth geometry, and check the raw-input tolerance budget.
4. Propose a monotone time law; starts, ends and hard stops use the existing stationary
   boundary conditions. Slow through turns according to certified motion bounds.
5. Check speed, acceleration, jerk, attitude, thrust, rotor and yaw constraints through
   the existing independent verifier. A requested speed is a cap, not a constant pace.
6. Independently verify ordered progress and required visits as well as geometric
   proximity. For crossings, use a bounded monotone correspondence to ordered source
   segments, not nearest-segment labels alone; ambiguity must be refused or reported
   as unsupported until checked. Repeated laps cannot be certified by distance alone.
7. Enforce a requested maximum route time only after limits and fidelity. A refusal
   reports the constraint and feasible candidate duration when available, never a
   proven global optimum or an automatically increased speed/tolerance.
8. Publish CSV, certificate, normalized intent, raw-input provenance, model/environment
   hashes and review metadata together. Reduced display samples are never execution data.

Fixed launch heading remains the connected default. Offline-only heading modes are
clearly labeled; switching modes invalidates verification. Do not substitute another
heading, model, tolerance, altitude or route ending automatically.

## 5. Draft state and simulator state

Keep two independent records:

- Draft: EMPTY, EDITING, PLANNING, VERIFIED or REFUSED; each change advances a revision.
- Confirmed job: READY, STARTING, RUNNING, cancellation requested, then
  COMPLETED / CANCELLED / FAILED, with a separate assessment verdict.

Compilation operates on an immutable submitted revision. On any draft edit, immediately
clear accepted-plan authority, confirmation and draft downloads. A late response for an
older revision cannot restore them. Starting a recompile also clears confirmation.
Keep the draft and recoverable error visible after refusal.

A confirmed simulation belongs to its immutable plan ID and trajectory digest. Draft
edits, new results and tab refreshes cannot reassign it. Preserve monitoring and
cancellation while compilation runs. Serialize start requests both in the UI and backend.
Apply results to their originating job, even if the user changes the draft during start.

On a status-network failure, retain ownership and show **Status unavailable**, not
**Stopped** or **Passed**. Keep explicit retry/recovery available. Missing saved jobs
must not block new drafts; existing simulators are preserved. Do not promise cross-tab
draft persistence until it is implemented; job recovery remains separate.

A cancellation request is not a completed cancellation. Keep monitoring until the
existing landing, disarm and stopped-output evidence supports the terminal verdict.
Completion and a passing sampled assessment remain distinct.

## 6. Review layout and useful errors

Keep the map central with nearby tools, START/END, direction arrows, length and an
immediately visible Plan and verify action. Below it, show the drawn/normalized/verified
route interpretation, input reductions, ending, stops, speed cap, route duration and
binding constraint. Full certificate details remain expandable.

Use stage-specific language: **Draft**, **Planning**, **Plan verified**,
**Simulating**, **Simulation checks passed/failed**, and **Cancellation confirmed**.
Errors identify the field or gesture and an actionable correction. Preserve structured
PlanningError fields through the worker/API; do not flatten a duration refusal to an
unexplained generic worker error. Suggested corrections require user edits and review.

No simulation starts on drawing, planning, checkbox changes or refresh. The sole start
action submits the reviewed plan ID and digest, and the server checks the exact artifact.

## 7. Implementation sequence

| Stage | Deliverable | Exit condition |
|---|---|---|
| A | Extract draft reducer, coordinate transform and explicit interaction tools. Preserve the unfinished visual increment. | Gesture/history/state behavior can be verified without PX4. |
| B | Add normalization provenance, capability limits, raw-input budget and structured refusals. | Backend and UI agree; no silent geometry edits or excess tolerance. |
| C | Expose stop/exact semantics, honest ending labels and lifecycle indication. Add traversal-order acceptance. | Review and exported intent agree; unsupported requests refuse visibly. |
| D | Bind review/confirmation/job state, fix pending-start/recompile races and stale-result handling. | Editing, retry, refresh and cancellation cannot change job ownership. |
| E | Update quickstart; inspect desktop/mobile and keyboard workflows. | The first drawing needs no explanation of where to begin or what flies next. |
| F | Run focused logic checks, full repository checks, browser cases, then selected connected regressions. | New evidence is retained on the flash-drive-backed environment and pushed to GitHub. |

Stages A-E precede execution. This document completes the planning pass only; production
changes and all test/simulation runs remain pending.

## 8. Acceptance cases declared before implementation

| Group | Required cases and expected result |
|---|---|
| Starting | Empty canvas; first stroke at START; first distant stroke pending; deliberate Add points connector. No hidden connector. |
| Continuing | Valid stroke from END; disconnected proposal accept/discard; pointer release between movement samples; pointercancel; second touch. |
| Limits | Inside/outside boundary; boundary exit/re-entry within one stroke; effective point limit and generated closure; no silent partial route. |
| Small details | Sub-0.6 m corner; short final segment; repeated consecutive point; retained nonconsecutive repeated visits; explicit reduction error bound. |
| History | Undo/redo stroke, point, stop, exact marker, preset, ending and New drawing. START remains fixed. |
| Geometry | Straight, L, S, tight U-turn, figure-eight, retraced segment, loop, multiple laps; verify ordered traversal or explicit refusal. |
| Point semantics | Hard stop, exact point, ordinary point, conflicting flags, unsupported periodic annotations. Dwell is not implied. |
| Motion/time | Lower speed cap; preferred above hard cap refused; impossible deadline refused with structured explanation; infeasible tolerance preserved. |
| Lifecycle | Open route END is distinct from automatic return; closed loop closure visible; route time differs from total simulation time. |
| Revision races | Edit during compile; two compile requests; recompile accepted draft; double start; edit during start; late response; assessment for older plan. |
| Recovery | Refresh during job; missing saved job; network loss and retry; pending cancellation; failure without full shutdown evidence cannot pass. |
| Presentation | Desktop/mobile coordinates agree; final endpoint retained; keyboard alternative; no color-only state; no automatic simulator start. |
| Connected regression | Reuse nominal turn, hard stop, circle and open-end missions plus cancellation; add figure-eight only if ordered checking accepts it. Existing thresholds remain fixed before running. |

Retain the original failed attempts and new outcomes separately. A passing test matrix
supports only its named cases and thresholds; it is not a percentage accuracy claim.
