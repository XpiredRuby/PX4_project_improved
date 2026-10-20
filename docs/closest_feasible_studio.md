# Closest-feasible studio planning — 20 October 2026

The studio now opts into `planning_mode: "closest"`. Existing API callers that omit it keep the strict schema and refusal behavior.

A drawing is a target. The planner first tries the smooth fit when appropriate. It may allow more time and then construct a seventh-order stop/turn trajectory along the same ordered polyline. Position, velocity, acceleration and jerk join continuously; heading turns while stationary instead of jumping or forcing every yaw value to zero. Repeated points can produce a hover. Stops may replace impossible exact fly-throughs and are reported.

Difficult reversals, zero-rounding drawings, long paths and dense input can use the constructive planner immediately to avoid an expensive unsuccessful spline search. Every polynomial phase uses the existing continuous Bernstein checker for speed, acceleration, jerk, heading derivatives, tilt, thrust, body rate, rotor authority and the convex operating envelope. CSV values, clocks and phase joins are checked before release. Full missions retain ground-zero takeoff and landing references.

Finite speed/height preferences and out-of-bounds coordinates may be adjusted to the unchanged aircraft/environment limits. When resource or declared energy limits require a smaller route, the bounded fallback refines the largest feasible uniform scale in its family. It preserves vertex/lap order, reports the scale and includes a monotone displacement bound. This is the closest route found in the implemented family, not a proof of a global optimum. Invalid/missing model data and genuinely infeasible models are not fabricated into accepted flight plans.

The original request and effective intent stay separate in the JSON report. The amber shadow uses the original path, and comparison bounds include the explicit repair displacement and circle discretization where relevant. The interface reports added stops, extra time and geometry changes, including when the original rounding allowance is exceeded. Requested settings are not silently overwritten.

## Validation completed

- 15 closest-preview tests passed: actual reversal and impossible-deadline compilation; exact corners; duplicate-point hover; envelope projection; launch legs; non-positive preferences; two laps; tiny circles; resource-scale refinement; declared energy/uncertainty; invalid data; strict-mode behavior; and compatibility refusal.
- 11 existing editor/API tests and five existing lifecycle tests passed.
- All 26 JavaScript tests passed, including preservation of the original shadow and rejection of an uncertified adjustment bound.
- A generated-page jsdom integration check passed using real reversal and projected-route responses. It checked closest-mode requests, adjustment summaries, original shadow, warning when the requested allowance is exceeded, finite canvas coordinates, draft recovery and stale-result invalidation. Canvas and transport were mocked; no real-browser visual or browser-Pyodide run is claimed.
- The portable build includes 28 planner/configuration files. Public HTTPS delivery and every manifest hash passed.

## Execution scope and remaining checks

Constructive plans have a different certificate from the existing spline runner. They remain offline previews/downloads; Gazebo playback has not been connected or qualified. A persisted-plan guard also refuses stop/turn startup before issuing an acknowledgement. That final startup guard and its additional regression were prepared after the last successful test run and could not be re-run locally because the F: project drive began rejecting writes. The broader native runner suite also did not complete because of local dependency/WSL errors. No Gazebo or physical-aircraft run is claimed.

The tested browser bundle was rebuilt and checked before the drive lost write access. Source on the review branch additionally contains the final native startup guard. Permanent Site publication remains blocked by the unavailable managed source-helper environment; the existing permanent Site is unchanged.

Temporary test URL (requires the laptop and tunnel to remain online):
https://wellington-offerings-thu-brick.trycloudflare.com/?v=closest

Manual checks: draw a reversal or tiny zigzag; use a very short route deadline; set rounding to zero; enter an out-of-bounds point. Confirm a feasible adjusted plan is shown with amber/blue comparison, disclosed changes, changing yaw when turns are required, and ground-zero endpoints in the full mission CSV. Refresh before testing to load the new browser worker.
