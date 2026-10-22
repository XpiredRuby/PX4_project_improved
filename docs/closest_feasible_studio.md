# Closest-feasible studio planning — 20 October 2026

The studio now opts into `planning_mode: "closest"`. Existing API callers that omit it keep the strict schema and refusal behavior.

A drawing is a target. The planner first tries the smooth fit when appropriate. It may allow more time and then construct a seventh-order stop/turn trajectory along the same ordered polyline. Position, velocity, acceleration and jerk join continuously; heading turns while stationary instead of jumping or forcing every yaw value to zero. Repeated points can produce a hover. Stops may replace impossible exact fly-throughs and are reported.

Difficult reversals, zero-rounding drawings, long paths and dense input can use the constructive planner immediately to avoid an expensive unsuccessful spline search. Every polynomial phase uses the existing continuous Bernstein checker for speed, acceleration, jerk, heading derivatives, tilt, thrust, body rate, rotor authority and the convex operating envelope. CSV values, clocks and phase joins are checked before release. Full missions retain ground-zero takeoff and landing references.

Finite speed/height preferences and out-of-bounds coordinates may be adjusted to the unchanged aircraft/environment limits. When resource or declared energy limits require a smaller route, the bounded fallback refines the largest feasible uniform scale in its family. It preserves vertex/lap order, reports the scale and includes a monotone displacement bound. This is the closest route found in the implemented family, not a proof of a global optimum. Invalid/missing model data and genuinely infeasible models are not fabricated into accepted flight plans.

The original request and effective intent stay separate in the JSON report. The amber shadow uses the original path, and comparison bounds include the explicit repair displacement and circle discretization where relevant. The interface reports added stops, extra time and geometry changes, including when the original rounding allowance is exceeded. Requested settings are not silently overwritten.

## Validation completed

- 16 closest-preview tests passed (including the persisted-plan startup guard): actual reversal and impossible-deadline compilation; exact corners; duplicate-point hover; envelope projection; launch legs; non-positive preferences; two laps; tiny circles; resource-scale refinement; declared energy/uncertainty; invalid data; strict-mode behavior; and compatibility refusal.
- 11 existing editor/API tests and five existing lifecycle tests passed.
- All 26 JavaScript tests passed, including preservation of the original shadow and rejection of an uncertified adjustment bound.
- A generated-page jsdom integration check passed using real reversal and projected-route responses. It checked closest-mode requests, adjustment summaries, original shadow, warning when the requested allowance is exceeded, finite canvas coordinates, draft recovery and stale-result invalidation. Canvas and transport were mocked in that integration check. The subsequent recovery validation below also exercised the real browser and Pyodide worker.
- The portable build includes 28 planner/configuration files. Public HTTPS delivery and every manifest hash passed.

## Recovery validation — 20 October 2026

The current source from GitHub commit `23b79618fb9579070a03aeacfd98baa940b85308` was restored into the permanent Site checkout. The portable build was rebuilt successfully with 28 planner/configuration files, including the closest-feasible planner and simplified interface.

All 58 selected regression tests passed in the managed environment: 16 closest-preview, 11 editor/API, five lifecycle, and 26 JavaScript tests. The persisted-plan startup guard is now exercised by a passing regression. The complete native simulator/runner campaign was not run.

Actual Chrome browser validation used the compiled static app with its real Pyodide/NumPy worker, without mocked transport or canvas. An L-shaped example completed successfully. An ordered reversal `(0,0) → (0,3) → (0,0)` completed through the stop-and-turn fallback, displayed the changes and original/plan overlay, and offered working CSV and JSON downloads. Downloaded artifacts and a browser screenshot are in `validation/2026-10-20-recovery/`.

The downloaded reversal report and CSV were cross-checked: acceptance passed; all continuous phase checks passed; the CSV hash matched the report; 5,144 samples had strictly increasing timestamps; both endpoints were stationary at the origin with ground height zero; maximum height was 20 m; yaw range was π radians; full mission duration was 257.15 s.

The temporary laptop/tunnel link is superseded by the permanent Site:
https://trajectory-studio.dainty-bell-1590.chatgpt.site

This browser-only planner does not require the PX4 laptop or tunnel. The first run downloads the Python runtime and may take noticeably longer; planning requires network access for those runtime dependencies. These checks validate planning and downloadable mission data, not executed flight tracking.

## Execution scope and remaining checks

Constructive plans have a different certificate from the existing spline runner. They remain offline previews/downloads; Gazebo playback has not been connected or qualified. A persisted-plan guard refuses stop/turn startup before issuing an acknowledgement. No Gazebo or physical-aircraft run is claimed.

Manual checks: draw a reversal or tiny zigzag; use a very short route deadline; set rounding to zero; enter an out-of-bounds point. Confirm a feasible adjusted plan is shown with amber/blue comparison, disclosed changes, changing yaw when turns are required, and ground-zero endpoints in the full mission CSV.

## Handwriting and scribble repair

Freehand capture now accepts separate strokes anywhere on the map and connects them in drawing order, with an explanatory hint. The UI no longer treats raw mouse events as a 256-waypoint flight contract. It retains up to 8,192 samples for comparison and automatically thins exceptionally long gestures in order, with a visible notice. Undo/redo retains complete strokes. Single-point drawings can produce a hover reference. Loop annotations are accepted in best-effort capture.

Closest planning parses the full drawing contract (up to its existing 10,000-point schema limit), then selects significant bends within a 48-vertex working budget for dense drawings. Explicit marked points are retained when they fit that budget. The original request stays separate. A conservative continuous monotone arc-length coupling bounds changes even when the two polylines have different vertex counts. This replaces the old refusal at 256 raw input points. Complex sketches use the checked stop/turn construction directly. The existing time, heading and scale fallback remains available. Shape simplification and changes beyond the requested allowance are disclosed. A global closest-trajectory optimum is not claimed.

Browser and local transport/draft recovery limits accommodate the captured samples. The studio includes Examples → Vin, using 481 samples; the actual Chrome/Pyodide run produced a verified 12-vertex route with its shadow retained, a changing heading and stationary ground-zero endpoints. CSV hash, clocks and full mission endpoints were cross-checked against the downloaded JSON report. Metrics and screenshot are in `validation/handwriting/`.

Validation for this change: 19 closest-preview tests, 29 JavaScript tests and 11 editor/API tests passed (59 total). Cases include dense Vin handwriting, self-crossing out-of-area scribbles, continuous order-aware repair bounds, disconnected-letter capture, 10,000-event capture, undo/redo, and larger worker payloads. The browser example and JSON/CSV downloads also passed. Malformed data, missing physical model evidence, runtime/network failures and genuinely impossible model limits still produce truthful errors; no invalid plan is relabeled as verified.
