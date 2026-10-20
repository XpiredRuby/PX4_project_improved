# Studio usability pass — 20 October 2026

The public workspace now presents one plan card. Its default view shows the full mission duration, closer-fit outcome and ground-to-ground sequence. Flight stages, heading, the drawing close-up, motion limits and engineering evidence expand inside that card. The amber drawing shadow and blue planned route remain on the main map.

One Download menu contains full mission CSV, route image and the plan/checks report. The separate route-only artifact for the native runner remains under Engineering details. Existing button handlers and artifact contents are retained.

Drafts save automatically to this browser's local storage after completed edits, with a versioned schema. Restore waits for current backend point and radius limits, validates point order, annotations and allowed form values, and ignores unknown certificate or job properties. It never restores accepted results, confirmation or flight authority. A late settings response cannot overwrite edits already made on this page. Unfinished strokes are not saved. Blocked storage leaves editing available and reports that saving failed.

Errors explain common timing, point count, boundary and planning failures, with shortcuts to points/stops and flight settings. Exact problem-corner highlighting is not claimed: the backend does not consistently provide a certified spatial location for a refusal. Technical reports remain available.

## Validation

- Portable public build passed and includes the new presentation module alongside the same 27 planner/configuration files.
- All 24 JavaScript tests passed: existing drawing, comparison and browser adapter coverage plus five draft/recovery/formatting tests.
- A real native straight-route compile was accepted and produced a ground-to-ground reference lasting 260.7662052468239 seconds.
- A jsdom integration check ran the generated page scripts in document order with mocked canvas and API transport. It exercised the compact accepted result using that real compiler response, preservation of download handlers, draft round-trip, corrupt and blocked storage, late-settings recovery, and invalidation after edits. Results and confirmation stayed cleared after recovery.
- Real-browser appearance, responsive rendering and live hosted behavior have not been checked in this pass. The managed execution/browser environment was unavailable.

## Publication status

Source is prepared for the existing public Site. Publication of this pass is pending because the supported Site source helper could not run in the unavailable managed execution environment. The existing live Site is unchanged. The isolated Windows working folder and GitHub review branch hold these changes.
