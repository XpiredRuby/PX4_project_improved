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

## Manual test access prepared later on 20 October

A temporary HTTPS preview of application commit 27b0dd4e2d382f2f481359b27b0e00457b2179b1 is running at:

https://wellington-offerings-thu-brick.trycloudflare.com

The preview serves only the built static studio directory. It can be opened from other devices while the connected laptop and preview processes stay online. This is not a deployment to the permanent Site; that Site remains unchanged. The current laptop also serves http://localhost:8765/.

Public HTTPS checks passed for the page, all nine UI/runtime assets, and the SHA-256 of all 27 planner/configuration files. This verifies delivery, not real-browser rendering or a browser-executed planning run.

Manual checklist:

1. Choose a straight-line or turn example, select Plan and verify, and compare the amber drawing with the blue plan.
2. Expand Flight stages and heading. Confirm planned height starts at 0, rises to the selected height and ends at 0. This is a reference trajectory, not a touchdown or Gazebo result.
3. Change height. The old result must disappear. Refresh and check that the completed route and new setting return, with a fresh plan check required.
4. Open Download. Check the full mission CSV, route image and plan report. The full mission CSV should have ground-zero height at both ends.

Gazebo remains disconnected in this browser-local preview. Permanent publication and a visual real-browser check remain outstanding.
