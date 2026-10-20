# Public workspace redesign validation

2026-10-19, public Site source based on d52f7d024b4ce7577cd2b829fc6a1d50762e087f.

- Chrome desktop: one visible primary planning action, no duplicate element IDs, no horizontal overflow. Advanced controls are in a native modal drawer; simulation and diagnostics are collapsed.
- Real Pyodide compiler: rounded L verified (41.7 s / 836 samples), annotated L stop verified (36.5 s / 733 samples), changed flight height reflected in accepted result. The plan JSON report downloaded.
- A 1-second deadline was refused with the actual minimum candidate duration and a clear instruction to remove or extend it. The UI returned to its enabled planning action.
- Responsive QA used the unchanged app inside 390 px and 768 px iframe viewports. Phone content had no horizontal overflow, a persistent planning action, usable settings categories and route example controls. Phone straight route verified (25.2 s / 505 samples). This is browser viewport testing, not physical phone hardware testing.
- Modal Escape closed the drawer and restored focus to its trigger. Settings categories, point annotations and model controls remained available. Native dialog focus containment is retained.
- All 11 drawing model tests and 4 public worker-adapter tests passed. JavaScript syntax and Python build syntax checked. Planner mathematics, worker authority and Gazebo availability were not changed.

The temporary responsive QA wrapper was removed before packaging. The normal index.html was restored. Only the actual site and its runtime assets ship.
