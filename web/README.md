# Trajectory Studio public planner

Open on a modern phone, tablet or computer. No PX4 laptop or account is required. Draw a route, plan and verify, review checks, and download CSV or evidence JSON. The first plan downloads pinned Pyodide 0.27.7 and NumPy from jsDelivr; an internet connection and WebAssembly/Web Worker support are required. Device performance varies; a 120-second worker budget aborts slow requests and enables retry.

This is the existing PX4_project_improved Python/NumPy planner, compiled in the browser with WebAssembly, not a replacement JavaScript approximation. Bundled Python/config files have SHA-256 integrity checks. Native editor and browser share mission_planner/preview.py. Source baseline: XpiredRuby/PX4_project_improved @154232f97172056e1c74bb7a4b18f5eaa66f0ceb, plus portable preview extraction and public adapter.

Build: `python web/build_public_studio.py dist` (Python and NumPy). Buildless hosted output is `dist/`. `.openai/hosting.json` retains the Site project identity.

## Cloud Gazebo remains unconnected

No Gazebo/PX4 process runs in Sites or in this browser. No simulator endpoint or laptop tunnel is exposed. Planning acceptance is mathematical model verification, not flight evidence. Cloud Gazebo requires a separately hosted Linux worker and a durable job/artifact service. Before enabling it:

- Compile and retain an authoritative artifact on the cloud worker; browser/native floating point exports need not be byte-identical. Show the authoritative route and digest for explicit user confirmation.
- Bind execution to the retained plan ID and trajectory digest. Never silently recompile a confirmed plan.
- Isolate jobs and permit cancellation/recovery only by the job owner, with rate limits and a bounded queue.
- Run PX4 SITL and Gazebo headless in an appropriately sized Linux container/VM. Retain telemetry and independent acceptance results. Confirm virtual landing/disarm before releasing a worker.
- Add secure result storage and browser status polling. An animation or a downloaded CSV must never be reported as a Gazebo run.

Nothing here controls physical aircraft. All output remains SITL_ONLY. Existing native Gazebo functionality remains on the project branch; the public site does not pretend to provide it.
