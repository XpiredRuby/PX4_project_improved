# Public Trajectory Studio

The public website removes the PX4 laptop requirement for drawing, planning, model verification and CSV/evidence downloads. The Python/NumPy compiler runs in a Web Worker using pinned Pyodide 0.27.7. There is no laptop tunnel and no account required to view the public Site.

Open on a modern browser with WebAssembly and Web Workers. Internet is required to load the app and download the planning runtime from jsDelivr. The first plan takes longer; keep the page open. Performance depends on the device. Slow computations stop at 120 seconds and can be retried with a simpler route.

Source is in `web/`, with `mission_planner/preview.py` shared by the local server. Run `python web/build_public_studio.py /absolute/path/to/dist`. The build follows Python module dependencies and includes only planner modules/configuration, never the flight runner. Source/configuration integrity is checked before imports. Output remains SITL_ONLY and simulation_status NOT_RUN.

The public Site source is independently versioned with Sites. Project ID: appgprj_6ad6da5ad64c81918224161704fed714. Public access was explicitly requested by the user. Keep this identity when updating; do not create a second Site.

## Gazebo hosting gap

Real PX4/Gazebo execution still needs a connected cloud Linux worker. Sites does not run Gazebo. The public website disables simulation until that service exists. No real Gazebo result is claimed for this deployment. See `web/README.md` for the job, artifact, ownership and landing/disarm requirements before enabling a hosted simulator. Railway was suggested as a possible compute connection; it is not confirmed connected or provisioned.

## Validation of the portable change

- 11 native editor/compiler/HTTP-boundary tests passed after extracting the shared preview.
- 15 aircraft-profile tests passed.
- 11 JavaScript drawing tests and 4 public adapter tests passed. The adapter tests cover disabled simulator authority, single-worker serialization, stale messages, request-size bounds, timeout termination and retry.
- Real Chrome preview loaded Pyodide and NumPy over HTTPS and compiled a straight route (25.2 s / 505 samples), a rounded L route (41.7 s / 836 samples), and a marked-stop L route (36.5 s / 733 samples). All displayed Plan verified, NOT_RUN, with simulation disabled. CSV download completed.
- Bundle SHA-256 manifest matches the shared planner source. A null simulation-overlay guard was fixed for plans without a server-issued plan ID.

Earlier native Gazebo validation remains documented in `validation/2026-10-19-studio-usability/`; it does not validate a future hosted Gazebo service. No Gazebo rerun was needed for the portable compiler extraction. Phone layout inherits the existing responsive drawing UI; a physical phone test of this public deployment remains for user acceptance.

## Workspace layout update

The public interface now prioritizes the route canvas with a compact plan review rail. It has one visible planning action, a native settings drawer for Flight / Points & stops / Model, on-demand charts and assumptions, compact example selection, and a persistent planning action on phones. See `web/workspace-validation.md` for the real compiler, deadline, keyboard, download and responsive viewport checks. Physical phone hardware testing remains separate from browser viewport testing.
