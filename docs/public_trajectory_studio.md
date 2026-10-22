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

## Compare the drawing with the planned route

After verification, the original input remains visible as an amber shadow and dashed line under the blue plan. Overlay, Drawing only and Plan only alter the view, never the trajectory. Fit route includes both inputs and the planned samples, while a magnified close-up makes centimetre-scale rounding visible without exaggerating the main plot.

Largest displayed gap is an approximate top-view nearest-segment distance among the display samples. Verified upper bound is derived from the independent INT-007 certificate and includes the controller interpolation allowance; it is not flight tracking error. The input rounding allowance is shown beside it. Exported PNGs identify the view and planning-only scope. See `web/comparison-validation.md` and the new geometry/rendering tests.


## Ground-to-ground mission reference (2026-10-20)

The public review now plots and downloads the complete planned lifecycle: stationary at launch (north/east/down = 0/0/0), smooth takeoff to the selected height, the verified route, an optional stationary turn toward launch, return to launch and smooth descent to ground. Charts use positive height. The full mission CSV retains PX4 LOCAL_NED z (negative above launch) and adds positive `height_m` and `phase` columns. Fixed heading is explicit, including an editable angle; public planning defaults to facing along the route. A straight route can correctly hold a constant outbound yaw.

`mission_planner/lifecycle.py` composes seventh-order rest-to-rest transitions with zero velocity, acceleration and jerk at their boundaries. Added phase durations are limited by translational/yaw derivatives, tilt, thrust, body rate and rotor authority. `lifecycle_verification.py` independently bounds continuous derivatives with subdivided Bernstein hulls; monotonic straight-segment position hulls check the flat ground plane and operating envelope. The original route retains its independent certificate. Full mission clocks increase strictly and endpoints are stationary at the ground origin. An aircraft profile additionally budgets full lifecycle time, settling, contingency and energy reserve.

The existing `trajectory_csv`, route samples, summary and hashes remain the exact **route-only** native-runner artifact. The added `mission` object carries the full reference, phases, samples, CSV, verification and its own hash. The default public download is `full_mission_ned.csv`; the separate developer export is `route_only_sitl.csv`. The full CSV is marked `execution_ready: false` because the current native runner already creates its own takeoff/return/landing. Feeding it the full reference would duplicate those phases.

This is a flat empty-world reference with model checks, not evidence of ground contact, landing gear behavior, ground effect, disarm or motor shutdown. `simulation_status` remains `NOT_RUN`. Mapped-obstacle missions are refused until lifecycle clearance can be certified. The public site still has no hosted Gazebo worker. Existing native Gazebo orchestration is unchanged.

Heading fixes accompanying the explicit policy: stationary yaw adds no duration constraint to a moving straight path; sharp curves may refine the continuous heading-error certificate from 8,000 to 32,000 samples while retaining the two-degree threshold. The independent route verifier repeats the refinement; rate, acceleration and jerk limits still apply. Truth samples are reused within profile fitting to avoid duplicate evaluations.


## Fidelity-first update (2026-10-20)

The fitter now targets one quarter of the hard rounding allowance, ranks a bounded candidate family by target excess before smoothness, and reports any hard-only fallback. Order-aware Fréchet acceptance, a separate Cox-basis evaluator, serialized-reference checks and an anchored periodic launch seam supplement the existing acceptance logic. See `docs/fidelity_first_phase1.md` for the exact scope, resource budgets, numerical methods and remaining research work. Hosted Gazebo remains Phase 2.
