# Ground-to-ground lifecycle validation — 2026-10-20

Changes: full mission reference, positive-height plots, ground endpoints, default full mission CSV, explicit heading settings, stationary-yaw duration fix and refined heading certificate.

- 45 Python tests passed: native editor/HTTP integration, lifecycle, route verification, retiming and input/evidence suites.
- 27 Python tests passed after the heading-fit evaluation optimization: lifecycle, aircraft profiles and path timing.
- Final 5 lifecycle regressions passed, including the actual L-turn preset with a varying route heading and the unchanged two-degree independent heading-error requirement.
- 19 JavaScript tests passed: drawing state/history/boundaries, worker serialization/timeout/staleness, comparison geometry and rendering.
- Python compilation and JavaScript syntax checks passed. The portable builder includes 26 Python files.
- Native L-turn mission example: 325.278 s including TAKEOFF, ROUTE, TURN_HOME, RETURN_HOME and LAND. First/final position and first three translational derivatives are zero. Full mission timestamps increase strictly. Route-only fields retain exact values and their own SHA-256; mission CSV has a distinct hash.
- Added continuous phase bounds cover speed, acceleration, jerk, yaw rate/acceleration/jerk, attitude/thrust/body rate, rotor authority, the flat ground plane and spatial envelope. Emitted derivatives were checked against those bounds.

Browser scope: Chrome loaded the new workspace and its default tangent-heading control, and submitted a real L-turn. This exposed the old coarse heading certificate refusal and motivated the refinement covered by native regression. Final browser reload and visual/download review were blocked by the browser origin approval service timing out; one allowed retry also timed out. No final visual, phone or download pass is claimed for this version. Earlier comparison UI evidence remains historical.

No new Gazebo run, terrain/contact simulation, touchdown detection, disarm or motor-stop evidence is claimed. The public site remains planning only. The ground-to-ground CSV is a reference, not a replacement executable for the native route runner, which already owns those phases.
