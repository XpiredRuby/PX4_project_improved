# Fidelity-first validation — 2026-10-20

## Current checks

- Geometry/order/lifecycle suites: 23 tests passed, including default-versus-legacy fidelity, explicit soft-target misses, launch-anchored periodic continuity, swapped-loop/wrong-order refusal, dynamic-programming equivalence and actual tangent-heading L compilation.
- Separate-basis/export suite: 7 tests passed, including proof that the geometric audit does not call the planner evaluator and detection of a modified serialized field. Two of these are new beyond the 23-test run.
- Final native verifier/editor/aircraft integration: 36 tests passed, including real subprocess compilation, native HTTP boundaries, four aircraft fixtures, artifact/profile hashes and energy refusal.
- Timing/retiming/input suites: 28 tests passed.
- Circle seam matrix: 4 tests passed: circle entry in motion, circle exit in motion, two tangent-yaw laps, and refusal of unsupported seam-stop/pattern requests. The two-lap test additionally asserts the origin is exact and the interior seam speed is nonzero.
- JavaScript drawing/worker/comparison suites: 19 tests passed.
- Changed-file Ruff, Python compilation, generated inline JavaScript syntax, external JavaScript syntax and the portable build are checked before publication.

## Corpus and timing

All 14 outcomes in `validation/2026-10-20-fidelity-first/corpus.json` matched expectations: 12 accepted valid routes and two refused invalid/oversized requests. Cases include straight, rounded/stopped L, zigzag, figure-eight, lawnmower, repeated visits, many stops, near-boundary, tiny feature and one/two-lap circles. Full per-case checks, fitting outcomes, computation times and timing relaxation comparisons are retained.

With the fixed-yaw audit baseline, the rounded L has a 0.09250 m conservative curve bound against a 0.125 m soft target, takes 44.094 s and is jerk limited. The stopped L takes 36.550 s and is also jerk limited. The straight 20 m route takes 25.168 s against a matched-margin 1-D reference of 23.889 s, approximately 5.35% above it. Curved-route ratios are against relaxed constraints, not proven gaps to an optimal matched-path solver.

The first corpus run exposed a repeated-visit failure: the old spatial metric chose a curve that covered the same line set while missing the intended reversal. Order now participates in fitting selection as well as final acceptance. The current repeated-visit route passes with a 0.09845 m curve bound and 48.006 s route duration.

## Browser evidence

Chrome on the managed preview compiled the actual public L preset using tangent yaw. Every v2 acceptance check passed. The downloaded plan report records the 0.09250 m curve bound, 0.125 m goal, eleven fitting candidates and a successful 1,035-row serialized route audit. Public tangent-yaw route duration is 51.7 s; fixed-yaw timing above is a separate baseline.

The full mission CSV download contains 6,677 rows. Its SHA-256 matches the report, and its first/last position and positive height are zero, with TAKEOFF and LAND phase labels respectively. The full mission is a reference; it is not sent to a real drone or used to duplicate the native runner lifecycle.

Changing height after verification cleared the prior plan, comparison and downloads, returning the workspace to Draft. The UI distinguishes the continuous-curve goal from the route bound that includes controller interpolation. Original drawing shadow, layer controls and full mission height chart are retained. Screenshots and the browser report are saved with this validation set. No physical phone validation or complete browser/native differential corpus is claimed.

## Limits

The second method is geometrically diverse, not a second aircraft-dynamics implementation or organizationally independent V&V. Energy is budgeted, not optimized. Stops are only added when requested by the user; hard-only fallback is disclosed, and a tighter fit may require an explicit corner stop. TOPP/COPP integration and benchmarking remain future research. No hosted Gazebo, physical touchdown, disarm or motor-stop evidence is claimed for this increment.
