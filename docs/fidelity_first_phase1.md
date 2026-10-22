# Fidelity-first planner baseline — 2026-10-20

## Frozen scope

This increment improves deterministic geometric intent and acceptance. It does not replace the retimer or add a neural network. The public site remains planning only; hosted Gazebo acceptance is Phase 2. Aircraft-model assumptions are shared and are not a physical-flight guarantee.

Baseline: standard x500 SITL model, 3 m/s model ceiling, public preferred/max route speed 1 m/s with 0.9 planning scale, acceleration 1 m/s², jerk 1.5 m/s³, tilt 25°, 40 m operating radius, 30 m maximum height. The audit corpus uses 20 m height, 0.5 m hard geometric allowance and fixed yaw; the browser L and lifecycle regression also exercise tangent yaw. The 200 m research mission length does not imply a 200 m radius. A 5 m/s research cruise remains outside this baseline.

## Selection and fallback

`geometry.build_smooth_path` evaluates at most eleven distinct control spacings, including four refinement levels below the requested spacing. Candidates must satisfy both conservative two-sided proximity and order-aware acceptance. The default soft target is 25% of the hard allowance. Candidate ranking first minimizes excess above that target, then maximizes control spacing as an explicit smoothness proxy among candidates already meeting it. The proxy is not an optimized physical energy or jerk integral. This is bounded candidate-family selection, not global optimality or minimization of deviation to zero.

For 0.5 m allowed rounding, the normal continuous-curve target is 0.125 m. The goal includes the larger of the proximity and order-aware curve bounds. Controller interpolation is an additional hard gate. The summary records the goal, achieved bound, candidate count and explicit fallback. A missed soft target can be accepted only inside the hard limit and is visibly labeled. No tolerance is silently expanded.

The tighter geometry is retimed under the existing speed, acceleration, jerk, yaw and model constraints. Slowing never substitutes for refitting. Existing hard stops remain exact; exact fly-through points remain fly-through. No automatic stops are added. If a closer fit cannot be certified, the interface suggests marking a corner stop or revising the route. Hard-limit or timing failures refuse output with the failed checks. This implements an assisted stop fallback; it does not silently alter the mission with new stops.

A Loop selection explicitly requests the END-to-START closing leg. Open routes stop at END before returning home. Periodic splines now center and anchor the seam at the requested source origin while updating repeated controls together, retaining periodic derivatives. Arbitrary nonzero-velocity initial/final boundary states remain unsupported.

## Acceptance methods

1. `INT-007`: existing continuous two-sided proximity and controller interpolation bound.
2. `INT-008`: monotone discrete Fréchet coupling of polygonal curves, plus a continuous spline approximation bound and controller interpolation reserve. Traversal order matters; repeated visits and swapped loops can no longer pass merely because their spatial sets coincide.
3. `VER-004`: every emitted position is checked with a separate Cox-basis evaluator. `ordered_fidelity.py` imports no planner geometry, timing or dynamics primitives.
4. After serialization, all CSV fields, row count, version and strictly increasing timestamps are re-read and compared with the accepted reference before its hash is released.

The spline-to-polygon error uses the second-derivative control hull: M₂ h²/8 on each parameter interval. Recursive subdivision also bounds polygon edge length. A feasible discrete monotone coupling upper-bounds continuous polygonal Fréchet distance; adding the spline approximation reserve gives the continuous curve bound. Seven decision refinements tighten the reported upper bound without changing acceptance. Hard limits: 4,096 nodes per polygon, four million coupling cells, depth 24, eleven fitting candidates, existing 20,000-control cap, and an additional 512-control cap for dense exact-point systems. Exhaustion causes an explicit refusal or a labeled hard-only fit, never omission of the order gate.

Reference: Eiter and Mannila, *Computing Discrete Fréchet Distance*, CD-TR 94/64, 1994, https://www.kr.tuwien.ac.at/staff/eiter/et-archive/files/cdtr9464.pdf . Their longest-edge approximation result applies to polygonal curves; the spline remainder above is separately included.

The existing verifier remains **separate acceptance logic with shared primitives**. The new geometric method adds numerical diversity. It does not constitute a second independent aircraft-dynamics verifier, formal floating-point proof, or independent organizational V&V. Required stops, endpoint conditions and loop count remain separate contract/structure checks.

## Heading, timing and evidence

Tighter corners can increase the heading certificate's global Lipschitz remainder even when the fitted heading is accurate. The bounded fallback now subdivides the common path/profile spans locally to hold each gap's remainder below 0.25°. The overall heading limit remains 2°. Both planning and acceptance perform the check. The sampler is capped at 32,000 coordinates. No heading or dynamic limit is relaxed.

`scripts/audit_fidelity_baseline.py` records native accept/refuse decisions, computation time, fidelity outcomes, all verification checks, duration and binding constraint. It compares duration with a matched-margin analytic 1-D jerk-limited endpoint-displacement relaxation and the path-length/speed bound. For curved paths this is **not** the optimum of the original constrained problem; the reported ratio is not a proven solver optimality gap. No TOPP/COPP installation, benchmark, native integration or Pyodide support is claimed.

The corpus includes straight, rounded/stopped L, zigzag, figure-eight, lawnmower, repeated visits, many stops, near-boundary and tiny-feature routes, one/two-lap circles, invalid and oversized input. A separate seam matrix exercises entry, exit, two tangent-yaw laps and unsupported seam-stop requests. Results are in `validation/2026-10-20-fidelity-first/corpus.json` and `web/fidelity-validation.md`.

Energy remains a conservative profile budget with reserve and contingency, not an optimized objective. Native and browser results are compared by acceptance and numerical tolerances, not byte-identical CSV assumptions. Remaining research work includes a genuinely diverse dynamics checker, matched native TOPP/COPP comparisons and formal cross-runtime differential measurement. Hosted tracking, touchdown, disarm and motor-stop evidence remain Phase 2.
