# Drawing and planned-route comparison

The accepted plan keeps the original input as a wide translucent amber shadow with a dashed centerline, under a blue planned route. Layer controls isolate original input or planned route without changing either. Rendering widths scale with the canvas CSS size so they remain legible on phone layouts.

A contextual close-up shows the greatest nearest-input-segment distance among the displayed planned samples. It is a top-view, sampled visual estimate. It is not a flight tracking measurement, continuous error proof, ordered-path proof, or a new verifier. The independent INT-007 certificate supplies the conservative controller/curve upper bound from declared tolerance minus the worst verification margin. Unknown, failed or invalid certificate margins do not produce a bound. Display sampling may be reduced, as the close-up explanation states.

The same route coordinates and smoothing remain in use. Layer changes trigger redraw only. Any edit clears comparison and resets the layer mode, preventing a stale overlay from appearing to match a changed drawing. PNG export includes a color legend, active layer view and planning-only scope.

Automated tests cover nearest segment distance, repeated vertices and overlapping routes, absent/failed/invalid certificates, and isolated high-contrast canvas rendering at phone scale. Worker authority and timeout tests also pass.

Browser QA: a real rounded L plan displayed a 23.3 cm sampled gap, a 41.4 cm conservative upper bound and a 50 cm allowance. Overlay / Drawing only / Plan only selected the expected layers, PNG download completed, and Undo immediately hid old plan and comparison. The accepted plan automatically fits the original and planned paths together; desktop review returns to the map instead of scrolling the comparison out of sight. The indicative simulator return segment is excluded from this planning comparison. Native editor/compiler/HTTP tests (11) and JavaScript drawing/worker/comparison tests (19) passed.

Final phone-width Chrome QA (390 px iframe): the real rounded-L result and the same numeric bounds rendered successfully, comparison controls and fitted route remained visible, and no horizontal overflow was detected. The close-up was brought into view after planning; the planning action remained reachable. This is browser viewport QA, not physical phone hardware testing. Temporary QA HTML was removed before packaging.
