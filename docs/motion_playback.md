# Full-resolution mission playback

The website now animates the complete planned mission from `mission.trajectory_csv`, including takeoff, route, stationary turns, return and descent. It never uses the decimated display points as the playback clock or source of motion. The blue route display also uses the full route samples when playback is available.

## Scale and time

Default playback is 1x: one mission second per wall-clock second. Other rates only affect the viewing clock. The north/east map uses one common metres-to-pixels scale with an explicit scale bar. The isometric view uses equal-length projections of the three world axes with no vertical exaggeration. The drone glyph is enlarged and labeled as a symbol; its size is not an airframe measurement.

Playback includes play/pause, a time scrubber, exact previous/next CSV sample controls, adjustable rate, current position, positive height, speed, heading and yaw rate. A recent motion trail and grouped event numbers help explain crossing and repeated visits. The event selector retains every timestamp even when only a subset of numbers fits on the map. Stops and turns do not move the marker through space. The return leg is shown separately. A second view plots height against time.

Exact sample selection returns original exported values. Between samples, a seventh-order Hermite interpolant uses position, velocity, acceleration and jerk (and the corresponding heading derivatives), with linear interpolation when imported measurements lack those derivatives. This exactly reconstructs the implemented seventh-order phase family up to floating-point error. It does not establish a general continuous-time tracking error bound for other planner families. Planned roll and pitch are ideal no-drag references inferred from the commanded acceleration and heading. No measured attitude or contact physics is fabricated.

## Measured comparison

An optional CSV import stays on the device and uses a separate purple track and marker. It requires `time,x,y,z,yaw` in LOCAL_NED metres, seconds and radians. Heading wrapping is unwrapped along the short turn. The user supplies matching origins and a clock offset (`measurement time = mission time + offset`). Measurements are displayed only during overlapping times, without extrapolation. The instantaneous three-dimensional separation is shown. Source and clock alignment are user supplied; import does not prove that a track came from Gazebo or a physical aircraft and does not start a simulator.

Editing the drawing clears playback, cancels animation and discards the previous measurement. In-flight file reads cannot attach stale measurements to a new plan. Hidden tabs pause playback rather than silently jumping ahead.

## Validation

- All 39 JavaScript tests passed, including ten motion tests for CSV integrity and clocks, exact knots, polynomial reconstruction, stationary yaw turns, ideal attitude, repeated-event timing, 1x clock behavior, initial turn precedence, sparse measurements and wrapped headings.
- A separate evaluation of all 22 route phases in the previously downloaded real-browser Vin mission was compared against playback interpolation. The 10,262-row CSV produced maximum position and yaw differences below 1e-12 in the selected interior checks. These are numerical reconstruction checks, not physical flight errors; details are in `validation/motion/interpolation-check.json`.
- The portable static build passed with the added motion module. Actual Chrome/Pyodide browser checks passed for the Vin mission: 1x play/pause with stationary position and changing yaw, exact sample stepping, event jumps, final ground state, height and 3D views, stale-result invalidation, and undo. A synthetic 0.1 m shifted track produced a 0.100 m comparison reading; this was an import test, not a Gazebo run. Metrics and screenshots are in `validation/motion/`.

## Meaning of accuracy

This is an accurate representation of the exported reference states at their timestamps. It is not a prediction that a physical drone will occupy those exact states. A matched PX4/Gazebo or aircraft recording is required to establish actual tracking, touchdown and disarm behavior. The current hosted page does not run Gazebo.
