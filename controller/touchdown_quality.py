"""Runtime touchdown verdict after native PX4 landing cleanup."""
import math


class UnsafeTouchdown(RuntimeError):
    """Contact completed, but measured touchdown quality was not acceptable."""


def touchdown_violations(snapshot, reference_xy=None):
    keys = ("vx", "vy", "vz", "roll", "pitch", "position_age_s", "attitude_age_s")
    if not all(math.isfinite(snapshot.get(key, math.nan)) for key in keys):
        return ["touchdown telemetry unavailable or non-finite"]
    violations = []
    if snapshot["position_age_s"] > .25 or snapshot["attitude_age_s"] > .25:
        violations.append("touchdown motion/attitude telemetry stale")
    if math.hypot(snapshot["vx"], snapshot["vy"]) > .5:
        violations.append("touchdown horizontal speed exceeded 0.5 m/s")
    if abs(snapshot["vz"]) > .5:
        violations.append("touchdown vertical speed exceeded 0.5 m/s")
    # Body z against world vertical includes simultaneous roll and pitch.
    tilt = math.acos(max(-1., min(1.,
        math.cos(snapshot["roll"]) * math.cos(snapshot["pitch"]))))
    if tilt > math.radians(10) + 1e-12:
        violations.append("touchdown tilt exceeded 10 degrees")
    if reference_xy is not None:
        position = (snapshot.get("x", math.nan), snapshot.get("y", math.nan))
        if (len(reference_xy) != 2
                or not all(math.isfinite(v) for v in (*position, *reference_xy))):
            violations.append("touchdown horizontal position/reference unavailable")
        elif math.hypot(position[0] - reference_xy[0],
                        position[1] - reference_xy[1]) > 1.5:
            violations.append("touchdown position exceeded 1.5 m")
    return violations
