"""Short bounded extrapolation for feedback only; never used to approve transitions."""
import math


def predict_position(snapshot, max_age_s, max_displacement_m):
    position = tuple(snapshot[axis] for axis in "xyz")
    velocity = tuple(snapshot["v" + axis] for axis in "xyz")
    age = snapshot["position_age_s"]
    if not all(math.isfinite(v) for v in (*position, *velocity, age)) or age < 0:
        return position, 0.0
    # Do not disguise stale telemetry as a current measurement.
    if age > max_age_s:
        return position, 0.0
    displacement = tuple(v * age for v in velocity)
    length = math.sqrt(sum(d * d for d in displacement))
    scale = min(1.0, max_displacement_m / length) if length else 1.0
    return tuple(p + d * scale for p, d in zip(position, displacement, strict=True)), age
