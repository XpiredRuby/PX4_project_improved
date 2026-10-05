"""Bounded Gazebo wind publication with observed seed confirmation."""
import math
import re
import subprocess
import time


def gz_service(world, action, request_type, response_type, request, timeout_s=5.):
    result = subprocess.run([
        "gz", "service", "-s", f"/world/{world}/{action}",
        "--reqtype", request_type, "--reptype", response_type,
        "--timeout", str(max(1, int(timeout_s * 1000))), "--req", request,
    ], check=True, capture_output=True, text=True, timeout=timeout_s + .1)
    if response_type == "gz.msgs.Boolean" and "data: true" not in result.stdout:
        raise RuntimeError(f"Gazebo rejected {action}: {result.stdout}")
    return result.stdout


def wind_seed_matches(response, velocity):
    # WindEffects replies with the stored command, including message presence
    # and enable_wind. Missing proto3 scalar axes legitimately represent zero;
    # an empty/no-service response cannot confirm any seed, including zero.
    if "linear_velocity" not in response or "enable_wind: true" not in response:
        return False
    values = {axis: float(value) for axis, value in
              re.findall(r"\b([xyz]):\s*([-+0-9.eE]+)", response)}
    return all(math.isclose(values.get(axis, 0.), value, abs_tol=1e-5)
               for axis, value in zip("xyz", velocity, strict=True))


def set_wind(world, velocity, timeout_s=5.):
    if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
        raise ValueError("Wind seed must have three finite components")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("Wind confirmation timeout must be finite and positive")
    x, y, z = velocity
    request = f"linear_velocity: {{x: {x} y: {y} z: {z}}} enable_wind: true"
    deadline = time.monotonic() + timeout_s
    publish_at = -math.inf
    last = ""
    while (remaining := deadline - time.monotonic()) > 0:
        try:
            if time.monotonic() >= publish_at:
                subprocess.run(["gz", "topic", "-t", f"/world/{world}/wind",
                                "-m", "gz.msgs.Wind", "-p", request],
                               check=True, capture_output=True, text=True,
                               timeout=min(2., remaining))
                publish_at = time.monotonic() + .5
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            last = gz_service(world, "wind_info", "gz.msgs.Empty", "gz.msgs.Wind", "",
                              timeout_s=min(1., max(.001, remaining - .1)))
            if time.monotonic() <= deadline and wind_seed_matches(last, velocity):
                return last
        except (subprocess.SubprocessError, RuntimeError) as exc:
            last = str(exc)
        time.sleep(min(.05, max(0., deadline - time.monotonic())))
    raise RuntimeError(f"Wind seed not confirmed within {timeout_s}s: {last}")
