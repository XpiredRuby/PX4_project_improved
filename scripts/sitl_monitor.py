"""Keep a SITL mission's parent alive after a test-injection failure."""

import math
import sys
import time


def trial_phase_clock(row):
    if row["phase"] in {"ALIGN", "HANDOFF"}:
        return float(row["phase_elapsed_s"])
    return float(row["phase_clock_s"])


def allow_next_trial(case, result):
    if result.get("scenario_errors"):
        return False
    if case.get("expected") == "quality_rejection":
        return result.get("quality_rejection_confirmed") is True
    return result.get("audit_passed") is True


def quality_rejection_confirmed(manifest, rows):
    reason = str(manifest.get("reason") or "")
    physical_violation = any(text in reason for text in (
        "touchdown horizontal speed exceeded", "touchdown vertical speed exceeded",
        "touchdown tilt exceeded", "touchdown position exceeded", "PX4 left ON_GROUND"))
    if (manifest.get("outcome") != "UNSAFE_TOUCHDOWN"
            or manifest.get("cleanup_status") != "landed_with_quality_violation"
            or manifest.get("cleanup_error") is not None or not physical_violation
            or not rows):
        return False
    try:
        end = float(rows[-1]["elapsed_s"])
        recent = [r for r in rows if float(r["elapsed_s"]) >= end - 1.]
        if len(recent) < 2 or end - float(recent[0]["elapsed_s"]) < .9:
            return False
        for row in recent:
            values = [float(row[f"actuator_output_{i}"]) for i in range(4)]
            if (str(row["armed"]).lower() != "false" or int(row["landed_state"]) != 1
                    or not 0 <= float(row["actuator_age_s"]) <= .25
                    or not 0 <= float(row["extended_state_age_s"]) <= 1.5
                    or not all(math.isfinite(v) and abs(v) <= .01 for v in values)):
                return False
    except (KeyError, TypeError, ValueError):
        return False
    return True


def monitor_child(child, step, on_error, timeout_s=450, interval_s=.15):
    """Stop injections on error; wait for the runner's own landing cleanup.

    A test infrastructure failure must not terminate an airborne runner by
    exiting its WSL parent. The runner owns its flight and cleanup timeouts.
    No later case is permitted without separate fresh ground/disarm evidence.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("Monitor timeout must be finite and positive")
    if not math.isfinite(interval_s) or interval_s <= 0:
        raise ValueError("Monitor interval must be finite and positive")
    deadline = time.monotonic() + timeout_s
    errors = []
    while child.poll() is None:
        if not errors:
            try:
                if time.monotonic() > deadline:
                    raise TimeoutError("Test deadline exceeded; waiting for runner cleanup")
                step()
            except Exception as exc:
                errors.append({"type": type(exc).__name__, "message": str(exc)})
                try:
                    on_error(errors)
                except Exception as record_exc:
                    errors.append({"type": type(record_exc).__name__,
                                   "message": "Error recording failed: " + str(record_exc)})
                print("SITL scenario failed; waiting for mission cleanup:",
                      errors, file=sys.stderr, flush=True)
        time.sleep(interval_s)
    return child.returncode, errors
