"""Explicit software faults used only by the confirmed local SITL harness."""
import json
import math
from pathlib import Path
import time


def install_storage_stall(logger_class, delay_s, after_rows, evidence_path):
    """Block only the evidence thread's underlying stream for a bounded time."""
    if (not math.isfinite(delay_s) or not 0 < delay_s <= 5
            or isinstance(after_rows, bool) or not isinstance(after_rows, int) or after_rows < 2):
        raise ValueError("SITL storage-stall duration/trigger is invalid")
    original = logger_class._run
    path = Path(evidence_path)

    class SlowStream:
        def __init__(self, stream):
            self.stream = stream
            self.rows = 0

        def write(self, value):
            self.rows += 1
            if self.rows == after_rows:
                record = {"started": True, "completed": False, "delay_s": delay_s,
                          "after_rows": after_rows, "started_monotonic": time.monotonic()}
                path.write_text(json.dumps(record, indent=2) + "\n")
                time.sleep(delay_s)
                record.update(completed=True, ended_monotonic=time.monotonic())
                path.write_text(json.dumps(record, indent=2) + "\n")
            return self.stream.write(value)

        def flush(self):
            return self.stream.flush()

        def close(self):
            return self.stream.close()

    def run(self):
        self._stream = SlowStream(self._stream)
        original(self)

    logger_class._run = run


def install_control_stall(controller_class, delay_s, phase_clock_s, evidence_path):
    """Pause the control worker after publication; telemetry/watchdog stay live."""
    if (not math.isfinite(delay_s) or not 0 < delay_s <= 5
            or not math.isfinite(phase_clock_s) or phase_clock_s < 0):
        raise ValueError("SITL control-stall duration/trigger is invalid")
    original = controller_class._build_log_row
    injected = False
    path = Path(evidence_path)

    def build_row(self, **kwargs):
        nonlocal injected
        if not injected and str(self.phase) == "TRAJECTORY" and self.phase_clock_s >= phase_clock_s:
            injected = True
            record = {"started": True, "completed": False, "delay_s": delay_s,
                      "phase": str(self.phase), "phase_clock_s": self.phase_clock_s,
                      "started_monotonic": time.monotonic()}
            path.write_text(json.dumps(record, indent=2) + "\n")
            time.sleep(delay_s)
            record.update(completed=True, ended_monotonic=time.monotonic())
            path.write_text(json.dumps(record, indent=2) + "\n")
        return original(self, **kwargs)

    controller_class._build_log_row = build_row
