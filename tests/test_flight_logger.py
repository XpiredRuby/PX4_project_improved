"""Storage faults must not block control publication or hide evidence loss."""
import csv
import io
from pathlib import Path
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controller"))
from flight_logger import FlightLogger  # noqa: E402


class SlowStream(io.StringIO):
    def __init__(self):
        super().__init__()
        self.blocked = threading.Event()
        self.release = threading.Event()

    def write(self, value):
        self.blocked.set()
        if not self.release.wait(2.):
            raise TimeoutError("test storage timeout")
        return super().write(value)


class FlightLoggerTests(unittest.TestCase):
    def test_unbounded_queue_and_invalid_shutdown_deadline_are_rejected(self):
        for capacity in (0, -1, True, 1.5):
            with self.subTest(capacity=capacity), self.assertRaises(ValueError):
                FlightLogger("unused", ["x"], capacity=capacity)
        for timeout in (0, -1, float("nan"), float("inf")):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                FlightLogger("unused", ["x"], close_timeout_s=timeout)

    def test_slow_storage_does_not_block_sample_or_flush_publication(self):
        stream = SlowStream()
        log = FlightLogger("unused", ["x"], stream=stream)
        self.assertTrue(stream.blocked.wait(1.))
        completed = threading.Event()

        def publish():
            log.writerow({"x": 1.})
            log.flush()
            completed.set()

        publisher = threading.Thread(target=publish, daemon=True)
        publisher.start()
        try:
            self.assertTrue(completed.wait(.5), "control publication waited for storage")
        finally:
            stream.release.set()
            publisher.join(1.)
            log.close()

    def test_capacity_failure_is_visible_without_overwriting_queued_evidence(self):
        stream = SlowStream()
        log = FlightLogger("unused", ["x"], capacity=2, stream=stream)
        self.assertTrue(stream.blocked.wait(1.))
        try:
            log.writerow({"x": 1})
            log.writerow({"x": 2})
            with self.assertRaisesRegex(RuntimeError, "queue exhausted"):
                log.writerow({"x": 3})
        finally:
            stream.release.set()
            log.close()

    def test_close_drains_ordered_immutable_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "log.csv"
            log = FlightLogger(path, ["x"])
            row = {"x": 1}
            log.writerow(row)
            row["x"] = 2
            log.writerow(row)
            log.flush()
            log.close()
            with path.open() as source:
                self.assertEqual(list(csv.DictReader(source)), [{"x": "1"}, {"x": "2"}])
            self.assertTrue(log.closed)
            with self.assertRaisesRegex(RuntimeError, "closing"):
                log.writerow(row)

    def test_write_failure_is_reported_to_caller(self):
        class BrokenStream(io.StringIO):
            def write(self, value):
                raise OSError("disk full")

        log = FlightLogger("unused", ["x"], stream=BrokenStream())
        with self.assertRaisesRegex(OSError, "disk full"):
            log.close()
        with self.assertRaisesRegex(OSError, "disk full"):
            log.writerow({"x": 1})

    def test_close_has_bounded_deadline_for_stuck_storage(self):
        stream = SlowStream()
        log = FlightLogger("unused", ["x"], close_timeout_s=.01, stream=stream)
        self.assertTrue(stream.blocked.wait(1.))
        try:
            with self.assertRaises(TimeoutError):
                log.close()
        finally:
            stream.release.set()
            self.assertTrue(log._closed.wait(1.))
            log.close()

    def test_repeated_flush_requests_are_coalesced(self):
        stream = SlowStream()
        log = FlightLogger("unused", ["x"], capacity=2, stream=stream)
        self.assertTrue(stream.blocked.wait(1.))
        try:
            for _ in range(100):
                log.flush()
            log.writerow({"x": 1})
        finally:
            stream.release.set()
            log.close()
