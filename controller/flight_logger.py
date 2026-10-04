"""Bounded, asynchronous flight evidence writing outside the control thread."""
import csv
import math
import queue
import threading


class FlightLogger:
    def __init__(self, filename, fields, capacity=512, close_timeout_s=2., stream=None):
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("Flight evidence capacity must be a positive integer")
        if not math.isfinite(close_timeout_s) or close_timeout_s <= 0:
            raise ValueError("Flight evidence shutdown deadline must be finite and positive")
        self.filename = filename
        self.fields = tuple(fields)
        self.close_timeout_s = close_timeout_s
        self._stream = stream if stream is not None else open(filename, "w", newline="")
        self._queue = queue.Queue(maxsize=capacity)
        self._closing = threading.Event()
        self._closed = threading.Event()
        self._lock = threading.Lock()
        self._error = None
        self._flush_pending = False
        self._thread = threading.Thread(target=self._run, name="flight-evidence-writer", daemon=True)
        self._thread.start()

    @property
    def closed(self):
        return self._closed.is_set()

    def _check(self):
        if self._error is not None:
            raise OSError(f"Flight evidence writing failed: {self._error!r}") from self._error
        if self._closing.is_set():
            raise RuntimeError("Flight evidence writer is closing")

    def _put(self, kind, value=None):
        with self._lock:
            self._check()
            if kind == "flush" and self._flush_pending:
                return
            try:
                self._queue.put_nowait((kind, value))
            except queue.Full as exc:
                raise RuntimeError("Flight evidence queue exhausted; storage is not keeping up") from exc
            if kind == "flush":
                self._flush_pending = True

    def writerow(self, row):
        # Freeze each sample before the control worker reuses its dictionaries.
        self._put("row", dict(row))

    def flush(self):
        self._put("flush")

    def status(self, text):
        self._put("status", str(text))

    def _run(self):
        try:
            writer = csv.DictWriter(self._stream, fieldnames=self.fields, extrasaction="ignore")
            writer.writeheader()
            self._stream.flush()
            while not self._closing.is_set() or not self._queue.empty():
                try:
                    kind, value = self._queue.get(timeout=.05)
                except queue.Empty:
                    continue
                try:
                    if kind == "row":
                        writer.writerow(value)
                    elif kind == "status":
                        print(value)
                    elif kind == "flush":
                        self._stream.flush()
                        with self._lock:
                            self._flush_pending = False
                finally:
                    self._queue.task_done()
            self._stream.flush()
        except Exception as exc:
            self._error = exc
        finally:
            try:
                self._stream.close()
            except Exception as exc:
                if self._error is None:
                    self._error = exc
            self._closed.set()

    def close(self):
        with self._lock:
            self._closing.set()
        if not self._closed.wait(self.close_timeout_s):
            raise TimeoutError("Flight evidence writer did not finish within its shutdown deadline")
        if self._error is not None:
            raise OSError(f"Flight evidence writing failed: {self._error!r}") from self._error
