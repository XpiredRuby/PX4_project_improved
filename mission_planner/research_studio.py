"""Serve the React offline workbench with the native preview compiler on localhost."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import re
import csv
from http.server import ThreadingHTTPServer
from io import BytesIO, StringIO
import mimetypes
from urllib.parse import urlsplit

import numpy as np

from .editor import MAX_REQUEST_BYTES, PreviewHandler, ROOT
from .study_queue import StudyQueue


def recording_csv(raw: bytes) -> bytes:
    from pyulog import ULog
    if not raw.startswith(b'ULog\x01\x12\x35'):
        raise ValueError("Expected a PX4 ULog file.")
    log = ULog(BytesIO(raw), message_name_filter_list=["vehicle_local_position", "trajectory_setpoint"])
    state = log.get_dataset("vehicle_local_position").data
    reference = next((d.data for d in log.data_list if d.name == "trajectory_setpoint" and d.multi_id == 0), None)
    sample_time = state.get("timestamp_sample", state["timestamp"])
    if not len(sample_time) or not np.all(np.diff(sample_time.astype(float)) > 0):
        raise ValueError("Missing or non-increasing position timestamps.")
    times = reference["timestamp"] if reference else []
    if len(times) and np.any(np.diff(times.astype(float)) < 0):
        raise ValueError("Setpoint timestamps are out of order.")
    out = StringIO()
    writer = csv.writer(out)
    writer.writerow(["elapsed_s", "estimate_x", "estimate_y", "estimate_z", "desired_x", "desired_y", "desired_z", "segment"])
    indices = np.searchsorted(times, sample_time, side="right") - 1
    segment = 0
    segment_start = -1
    for i, (time, j) in enumerate(zip(sample_time, indices, strict=True)):
        if i and any(k in state and state[k][i] != state[k][i - 1]
                     for k in ("xy_reset_counter", "z_reset_counter")):
            segment += 1
            segment_start = int(time)
        pos = [state[k][i] for k in ("x", "y", "z")]
        if any(k in state and not state[k][i] for k in ("xy_valid", "z_valid")):
            pos = [""] * 3
        target = [""] * 3
        if reference and j >= 0 and segment_start <= int(times[j]) and int(time) - int(times[j]) <= 250_000:
            target = [reference[f"position[{axis}]"][j] for axis in range(3)]
        writer.writerow([(int(time) - int(sample_time[0])) / 1e6, *pos, *target, segment])
    return out.getvalue().encode()


class ResearchHandler(PreviewHandler):
    def do_GET(self):
        if not self._local_request():
            return self._json(403, {"error": "local origin required"})
        path = urlsplit(self.path).path
        queue = getattr(self.server, "study_queue", None)
        if path == "/api/capabilities":
            return self._json(200, {"execution_location": "local", "study_queue": queue is not None,
                                    "sitl_enabled": False, "hardware_enabled": False})
        if queue and path == "/api/studies":
            return self._json(200, {"studies": queue.list()})
        match = re.fullmatch(r"/api/studies/([a-f0-9-]{36})(?:/(export|cases/([0-9]+)))?", path)
        if queue and match:
            identifier, action, index = match.groups()
            try:
                if index is not None:
                    return self._reply(200, queue.artifact(identifier, int(index)), "application/json")
                return self._json(200, queue.status(identifier, snapshot=action == "export"))
            except KeyError as exc:
                return self._json(404, {"error": exc.args[0]})
        if path.startswith("/api/"):
            return super().do_GET()
        if path == "/browser-api.js":
            return self._reply(200, b"// Native localhost planner.", "text/javascript")
        if path not in {"/", "/index.html", "/favicon.ico"} and not path.startswith("/assets/"):
            return self._json(404, {"error": "Unknown workspace asset"})
        dist = getattr(self.server, "assets", ROOT / "studio-v2/dist")
        asset = (dist / (path.lstrip("/") or "index.html")).resolve()
        if asset.suffix not in {".html", ".js", ".css", ".csv", ".gz", ".ico", ".svg", ".png", ".woff2"}:
            return self._json(404, {"error": "Unknown workspace asset"})
        if (not asset.is_relative_to(dist / "assets") and asset != dist / "index.html" and asset != dist / "favicon.ico") or not asset.is_file():
            return self._json(404, {"error": "Unknown workspace asset"})
        body = asset.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", mimetypes.guess_type(asset.name)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self' https://api.open-meteo.com https://geocoding-api.open-meteo.com; frame-ancestors 'none'; object-src 'none'")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == "/api/studies" or self.path.startswith("/api/studies/"):
            if not self._local_request():
                return self._json(403, {"error": "local origin required"})
            queue = getattr(self.server, "study_queue", None)
            if queue is None:
                return self._json(409, {"error": "Start the native studio to run a durable study"})
            try:
                size = int(self.headers.get("Content-Length", 0))
                if size > MAX_REQUEST_BYTES:
                    return self._json(413, {"error": "Study submission exceeds the request budget"})
                if size < 0 or self.headers.get_content_type() != "application/json":
                    raise ValueError("Expected a JSON request")
                self.connection.settimeout(10)
                body = self.rfile.read(size)
                if len(body) != size:
                    raise ValueError("Incomplete request")
                payload = json.loads(body)
                if self.path == "/api/studies":
                    return self._json(202, queue.submit(payload))
                match = re.fullmatch(r"/api/studies/([a-f0-9-]{36})/(cancel|retry)", self.path)
                if not match:
                    return self._json(404, {"error": "Unknown study action"})
                return self._json(200, queue.change(*match.groups()))
            except KeyError as exc:
                return self._json(404, {"error": str(exc.args[0])})
            except (ValueError, TypeError, OverflowError) as exc:
                return self._json(400, {"error": str(exc)})
        if self.path != "/api/recording":
            return super().do_POST()
        if not self._local_request():
            return self._json(403, {"error": "local origin required"})
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 64 * 1024**2:
                raise ValueError("Select a ULog up to 64 MiB.")
            self.connection.settimeout(15)
            body = recording_csv(self.rfile.read(length))
        except Exception as exc:
            return self._json(422, {"error": f"Unable to read PX4 local-position recording: {exc}"})
        self._reply(200, body, "text/csv; charset=utf-8")


@contextmanager
def research_server(port, directory, assets=None):
    with ThreadingHTTPServer(("127.0.0.1", port), ResearchHandler) as server:
        server.assets = Path(assets or ROOT / "studio-v2/dist").resolve()
        server.study_queue = StudyQueue(directory)
        try:
            yield server
        finally:
            server.study_queue.close()


def main(default_assets=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--studies-dir", type=Path, default=Path.home() / ".px4-trajectory-studio/studies")
    args = parser.parse_args()
    assets = Path(default_assets or ROOT / "studio-v2/dist")
    if not (assets / "index.html").exists():
        parser.error("Build the frontend in studio-v2 with npm install and npm run build.")
    with research_server(args.port, args.studies_dir, assets) as server:
        print(f"Trajectory Studio: http://127.0.0.1:{server.server_port}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
