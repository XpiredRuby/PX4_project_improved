"""Serve the React offline workbench with the native preview compiler on localhost."""
from __future__ import annotations

import argparse
import csv
from http.server import ThreadingHTTPServer
from io import BytesIO, StringIO
import mimetypes
from urllib.parse import urlsplit

import numpy as np

from .editor import PreviewHandler, ROOT


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
        if path.startswith("/api/"):
            return super().do_GET()
        if path == "/browser-api.js":
            return self._reply(200, b"// Native localhost planner.", "text/javascript")
        dist = ROOT / "studio-v2/dist"
        asset = (dist / (path.lstrip("/") or "index.html")).resolve()
        if not asset.is_relative_to(dist) or not asset.is_file():
            return self._json(404, {"error": "Unknown workspace asset"})
        body = asset.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", mimetypes.guess_type(asset.name)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self' https://api.open-meteo.com https://geocoding-api.open-meteo.com; frame-ancestors 'none'; object-src 'none'")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8767)
    args = parser.parse_args()
    if not (ROOT / "studio-v2/dist/index.html").exists():
        parser.error("Build the frontend in studio-v2 with npm install and npm run build.")
    with ThreadingHTTPServer(("127.0.0.1", args.port), ResearchHandler) as server:
        print(f"Trajectory Studio: http://127.0.0.1:{server.server_port}", flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
