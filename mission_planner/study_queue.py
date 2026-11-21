"""Durable, single-worker queue for offline trajectory studies."""
from __future__ import annotations

from contextlib import closing, contextmanager
from copy import deepcopy
import ast
import csv
import hashlib
from io import StringIO
import json
import math
from pathlib import Path
import platform
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
from uuid import UUID
import zlib

import numpy as np

from .editor import ROOT, WORKER_TIMEOUT_S
from .preview import MAX_RESPONSE_BYTES, validate_preview_request


def encode(value):
    return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":"))


def engine_identity():
    digest = hashlib.sha256()
    # The portable compiler manifest and the native source tree contain the
    # same preview dependency closure; execution-only modules are immaterial.
    from . import preview
    pending = ["preview"]
    seen = set()
    sources = list((ROOT / "config/planner").rglob("*.json"))
    folder = Path(preview.__file__).parent
    while pending:
        name = pending.pop()
        if name in seen:
            continue
        seen.add(name)
        path = folder / f"{name}.py"
        sources.append(path)
        for node in ast.walk(ast.parse(path.read_bytes())):
            if isinstance(node, ast.ImportFrom) and node.level == 1 and node.module:
                pending.append(node.module)
    for path in sorted(sources):
        digest.update(path.relative_to(ROOT).as_posix().encode() + b"\0" + path.read_bytes() + b"\0")
    return {"backend": "native-python", "python": platform.python_version(), "numpy": np.__version__,
            "platform": platform.system(), "architecture": platform.machine(), "compiler_sha256": digest.hexdigest()}


def validate_study(payload):
    if not isinstance(payload, dict) or set(payload) != {"id", "project", "request", "speeds"}:
        raise ValueError("A study needs an ID, frozen project, compiler request and speed list")
    identifier = payload["id"]
    if not isinstance(identifier, str) or str(UUID(identifier)) != identifier:
        raise ValueError("Invalid study ID")
    speeds = payload["speeds"]
    if not isinstance(speeds, list) or not speeds or any(type(v) not in (int, float)
            or not math.isfinite(v) or v <= 0 for v in speeds):
        raise ValueError("Enter finite positive requested speeds")
    project, request = payload["project"], payload["request"]
    if not isinstance(project, dict) or project.get("schema") != 1 or not isinstance(project.get("title"), str):
        raise ValueError("Missing editable project")
    if not isinstance(request, dict) or request.get("source") != "drawing" or request.get("planning_mode") != "closest":
        raise ValueError("Use a Studio drawing request")
    validate_preview_request(request)
    document = request["document"]
    path, motion = document["path"], document["motion"]
    points = project.get("points")
    if not isinstance(points, list) or len(points) != len(path["points"]) or len(points) < 2:
        raise ValueError("Frozen project and compiler request must describe the same route")
    for i, (p, planned) in enumerate(zip(points, path["points"], strict=True)):
        if not isinstance(p, dict) or type(p.get("altitude")) not in (float, int):
            raise ValueError("Missing project waypoint")
        hard = i in (0, len(points)-1) or bool(p.get("hard"))
        expected = {"point_id": p.get("id") or f"point-{i}", "north_m": p.get("north"), "east_m": p.get("east"),
                    "down_m": -p["altitude"], "hard": hard, "exact": not hard and bool(p.get("exact"))}
        if planned != expected:
            raise ValueError("Frozen waypoints differ from the compiler request")
    settings = (motion.get("target_speed_m_s"), motion["maximum_speed_m_s"], motion.get("fixed_yaw_deg"),
                path.get("maximum_deviation_m"), request.get("aircraft_profile"))
    aircraft = project.get("aircraft") or {}
    if not isinstance(aircraft, dict) or settings != (project.get("cruise"), project.get("speedCap"),
            project.get("yaw"), project.get("tolerance"), aircraft.get("profile")):
        raise ValueError("Frozen aircraft or motion settings differ from the compiler request")
    env = project.get("environment")
    if not isinstance(env, dict) or env.get("mode") not in {"manual", "live"}:
        raise ValueError("Missing weather snapshot")
    if any(type(env.get(k)) not in (float, int) or not math.isfinite(env[k]) for k in ("windSpeed", "windDirection")) or env["windSpeed"] < 0:
        raise ValueError("Missing finite wind inputs")
    return identifier


def case_artifact(preview, request, project, engine):
    if not preview.get("accepted"):
        raise ValueError(preview.get("error") or "Planner rejected the case")
    route = preview["trajectory_csv"]
    actual = hashlib.sha256(route.encode()).hexdigest()
    summary = preview["summary"]
    if summary.get("controller_trajectory_sha256") != actual:
        raise ValueError("Compiler result and exact route CSV have different digests")
    speed = summary["speed_preferences"]
    metrics = {"requested_m_s": request["document"]["motion"]["target_speed_m_s"],
               "peak_m_s": speed["planned_route_peak_sampled_m_s"], "route_s": speed["route_duration_s"],
               "distance_m": speed["route_distance_sampled_m"], "limits": speed["reasons"],
               "method": preview.get("planning_adjustments", {}).get("method"), "route_sha256": actual}
    weather = project.get("environment") or {}
    metrics["peak_airspeed_m_s"] = None
    if weather.get("mode") == "manual" or (weather.get("mode") == "live" and weather.get("fetchedAt")):
        wind, direction = weather["windSpeed"], math.radians(weather["windDirection"])
        north, east = -wind * math.cos(direction), -wind * math.sin(direction)
        metrics["peak_airspeed_m_s"] = max(math.hypot(float(r["vx"])-north, float(r["vy"])-east, float(r["vz"]))
                                            for r in csv.DictReader(StringIO(route)))
    artifact = {"schema": "trajectory-studio.case.v1", "scope": "offline reference", "engine": engine,
                "project": dict(project, cruise=metrics["requested_m_s"]), "request": request,
                "metrics": metrics, "result": preview}
    return metrics, zlib.compress(encode(artifact).encode())


class StudyQueue:
    def __init__(self, directory, *, timeout=WORKER_TIMEOUT_S, compiler=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.database = self.directory / "studies.sqlite3"
        self.engine = engine_identity()
        self.timeout = timeout
        self.compiler = compiler or self._compile
        self._stop = threading.Event()
        self._wake = threading.Event()
        # SQLite releases this independent ownership lock on process death.
        # A second server must not requeue work still owned by the first one.
        self._owner = sqlite3.connect(self.directory / "worker.lock.sqlite3", timeout=0)
        try:
            self._owner.execute("BEGIN IMMEDIATE")
        except sqlite3.OperationalError as exc:
            self._owner.close()
            raise ValueError("Another studio already owns this study directory") from exc
        try:
            with self._connect() as db:
                db.execute("PRAGMA journal_mode=WAL")
                db.executescript("""
                    CREATE TABLE IF NOT EXISTS studies(
                        id TEXT PRIMARY KEY, created REAL NOT NULL, fingerprint TEXT NOT NULL,
                        title TEXT NOT NULL, snapshot TEXT NOT NULL, engine TEXT NOT NULL
                    );
                    CREATE TABLE IF NOT EXISTS cases(
                        study_id TEXT NOT NULL REFERENCES studies(id), number INTEGER NOT NULL,
                        speed REAL NOT NULL, state TEXT NOT NULL, attempts INTEGER NOT NULL DEFAULT 0,
                        started REAL, finished REAL, error TEXT, metrics TEXT, artifact BLOB,
                        PRIMARY KEY(study_id, number)
                    );
                    CREATE INDEX IF NOT EXISTS ready_cases ON cases(state,study_id,number);
                """)
                db.execute("UPDATE cases SET state='queued' WHERE state='running'")
            self._thread = threading.Thread(target=self._work, name="offline-study-compiler", daemon=True)
            self._thread.start()
        except BaseException:
            self._owner.close()
            raise

    @contextmanager
    def _connect(self):
        with closing(sqlite3.connect(self.database, timeout=10)) as db:
            db.row_factory = sqlite3.Row
            with db:
                yield db

    def close(self):
        self._stop.set()
        self._wake.set()
        self._thread.join()
        self._owner.close()

    def submit(self, payload):
        identifier = validate_study(payload)
        body = encode({k: payload[k] for k in ("project", "request", "speeds")})
        fingerprint = hashlib.sha256(body.encode()).hexdigest()
        with self._connect() as db:
            db.execute("BEGIN IMMEDIATE")
            found = db.execute("SELECT fingerprint FROM studies WHERE id=?", (identifier,)).fetchone()
            if found:
                if found["fingerprint"] != fingerprint:
                    raise ValueError("This submission ID already belongs to a different scenario")
            else:
                db.execute("INSERT INTO studies VALUES (?,?,?,?,?,?)",
                           (identifier, time.time(), fingerprint, payload["project"]["title"], body, encode(self.engine)))
                db.executemany("INSERT INTO cases(study_id,number,speed,state) VALUES (?,?,?,'queued')",
                               ((identifier, i, speed) for i, speed in enumerate(payload["speeds"])))
        self._wake.set()
        return self.status(identifier)

    def list(self):
        with self._connect() as db:
            return [dict(row) for row in db.execute("""
                SELECT s.id,s.title,s.created,COUNT(*) AS total,
                SUM(c.state IN ('verified','failed','timed_out','cancelled')) AS finished
                FROM studies s JOIN cases c ON c.study_id=s.id GROUP BY s.id ORDER BY s.created DESC,s.id""")]

    def status(self, identifier, *, snapshot=False):
        with self._connect() as db:
            db.execute("BEGIN")
            record = db.execute("SELECT id,title,created,engine,fingerprint FROM studies WHERE id=?", (identifier,)).fetchone()
            if not record:
                raise KeyError("Unknown study")
            result = dict(record)
            result["engine"] = json.loads(result["engine"])
            result["cases"] = []
            for row in db.execute("""SELECT number,speed,state,attempts,started,finished,error,metrics
                                     FROM cases WHERE study_id=? ORDER BY number""", (identifier,)):
                case = dict(row)
                case["metrics"] = json.loads(case["metrics"]) if case["metrics"] else None
                result["cases"].append(case)
            if snapshot:
                result["snapshot"] = json.loads(db.execute("SELECT snapshot FROM studies WHERE id=?", (identifier,)).fetchone()[0])
        return result

    def change(self, identifier, action):
        with self._connect() as db:
            db.execute("BEGIN IMMEDIATE")
            row = db.execute("SELECT engine FROM studies WHERE id=?", (identifier,)).fetchone()
            if not row:
                raise KeyError("Unknown study")
            if action == "cancel":
                db.execute("""UPDATE cases SET state='cancelled',finished=?,error='Cancelled by researcher'
                              WHERE study_id=? AND state IN ('running','queued')""", (time.time(), identifier))
            elif action == "retry":
                if json.loads(row["engine"]) != self.engine:
                    raise ValueError("Compiler version changed. Restore the project and submit a new study.")
                db.execute("""UPDATE cases SET state='queued',started=NULL,finished=NULL,error=NULL
                              WHERE study_id=? AND state IN ('failed','cancelled','timed_out')""", (identifier,))
            else:
                raise ValueError("Unknown study action")
        self._wake.set()
        return self.status(identifier)

    def artifact(self, identifier, index):
        with self._connect() as db:
            row = db.execute("SELECT artifact FROM cases WHERE study_id=? AND number=?", (identifier, index)).fetchone()
        if not row or not row[0]:
            raise KeyError("This case has no completed artifact")
        return zlib.decompress(row[0])

    def _claim(self):
        with self._connect() as db:
            db.execute("BEGIN IMMEDIATE")
            row = db.execute("""SELECT s.id,s.snapshot,s.engine,c.number,c.speed,c.attempts FROM studies s
                         JOIN cases c ON c.study_id=s.id WHERE c.state='queued'
                         ORDER BY s.created,s.id,c.number LIMIT 1""").fetchone()
            if row:
                db.execute("""UPDATE cases SET state='running',started=?,attempts=attempts+1
                              WHERE study_id=? AND number=?""", (time.time(), row["id"], row["number"]))
            return row

    def _work(self):
        while not self._stop.is_set():
            self._wake.clear()
            row = self._claim()
            if row is None:
                self._wake.wait(1)
                continue
            key = (row["id"], row["number"], row["attempts"] + 1)

            def active(key=key):
                if self._stop.is_set():
                    return False
                with self._connect() as db:
                    return db.execute("""SELECT 1 FROM cases WHERE study_id=? AND number=?
                                         AND attempts=? AND state='running'""", key).fetchone() is not None

            artifact = metrics = error = None
            try:
                if json.loads(row["engine"]) != self.engine:
                    raise ValueError("Compiler version changed since submission. Submit a new study.")
                snapshot = json.loads(row["snapshot"])
                request = deepcopy(snapshot["request"])
                request["document"]["motion"]["target_speed_m_s"] = row["speed"]
                result = self.compiler(request, active)
                metrics, artifact = case_artifact(result, request, snapshot["project"], self.engine)
                state = "verified"
            except InterruptedError:
                state = "queued" if self._stop.is_set() else "cancelled"
            except TimeoutError as exc:
                state, error = "timed_out", str(exc)
            except Exception as exc:
                state, error = "failed", str(exc) or type(exc).__name__
            with self._connect() as db:
                db.execute("""UPDATE cases SET state=?,finished=?,error=?,metrics=?,artifact=?
                              WHERE study_id=? AND number=? AND attempts=? AND state='running'""",
                           (state, None if state == "queued" else time.time(), error,
                            encode(metrics) if metrics is not None else None, artifact, *key))

    def _compile(self, request, active):
        with tempfile.TemporaryDirectory(prefix="trajectory-study-") as temp:
            path = Path(temp) / "request.json"
            path.write_text(encode(request), encoding="utf-8")
            with tempfile.TemporaryFile() as output, tempfile.TemporaryFile() as errors:
                proc = subprocess.Popen([sys.executable, "-m", "mission_planner.editor", "--compile-request", str(path)],
                                        cwd=ROOT, stdin=subprocess.DEVNULL, stdout=output, stderr=errors)
                deadline = time.monotonic() + self.timeout
                try:
                    while proc.poll() is None:
                        if not active():
                            raise InterruptedError()
                        if time.monotonic() >= deadline:
                            raise TimeoutError(f"Compiler exceeded the {self.timeout:g} s case deadline")
                        self._stop.wait(.15)
                    if proc.returncode:
                        errors.seek(0)
                        raise ValueError("Compiler failed: " + errors.read(1000).decode(errors="replace"))
                    output.seek(0)
                    result = output.read(MAX_RESPONSE_BYTES + 1)
                    if len(result) > MAX_RESPONSE_BYTES:
                        raise ValueError("Compiler response exceeds the preview size budget")
                    return json.loads(result)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                    proc.wait()
