"""Unattended study recovery, identity and evidence at the native service boundary."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from uuid import uuid4

from mission_planner.study_queue import StudyQueue

ROOT = Path(__file__).resolve().parents[1]
CSV = "time,x,y,z,vx,vy,vz\n0,0,0,-12,0,0,0\n.05,0,0,-12,0,0,0\n"


def submission(speeds):
    project = json.loads((ROOT / "studio-v2/examples/200m-five-stops.px4-project.json").read_text())
    request = {"source": "drawing", "planning_mode": "closest", "document": {
        "motion": {"target_speed_m_s": project["cruise"], "maximum_speed_m_s": project["speedCap"], "fixed_yaw_deg": 0},
        "path": {"maximum_deviation_m": .5, "points": [
            {"point_id": p["id"], "north_m": p["north"], "east_m": p["east"],
             "down_m": -p["altitude"], "hard": True, "exact": False} for p in project["points"]]}}}
    return {"id": str(uuid4()), "project": project, "request": request, "speeds": speeds}


def result(request, active):
    return {"accepted": True, "trajectory_csv": CSV, "summary": {
        "controller_trajectory_sha256": hashlib.sha256(CSV.encode()).hexdigest(),
        "speed_preferences": {"planned_route_peak_sampled_m_s": 0, "route_duration_s": .05,
                              "route_distance_sampled_m": 0, "reasons": []}}}


def await_cases(queue, identifier, states):
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        rows = queue.status(identifier)["cases"]
        if [r["state"] for r in rows] == states:
            return rows
        time.sleep(.02)
    raise AssertionError(f"Expected {states}, last state {rows}")


class TestStudyQueue(unittest.TestCase):
    def test_concurrent_duplicate_submissions_preserve_single_frozen_study(self):
        with tempfile.TemporaryDirectory() as tmp:
            queue = StudyQueue(tmp, compiler=result)
            self.addCleanup(queue.close)
            with self.assertRaisesRegex(ValueError, "already owns"):
                StudyQueue(tmp)
            payload = submission([25, 50, 170])
            with ThreadPoolExecutor(max_workers=4) as pool:
                list(pool.map(queue.submit, [payload]*4))
            payload["project"]["environment"]["windSpeed"] = 10
            with self.assertRaisesRegex(ValueError, "different scenario"):
                queue.submit(payload)
            await_cases(queue, payload["id"], ["verified"]*3)
            self.assertEqual(len(queue.list()), 1)
            record = queue.status(payload["id"], snapshot=True)
            self.assertEqual(record["snapshot"]["project"]["environment"]["windSpeed"], 0)
            artifact = json.loads(queue.artifact(payload["id"], 2))
            self.assertEqual(artifact["metrics"]["requested_m_s"], 170)
            self.assertEqual(artifact["request"]["document"]["motion"]["target_speed_m_s"], 170)
            self.assertEqual(artifact["metrics"]["route_sha256"], hashlib.sha256(CSV.encode()).hexdigest())
            self.assertEqual(artifact["engine"], record["engine"])
            self.assertEqual(artifact["project"]["cruise"], 170)
            self.doCleanups()

    def test_cancel_and_immediate_retry_discards_old_result(self):
        gate = threading.Event()
        started = threading.Event()
        calls = []

        def compile_case(request, active):
            calls.append(1)
            if len(calls) == 1:
                started.set()
                if not gate.wait(5):
                    raise TimeoutError("test gate")
                return {"accepted": False, "error": "stale result"}
            return result(request, active)

        with tempfile.TemporaryDirectory() as tmp:
            q = StudyQueue(tmp, compiler=compile_case)
            try:
                p = submission([50])
                q.submit(p)
                self.assertTrue(started.wait(5))
                q.change(p["id"], "cancel")
                await_cases(q, p["id"], ["cancelled"])
                q.change(p["id"], "retry")
                gate.set()
                self.assertEqual(await_cases(q, p["id"], ["verified"])[0]["attempts"], 2)
            finally:
                gate.set()
                q.close()

    def test_killed_worker_recovers_only_uncommitted_case(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = submission([1, 2])
            script = """
from mission_planner.study_queue import StudyQueue
from tests.test_study_queue import result
import sys,time,json
def compile_case(request, active):
    if request['document']['motion']['target_speed_m_s']==2:
        print('second claimed',flush=True)
        time.sleep(120)
    return result(request,active)
q=StudyQueue(sys.argv[1],compiler=compile_case)
q.submit(json.loads(sys.stdin.readline()))
time.sleep(120)
"""
            with subprocess.Popen([sys.executable, "-c", script, tmp], cwd=ROOT,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True) as process:
                try:
                    process.stdin.write(json.dumps(p) + "\n")
                    process.stdin.flush()
                    ready = threading.Event()

                    def listen():
                        if process.stdout.readline().strip() == "second claimed":
                            ready.set()

                    reader = threading.Thread(target=listen, daemon=True)
                    reader.start()
                    self.assertTrue(ready.wait(10))
                finally:
                    process.kill()
                    process.wait(timeout=5)
                    reader.join(2)
            q = StudyQueue(tmp, compiler=result)
            try:
                rows = await_cases(q, p["id"], ["verified", "verified"])
                self.assertEqual([c["attempts"] for c in rows], [1, 2])
                self.assertTrue(q.artifact(p["id"], 0))
            finally:
                q.close()

    def test_timeout_real_compiler_process_and_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            q = StudyQueue(tmp, timeout=.001)
            try:
                p = submission([50])
                real_popen = subprocess.Popen

                def slow_child(args, **kwargs):
                    return real_popen([sys.executable, "-c", "import time; time.sleep(60)"], **kwargs)
                with patch("mission_planner.study_queue.subprocess.Popen", side_effect=slow_child):
                    q.submit(p)
                    row = await_cases(q, p["id"], ["timed_out"])[0]
                    self.assertIn("deadline", row["error"])
                q.compiler = result
                q.change(p["id"], "retry")
                self.assertEqual(await_cases(q, p["id"], ["verified"])[0]["attempts"], 2)
            finally:
                q.close()

    def test_inconsistent_snapshot_wrong_digest_and_invalid_speed(self):
        with tempfile.TemporaryDirectory() as tmp:
            def corrupt(request, active):
                reply = result(request, active)
                reply["trajectory_csv"] += "\n"
                return reply
            q = StudyQueue(tmp, compiler=corrupt)
            try:
                p = submission([50])
                bad = deepcopy(p)
                bad["project"]["points"][2]["north"] = 500
                with self.assertRaisesRegex(ValueError, "waypoints"):
                    q.submit(bad)
                with self.assertRaisesRegex(ValueError, "positive"):
                    q.submit(dict(p, speeds=[-1]))
                q.submit(p)
                self.assertIn("different digests", await_cases(q, p["id"], ["failed"])[0]["error"])
                with self.assertRaises(KeyError):
                    q.artifact(p["id"], 0)
            finally:
                q.close()
