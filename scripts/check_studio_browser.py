"""Exercise the real local studio confirmation-to-Gazebo flow in Chrome.

Requires optional Playwright and an already configured local studio. This
actively runs a SITL mission; it has no physical-flight capability.
"""

import argparse
import json
import os
from pathlib import Path
import tempfile
import time
import traceback
from urllib.parse import urlparse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm-sitl", action="store_true", required=True)
    parser.add_argument("--url", default="http://127.0.0.1:8766")
    parser.add_argument("--chrome", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=420.0)
    parser.add_argument("--registered-aircraft", action="store_true",
                        help="load and check the registered reusable aircraft profile")
    args = parser.parse_args()
    parsed = urlparse(args.url)
    if (
        parsed.scheme != "http"
        or parsed.hostname not in ("127.0.0.1", "localhost")
        or parsed.path not in ("", "/")
        or parsed.query
        or parsed.fragment
        or parsed.username
        or parsed.password
    ):
        parser.error("the browser test accepts only a loopback studio URL")
    if not 30 <= args.timeout_s <= 900:
        parser.error("timeout must be between 30 and 900 seconds")
    url = args.url.rstrip("/")
    from playwright.sync_api import sync_playwright

    base = args.output.resolve()
    base.mkdir(parents=True, exist_ok=False)
    temp = base / "temp"
    temp.mkdir(exist_ok=True)
    os.environ["TEMP"] = str(temp)
    os.environ["TMP"] = str(temp)
    tempfile.tempdir = str(temp)
    results = []
    errors = []
    identifier = None

    def state(stage, **extra):
        (base / "state.json").write_text(
            json.dumps(
                {
                    "stage": stage,
                    "plan_id": identifier,
                    "results": results,
                    "page_errors": errors,
                    **extra,
                },
                indent=2,
            )
        )

    try:
        with sync_playwright() as p:
            context = p.chromium.launch_persistent_context(
                str(base / "profile"),
                executable_path=str(args.chrome.resolve()),
                headless=True,
                viewport={"width": 1440, "height": 1100},
                accept_downloads=True,
                downloads_path=str(base / "downloads"),
            )
            page = context.new_page()
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(url, wait_until="networkidle", timeout=30000)
            page.locator("#simulationStatus").filter(
                has_text="Local PX4/Gazebo backend configured"
            ).wait_for()
            if args.registered_aircraft:
                page.locator("#useRegisteredAircraft").click()
                with page.expect_response(lambda r: r.url.endswith("/api/aircraft/assess")) as setup:
                    page.locator("#checkAircraft").click()
                assert setup.value.json()["ready"]
                page.locator("#saveAircraft").click()
                results.append({"case": "reusable_aircraft_setup_and_model_binding", "passed": True})
            else:
                page.locator("#aircraftMode").select_option("demo")
            page.locator("#linePreset").click()
            assert page.locator("#altitude").input_value() == "20"
            with page.expect_response(
                lambda r: r.url.endswith("/api/preview"), timeout=125000
            ) as response:
                page.locator("#compile").click()
            result = response.value.json()
            identifier = result["plan_id"]
            assert result["accepted"] and result["verification"]["accepted"]
            page.locator("#results").wait_for(state="visible")
            assert page.locator("#rehearse").is_disabled()
            digest = result["summary"]["controller_trajectory_sha256"]
            bad = page.request.post(
                url + "/api/rehearsal",
                data={
                    "plan_id": identifier,
                    "controller_trajectory_sha256": "0" * 64,
                    "confirm_interpretation": True,
                },
            )
            assert bad.status == 400
            results.append(
                {
                    "case": "exact_review_gate_and_modified_hash_refusal",
                    "passed": True,
                    "trajectory_sha256": digest,
                }
            )
            page.locator("#confirmPlan").check()
            with page.expect_response(
                lambda r: r.url.endswith("/api/rehearsal"), timeout=30000
            ) as started:
                page.locator("#rehearse").click()
            assert started.value.status == 202
            state("SIMULATION_STARTED")
            second = context.new_page()
            second.on("pageerror", lambda e: errors.append(str(e)))
            second.goto(url, wait_until="networkidle")
            second.locator("#simulationStatus").filter(
                has_text="Earlier confirmed plan:"
            ).wait_for()
            second.locator("#altitude").fill("21")
            assert second.locator("#results").is_hidden()
            assert (
                "earlier confirmed plan"
                in second.locator("#simulationStatus").inner_text().lower()
            )
            second.reload(wait_until="networkidle")
            second.locator("#simulationStatus").filter(
                has_text="Earlier confirmed plan:"
            ).wait_for()
            results.append(
                {
                    "case": "edit_does_not_cancel_or_rebind_job_and_refresh_resumes",
                    "passed": True,
                }
            )
            deadline = time.monotonic() + args.timeout_s
            while time.monotonic() < deadline:
                status = page.request.get(url + "/api/rehearsal/" + identifier).json()
                state(status["stage"], latest_error=status.get("error"))
                if status["stage"] in ("COMPLETED", "FAILED"):
                    break
                page.wait_for_timeout(2000)
            else:
                raise TimeoutError("connected rehearsal exceeded its test timeout")
            (base / "final_job.json").write_text(json.dumps(status, indent=2))
            assert status["stage"] == "COMPLETED", status.get("error")
            assert status["assessment"]["sampled_rehearsal_passed"], [
                c for c in status["assessment"]["checks"] if not c["passed"]
            ]
            assert status["flight_approval"] is False
            page.locator("#downloadAssessment").wait_for(state="visible", timeout=10000)
            assert (
                "Gazebo sampled rehearsal passed"
                in page.locator("#status").inner_text()
            )
            assert (
                "Gazebo completed; sampled checks passed"
                in page.locator("#gazeboNotice").inner_text()
            )
            assert page.evaluate("simulationAssessment.plan_id===lastResult.plan_id")
            assert page.locator("#rehearse").is_disabled()
            with page.expect_download() as download:
                page.locator("#downloadAssessment").click()
            download.value.save_as(str(base / "downloaded_assessment.json"))
            downloaded = json.loads((base / "downloaded_assessment.json").read_text())
            assert (
                downloaded["plan_id"] == identifier
                and downloaded["hardware_approval"] is False
            )
            assert downloaded["track_samples"]
            results.append(
                {
                    "case": "real_browser_to_gazebo_and_truth_overlay_and_download",
                    "passed": True,
                    "duration_s": result["summary"]["retiming"]["selected_duration_s"],
                }
            )
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
            page.screenshot(path=str(base / "connected-desktop.png"), full_page=True)
            page.set_viewport_size({"width": 390, "height": 844})
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
            page.screenshot(path=str(base / "connected-mobile.png"), full_page=True)
            results.append(
                {"case": "connected_desktop_and_mobile_layout", "passed": True}
            )
            assert not errors, errors
            context.close()
        state("PASSED")
    except Exception:
        (base / "error.txt").write_text(traceback.format_exc())
        state("FAILED")
        raise


if __name__ == "__main__":
    main()
