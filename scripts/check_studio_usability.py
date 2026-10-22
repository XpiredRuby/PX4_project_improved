"""Browser acceptance of drawing and review; never starts a simulator.

Requires Playwright, installed Chrome and an already running loopback studio.
Real HTTP compiler checks are exercised. Simulation start/status endpoints are
intercepted only for the explicitly reported UI ownership/race checks.
"""
import argparse
import json
from pathlib import Path
from urllib.parse import urlparse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8767")
    parser.add_argument("--chrome", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    parsed = urlparse(args.url)
    if (parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost")
            or parsed.path not in ("", "/") or parsed.username or parsed.password
            or parsed.query or parsed.fragment):
        parser.error("use a loopback studio URL")
    from playwright.sync_api import sync_playwright
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    results, errors = [], []
    report = {"simulation_ran": False, "results": results, "page_errors": errors}

    def passed(name):
        results.append({"case": name, "passed": True})

    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(executable_path=str(args.chrome), headless=True)
            context = browser.new_context(viewport={"width": 1440, "height": 1100},
                                          accept_downloads=True)
            page = context.new_page()
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(args.url, wait_until="networkidle")
            page.wait_for_function("() => settingsReady")
            assert page.evaluate("points") == [[0, 0]]
            assert page.locator("#planRoute").is_disabled()
            assert page.locator("#confirmPlan").is_disabled()
            passed("empty_start_and_disabled_actions")

            def pointer(east, north):
                page.locator("#route").scroll_into_view_if_needed()
                return page.evaluate("""p => {const b=route.getBoundingClientRect();
                    return [b.x+(origin[0]+scale*p[0])*b.width/route.width,
                            b.y+(origin[1]-scale*p[1])*b.height/route.height]}""", [east, north])

            def stroke(start, finish):
                page.mouse.move(*pointer(*start))
                page.mouse.down()
                page.mouse.move(*pointer(*finish), steps=6)
                page.mouse.up()

            stroke((0, 0), (0, 6))
            assert page.evaluate("points.at(-1)[1]") > 5.9
            saved = page.evaluate("points")
            stroke((10, 10), (12, 10))
            assert page.locator("#pendingStroke").is_visible()
            assert page.locator("#planRoute").is_disabled()
            assert page.evaluate("points") == saved
            page.locator("#discardStroke").click()
            assert page.evaluate("points") == saved
            stroke((10, 10), (12, 10))
            page.locator("#connectStroke").click()
            assert len(page.evaluate("points")) > len(saved)
            page.locator("#undo").click()
            assert page.evaluate("points") == saved
            page.locator("#redo").click()
            assert len(page.evaluate("points")) > len(saved)
            passed("draw_connect_discard_and_gesture_history")

            page.locator("#clear").click()
            page.locator("#pointTool").click()
            page.mouse.click(*pointer(0, 6))
            page.mouse.click(*pointer(.2, 6.2))
            assert len(page.evaluate("points")) == 3
            page.locator("#pointEditor").evaluate("el => el.open=true")
            page.locator("#pointEast").fill("4")
            page.locator("#pointNorth").fill("8")
            page.locator("#pointNorth").press("Enter")
            assert page.evaluate("points.at(-1)") == [4, 8]
            page.locator('select[aria-label="Type for point 1"]').select_option("stop")
            assert page.evaluate("documentInput().path.points[1].hard")
            page.locator("#closed").select_option("periodic")
            assert page.locator("#drawingError").is_visible()
            assert page.locator("#closed").input_value() == "open_stop"
            passed("short_details_keyboard_points_stop_and_loop_refusal")

            with page.expect_response(lambda r: r.url.endswith("/api/preview"), timeout=125000) as stopped:
                page.locator("#planRoute").click()
            stop_plan = stopped.value.json()
            assert stop_plan["accepted"]
            assert stop_plan["intent"]["path"]["points"][1]["hard"] is True
            assert stop_plan["summary"]["retiming"]["segment_count"] == 2
            passed("real_marked_stop_compilation")

            page.locator("#linePreset").click()
            with page.expect_response(lambda r: r.url.endswith("/api/preview"), timeout=125000) as planned:
                page.locator("#planRoute").click()
            result = planned.value.json()
            assert result["accepted"]
            assert page.locator("#results").is_visible()
            assert "returns to START" in page.locator("#reviewSummary").inner_text()
            assert not page.locator("#confirmPlan").is_disabled()
            assert page.locator("#rehearse").is_disabled()
            with page.expect_download() as download:
                page.locator("#downloadCsv").click()
            download.value.save_as(output / "verified-route.csv")
            passed("real_compilation_review_and_csv_download")

            page.locator("#motionOptions").evaluate("el => el.open=true")
            page.locator("#deadline").fill("1")
            assert page.locator("#results").is_hidden()
            assert not page.locator("#confirmPlan").is_checked()
            with page.expect_response(lambda r: r.url.endswith("/api/preview"), timeout=125000) as refused:
                page.locator("#planRoute").click()
            assert refused.value.status == 422
            assert refused.value.json()["limiting_constraint"] == "duration"
            assert "time limit is too short" in page.locator("#status").inner_text()
            page.locator("#deadline").fill("")
            passed("real_deadline_refusal_with_actionable_explanation")

            # Return a known valid old result after editing the submitted draft.
            pending = []
            page.route("**/api/preview", lambda route: pending.append(route))
            page.locator("#planRoute").click()
            page.wait_for_function("() => compiling")
            page.locator("#altitude").fill("21")
            for _ in range(100):
                if pending:
                    break
                page.wait_for_timeout(20)
            assert pending, "compiler request was not intercepted"
            pending.pop().fulfill(json=result)
            page.wait_for_function("() => !compiling")
            assert page.locator("#results").is_hidden()
            assert "discarded" in page.locator("#status").inner_text()
            page.unroute("**/api/preview")
            page.locator("#altitude").fill("20")
            passed("late_compile_cannot_restore_edited_plan")

            # Check start ownership with intercepted requests; no simulator starts.
            result["plan_id"] = "a" * 32
            result["simulation_compatible"] = True
            page.evaluate("r => {sitlEnabled=true;show(r)}", result)
            ready_state = {"stage": "READY"}
            page.route("**/api/rehearsal", lambda route: route.abort("failed"))
            page.route("**/api/rehearsal/" + "a" * 32,
                       lambda route: route.fulfill(json=ready_state))
            page.locator("#confirmPlan").check()
            page.locator("#rehearse").click()
            page.locator("#simulationStatus").filter(has_text="server reports").wait_for()
            assert page.evaluate("activeJob") is None
            assert page.locator("#rehearse").is_enabled()
            page.unroute("**/api/rehearsal")
            page.unroute("**/api/rehearsal/" + "a" * 32)
            passed("mocked_lost_start_response_recovers_server_state")
            starts = []
            page.route("**/api/rehearsal", lambda route: starts.append(route))
            fake_state = {"stage": "RUNNING", "phase": "TRAJECTORY",
                          "controller_trajectory_sha256": result["summary"]["controller_trajectory_sha256"]}
            page.route("**/api/rehearsal/" + "a" * 32,
                       lambda route: route.fulfill(json=fake_state))
            page.locator("#confirmPlan").check()
            page.locator("#rehearse").click()
            page.locator("#altitude").fill("22")
            page.evaluate("document.getElementById('rehearse').onclick()")
            for _ in range(100):
                if starts:
                    break
                page.wait_for_timeout(20)
            assert starts, "simulation start request was not intercepted"
            assert len(starts) == 1
            starts[0].fulfill(status=202, json=fake_state)
            page.wait_for_function("() => activeJob !== null && !startingJob")
            assert page.evaluate("activeJob") == "a" * 32
            assert page.locator("#results").is_hidden()
            assert "Earlier confirmed plan" in page.locator("#simulationStatus").inner_text()
            assert page.locator("#cancelRehearsal").is_enabled()
            passed("mocked_start_race_and_ownership_not_a_simulator_run")

            page.screenshot(path=str(output / "desktop.png"), full_page=True)
            page.set_viewport_size({"width": 390, "height": 844})
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth + 1")
            assert page.locator(".workspace").bounding_box()["y"] < page.locator("aside").bounding_box()["y"]
            page.locator("#route").scroll_into_view_if_needed()
            old_points = page.evaluate("points")
            page.locator("#zoomIn").click()
            assert page.evaluate("points") == old_points
            page.wait_for_timeout(150)
            page.screenshot(path=str(output / "mobile.png"), full_page=True)
            passed("desktop_and_mobile_no_horizontal_overflow")
            assert not errors, errors
            report["passed"] = True
            context.close()
            browser.close()
    except Exception as exc:
        report["passed"] = False
        report["error"] = str(exc)
        raise
    finally:
        (output / "assessment.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
