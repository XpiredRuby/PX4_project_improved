"""Exercise offline planning, exports, forecast ingestion and recorded replay in Chromium."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import tempfile
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission_planner.research_studio import research_server  # noqa: E402


def check_navigation(page, base):
    from playwright.sync_api import expect

    page.goto(base)
    workflow = page.get_by_role("navigation", name="Mission workflow")
    expect(workflow.get_by_role("button", name="Build route")).to_have_attribute("aria-current", "step")
    help_button = page.get_by_role("button", name="Help", exact=True)
    help_button.focus()
    page.keyboard.press("Enter")
    dialog = page.get_by_role("dialog", name="Studio help")
    expect(dialog.get_by_role("button", name="Close help")).to_be_focused()
    dialog.get_by_role("button", name="Read the route map").click()
    expect(dialog.get_by_text("North and east", exact=True)).to_be_visible()
    page.keyboard.press("Escape")
    expect(dialog).to_be_hidden()
    expect(help_button).to_be_focused()

    page.get_by_role("button", name="Map help", exact=True).click()
    expect(dialog.get_by_text("North and east", exact=True)).to_be_visible()
    dialog.get_by_role("button", name="Close help").click()
    expect(page.get_by_role("button", name="Map help", exact=True)).to_be_focused()

    workflow.get_by_role("button", name="Review results").click()
    expect(page.get_by_role("heading", name="Review results", exact=True)).to_be_focused()
    expect(page.get_by_text("Check a route to see its planned motion", exact=False)).to_be_visible()
    expect(page.get_by_role("navigation", name="Result views").get_by_role("button", name="Checks", exact=True)).to_have_attribute("aria-pressed", "true")
    workflow.get_by_role("button", name="Build route").click()
    expect(page.get_by_role("button", name="Plan & verify")).to_be_enabled()

    for width, height in ((1366, 768), (1024, 640), (390, 844), (320, 620)):
        page.set_viewport_size({"width": width, "height": height})
        assert page.evaluate("document.documentElement.scrollWidth") == width
        assert page.evaluate("document.documentElement.scrollHeight") == height
        box = page.get_by_role("button", name="Help", exact=True).bounding_box()
        assert box and box["width"] >= 24 and box["height"] >= 24
        for label in ("Mission coordinate drawing canvas", "Waypoints"):
            target = page.get_by_label(label, exact=True)
            if label == "Waypoints":
                target = page.get_by_role("region", name=label, exact=True)
            bounds = target.bounding_box()
            assert bounds and bounds["height"] > 0 and bounds["y"]+bounds["height"] <= height
        for label in ("Open project", "Save editable project", "Help"):
            bounds = page.get_by_role("button", name=label, exact=True).bounding_box()
            assert bounds and bounds["x"] >= 0 and bounds["x"]+bounds["width"] <= width + .5, (label, width, bounds)
    page.get_by_role("button", name="Settings", exact=True).click()
    expect(page.get_by_label("Height above launch (m)")).to_be_visible()
    page.get_by_role("button", name="Project", exact=True).click()
    page.get_by_role("button", name="200 m · five stops").click()
    expect(page.get_by_label("Draft route dimensions").get_by_text("200.0 m")).to_be_visible()
    next_page = page.get_by_role("button", name="Next waypoint page")
    expect(next_page).to_be_enabled()
    page.locator(".waypoint-table tbody tr").first.click()
    remove_button = page.get_by_role("button", name="Remove selected")
    expect(remove_button).to_be_enabled()
    next_page.click()
    expect(page.get_by_role("button", name="Previous waypoint page")).to_be_enabled()
    expect(remove_button).to_be_disabled()
    page.set_viewport_size({"width": 1440, "height": 920})
    print("PASS fixed viewport, waypoint paging, compact docks, and keyboard help", flush=True)


def check(page, base, csv, ulog, output):
    from playwright.sync_api import expect
    page.add_init_script("""
      const begin = CanvasRenderingContext2D.prototype.moveTo;
      const line = CanvasRenderingContext2D.prototype.lineTo;
      CanvasRenderingContext2D.prototype.moveTo = function (x, y) {
        if (['#ffffff','#d36f73','#56c18f'].includes(this.strokeStyle)) this.vectorStart = [x, y];
        return begin.call(this, x, y);
      };
      CanvasRenderingContext2D.prototype.lineTo = function (x, y) {
        if (this.vectorStart) {
          const vector = [x-this.vectorStart[0], y-this.vectorStart[1]];
          if (this.strokeStyle === '#d36f73') window.northVector = vector;
          if (this.strokeStyle === '#56c18f') window.eastVector = vector;
          if (this.strokeStyle === '#ffffff' && Math.abs(Math.hypot(...vector)-24)<1e-9)
            window.headingVector = vector;
          this.vectorStart = null;
        }
        return line.call(this, x, y);
      };
    """)
    page.goto(base)
    page.get_by_label("P02 north").fill("14")
    page.get_by_role("button", name="Remove selected").click()
    page.get_by_role("button", name="Add row").click()
    assert page.get_by_label("P05 north").count() == 1
    page.get_by_role("button", name="Undo").click()
    assert page.get_by_label("P05 north").count() == 0
    assert page.get_by_label("P02 north").count() == 0
    page.get_by_role("button", name="Undo").click()
    assert page.get_by_label("P02 north").input_value() == "14"
    page.get_by_role("button", name="Undo").click()
    assert page.get_by_label("P02 north").input_value() == "12"
    page.get_by_label("Preferred cruise").fill("50")
    page.get_by_role("button", name="Plan & verify").click()
    page.locator(".verification-grid").get_by_text("Verified", exact=True).wait_for(timeout=150_000)
    verified = page.locator(".verification-grid").inner_text()
    assert "50.00 m/s" in verified
    print(verified.replace("\n", " "), flush=True)
    page.get_by_role("button", name="Play in 3D").click()
    page.get_by_text("TAKEOFF", exact=True).wait_for()
    view = page.get_by_label("3D trajectory orbit visualization", exact=False)
    view.scroll_into_view_if_needed()
    page.wait_for_function("window.headingVector")
    correct_yaw = """() => {
      const n=window.northVector, h=window.headingVector;
      return n && h && (n[0]*h[0]+n[1]*h[1])/(Math.hypot(...n)*Math.hypot(...h)) > 1-1e-9;
    }"""
    page.wait_for_function(correct_yaw)
    page.get_by_role("button", name="Top view", exact=True).click()
    page.wait_for_function("""() => {
      const n = window.northVector, e = window.eastVector;
      return n && e && Math.abs(n[0]) < .01 && n[1] < -1 && e[0] > 1 && Math.abs(e[1]) < .01;
    }""")
    page.get_by_role("button", name="Orbit", exact=True).click()
    page.wait_for_function(correct_yaw)
    original_heading = page.evaluate("window.headingVector")
    bounds = view.bounding_box()
    x, y = bounds["x"] + bounds["width"] / 2, bounds["y"] + bounds["height"] / 2
    page.mouse.move(x, y)
    page.mouse.down()
    page.mouse.move(x + 110, y, steps=8)
    page.mouse.up()
    page.wait_for_function(
        "(a) => window.headingVector.some((v,i)=>Math.abs(v-a[i])>2)",
        arg=original_heading,
    )
    page.wait_for_function(correct_yaw)
    page.get_by_role("button", name="Play or pause").click()
    expect(page.get_by_label("Reference time")).not_to_have_value("0")
    page.get_by_title("Reset time").click()
    expect(page.get_by_label("Reference time")).to_have_value("0")
    page.get_by_role("button", name="Review results", exact=True).click()
    page.get_by_role("button", name="Weather for analysis", exact=True).click()
    page.get_by_label("Wind speed (m/s)", exact=True).fill("5")
    page.get_by_role("button", name="Build route", exact=True).click()
    page.get_by_role("button", name="Plan & verify").click()
    page.locator(".verification-grid").get_by_text("Verified", exact=True).wait_for(timeout=150_000)
    page.get_by_role("button", name="Wind analysis", exact=True).click()
    page.get_by_role("button", name="Predict tracking").click()
    page.get_by_text("Predicted 3D tracking RMS", exact=False).wait_for()
    with page.expect_download() as prediction:
        page.get_by_role("button", name="Export predicted positions").click()
    prediction.value.save_as(output / "prediction.json")
    track = json.loads((output / "prediction.json").read_text())
    assert track["maxError"] > 0 and len(track["points"]) > 100
    print(f"Wind 5 m/s: predicted RMS {track['rms']:.3f} m, peak {track['maxError']:.3f} m", flush=True)
    page.get_by_role("button", name="Compare winds", exact=True).click()
    page.get_by_text("Compare steady winds on this route", exact=True).click()
    assert page.get_by_role("button", name="Export wind study").is_visible()
    page.get_by_role("button", name="Checks", exact=True).click()
    with page.expect_download() as download:
        page.get_by_role("button", name="Report JSON").click()
    download.value.save_as(output / "scenario.json")
    report = json.loads((output / "scenario.json").read_text())
    assert report["project"]["cruise"] == 50 and len(report["solverSourceSha256"]) == 64
    assert float(report["result"]["summary"]["speed_preferences"]["planned_route_peak_sampled_m_s"]) <= 3
    page.get_by_role("button", name="Recordings", exact=True).click()
    replay = page.locator("section").filter(has=page.get_by_role("heading", name="Recorded flight replay"))
    replay.get_by_role("button", name="Load recorded x500 flight").click()
    replay.get_by_text("Retained x500 SITL", exact=False).wait_for()
    assert "3508 paired" in replay.inner_text()
    replay.get_by_label("Recording view").select_option("height")
    assert replay.get_by_label("Recorded height along the timeline").is_visible()
    replay.get_by_role("button", name="Play replay").click()
    expect(replay.get_by_label("Recording sample")).not_to_have_value("0")
    if csv:
        replay.locator("input[type=file]").set_input_files(csv)
        replay.get_by_text(csv.name, exact=True).wait_for()
        assert "Hardware motion capture" in replay.inner_text()
    if ulog:
        replay.locator("input[type=file]").set_input_files(ulog)
        replay.get_by_text(ulog.name, exact=True).wait_for()
        assert "PX4 position estimate" in replay.inner_text()
    page.get_by_role("button", name="Speed sweep", exact=True).click()
    sweep = page.locator("section").filter(has=page.get_by_role("heading", name="Compare requested speeds"))
    sweep.get_by_role("button", name="Run speed sweep").click()
    sweep.locator("tbody tr").nth(3).wait_for(timeout=300_000)
    print(sweep.locator("tbody").inner_text(), flush=True)
    assert all(float(cell.inner_text()) <= 3 for cell in sweep.locator("tbody tr td:nth-child(2)").all())
    with page.expect_download() as download:
        page.get_by_title("Save editable project").click()
    download.value.save_as(output / "project.json")
    page.locator('input[accept=".json"]').set_input_files(output / "project.json")
    page.get_by_text("No active verified result", exact=True).wait_for()
    if not page.get_by_role("button", name="Live forecast", exact=True).is_visible():
        page.get_by_role("button", name="Weather for analysis", exact=True).click()
    page.get_by_role("button", name="Live forecast", exact=True).click()
    target = "2026-11-21T12:00"
    page.get_by_label("Forecast instant").fill(target)
    hourly = {"time": [target], "relative_humidity_2m": [76], "temperature_2m": [20],
              "surface_pressure": [990], "wind_speed_10m": [5], "wind_direction_10m": [90],
              "wind_gusts_10m": [8]}
    page.route("https://api.open-meteo.com/**", lambda route: route.fulfill(json={"hourly": hourly}))
    page.get_by_role("button", name="Fetch live forecast").click()
    page.get_by_text("Open-Meteo forecast", exact=False).wait_for()
    assert page.get_by_label("Relative humidity").input_value() == "76"
    page.get_by_label("Latitude").fill("32")
    assert page.get_by_label("Wind speed (m/s)", exact=True).input_value() == ""
    page.get_by_role("button", name="Manual", exact=True).click()
    assert page.get_by_label("Wind speed (m/s)", exact=True).input_value() == "5"
    assert page.get_by_label("Wind from (°)", exact=True).input_value() == "0"
    page.get_by_role("button", name="Build route", exact=True).click()
    page.get_by_role("button", name="New blank mission", exact=False).click()
    canvas = page.get_by_label("Mission coordinate drawing canvas")
    canvas.scroll_into_view_if_needed()
    bounds = canvas.bounding_box()
    x, y = bounds["x"] + bounds["width"] * .46, bounds["y"] + bounds["height"] * .5
    page.mouse.move(x, y)
    page.mouse.down()
    page.mouse.move(x+35, y-25, steps=6)
    page.mouse.move(x+65, y+15, steps=6)
    page.mouse.up()
    page.get_by_role("navigation", name="Settings views").get_by_role("button", name="Aircraft").click()
    page.get_by_label("Aircraft model").select_option("hex")
    expect(page.get_by_label("Aircraft model")).to_have_value("hex", timeout=60_000)
    page.get_by_role("button", name="Plan & verify").click()
    page.locator(".verification-grid").get_by_text("Verified", exact=True).wait_for(timeout=150_000)
    print("PASS freehand hexacopter, replay, scenario round trip, hourly forecast fixture and sweeps", flush=True)


def read_download(page, label):
    with page.expect_download() as item:
        page.get_by_role("button", name=label, exact=True).click()
    return json.loads(Path(item.value.path()).read_text())


def check_studies(page, base, output):
    from playwright.sync_api import expect
    page.goto(base)
    page.get_by_role("button", name="200 m · five stops").click()
    page.get_by_role("button", name="Plan & verify").click()
    page.locator(".verification-grid").get_by_text("Verified", exact=True).wait_for(timeout=150_000)
    project_report = read_download(page, "Report JSON")
    page.get_by_role("button", name="Compare flights", exact=True).click()
    expected_sha = project_report["result"]["summary"]["controller_trajectory_sha256"]
    page.get_by_role("button", name="Test report with synthetic run").click()
    synthetic = read_download(page, "Export mission comparison")
    assert synthetic["evidence"] == "synthetic fixture" and synthetic["verdict"] == "pass", synthetic["checks"]
    assert len(synthetic["visits"]) == 5 and all(s["verdict"] == "pass" for s in synthetic["visits"])
    assert synthetic["routeSha256"] == expected_sha and abs(synthetic["tracking"]["rmsM"]-.1) < 1e-8
    print("200 m synthetic report: five ordered stops, 0.100 m injected RMS and landing error", flush=True)
    page.get_by_role("button", name="Test simulator handoff offline").click()
    page.get_by_text("Synthetic handoff · assessment matched", exact=False).wait_for()
    page.get_by_role("button", name="Review archived X500 run").click()
    archived = read_download(page, "Export mission comparison")
    assert archived["evidence"] == "retained simulator recording" and archived["verdict"] == "diagnostic only"
    assert math.isclose(archived["landing"]["horizontalErrorM"], .07827087451620429, abs_tol=1e-7)
    assert archived["coverage"]["truthSamples"] == 2539
    assert next(c["verdict"] for c in archived["checks"] if c["name"] == "Route timing") == "fail"
    print("Archived X500 report: tracking RMS %.4f m, touchdown error %.4f m, elapsed route %.2f s" %
          (archived["tracking"]["rmsM"], archived["landing"]["horizontalErrorM"], archived["runPhaseTimeS"]["elapsed"]), flush=True)
    page.get_by_text("Compare an imported run with this plan", exact=True).click()
    directory = ROOT / "validation/2026-10-03/cases/raised-10m-final"
    page.get_by_label("Aligned telemetry CSV or CSV.gz").set_input_files(directory / "telemetry_and_truth.csv.gz")
    page.get_by_label("Run manifest (optional)").set_input_files(next(directory.glob("run_manifest_*.json")))
    page.get_by_role("button", name="Compare imported run").click()
    page.get_by_text("Route identity mismatch.", exact=False).wait_for()

    page.get_by_role("button", name="Saved studies", exact=True).click()
    page.get_by_label("Batch speeds").fill("1, 25, 50")
    with page.expect_response(lambda r: r.url.endswith("/api/studies") and r.request.method == "POST") as response:
        page.get_by_role("button", name="Queue study").click()
    identifier = response.value.json()["id"]
    page.get_by_role("button", name="Weather for analysis", exact=True).click()
    page.get_by_label("Wind speed (m/s)", exact=True).fill("5")
    page.get_by_label("Batch speeds").fill("50")
    with page.expect_response(lambda r: r.url.endswith("/api/studies") and r.request.method == "POST") as response:
        page.get_by_role("button", name="Queue study").click()
    windy = response.value.json()["id"]
    browser = page.context.browser
    page.close()
    page = browser.new_page(accept_downloads=True)
    page.goto(base)
    page.get_by_role("button", name="Review results", exact=True).click()
    page.get_by_role("button", name="Saved studies", exact=True).click()
    selector = page.get_by_role("combobox", name="Saved studies", exact=False)
    expect(selector.locator("option")).to_have_count(2)
    selector.select_option(identifier)
    button = page.get_by_role("button", name="Download case 3")
    button.wait_for(timeout=150_000)
    case = read_download(page, "Download case 3")
    assert case["metrics"]["route_sha256"] == expected_sha
    assert case["project"]["environment"]["windSpeed"] == 0
    assert abs(case["metrics"]["distance_m"]-200) < 1e-5
    assert case["metrics"]["requested_m_s"] == 50 and abs(case["metrics"]["peak_m_s"]-2.6461693548)<1e-6
    downloaded = output / "saved-case.json"
    downloaded.write_text(json.dumps(case))
    page.get_by_role("button", name="Compare flights", exact=True).click()
    page.get_by_text("Use a saved study case", exact=True).click()
    page.get_by_label("Verified study case JSON").set_input_files(downloaded)
    page.get_by_text("Selected saved case", exact=False).wait_for()
    page.get_by_role("button", name="Test report with synthetic run").click()
    reopened = read_download(page, "Export mission comparison")
    assert reopened["routeSha256"] == expected_sha and reopened["verdict"] == "pass"
    page.get_by_role("button", name="Saved studies", exact=True).click()
    saved = read_download(page, "Export study")
    assert saved["snapshot"]["speeds"] == [1,25,50] and saved["id"] == identifier
    selector.select_option(windy)
    expect(page.get_by_role("button", name="Download case 3")).to_have_count(0)
    page.get_by_role("button", name="Download case 1").wait_for(timeout=150_000)
    wind = read_download(page, "Download case 1")
    assert wind["project"]["environment"]["windSpeed"] == 5 and wind["metrics"]["peak_airspeed_m_s"] > 5
    page.get_by_role("button", name="Restore scenario").click()
    expect(page.get_by_label("Preferred cruise")).to_have_value("50")
    page.get_by_role("button", name="Weather for analysis", exact=True).click()
    expect(page.get_by_label("Wind speed (m/s)", exact=True)).to_have_value("5")
    (output / "archived-comparison.json").write_text(json.dumps(archived))
    print("PASS both frozen studies survived tab closure, exact route re-exported, windy scenario restored", flush=True)
    page.close()


def main():
    from playwright.sync_api import sync_playwright
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chromium", type=Path)
    parser.add_argument("--assets", type=Path, help="Test an alternate production build")
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--ulog", type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as studies, research_server(0, studies, args.assets) as server:
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            with sync_playwright() as driver, tempfile.TemporaryDirectory() as directory:
                browser = driver.chromium.launch(executable_path=args.chromium, args=["--no-sandbox"])
                page = browser.new_page(viewport={"width": 1440, "height": 920}, accept_downloads=True)
                errors = []
                page.on("pageerror", lambda error: errors.append(str(error)))
                check_navigation(page, f"http://127.0.0.1:{server.server_port}")
                check(page, f"http://127.0.0.1:{server.server_port}", args.csv, args.ulog, Path(directory))
                check_studies(page, f"http://127.0.0.1:{server.server_port}", Path(directory))
                assert not errors, errors
                browser.close()
        finally:
            server.shutdown()


if __name__ == "__main__":
    main()
