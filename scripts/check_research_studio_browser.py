"""Exercise offline planning, exports, forecast ingestion and recorded replay in Chromium."""
from __future__ import annotations

import argparse
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission_planner.research_studio import ResearchHandler  # noqa: E402


def check(page, base, csv, ulog, output):
    from playwright.sync_api import expect
    page.goto(base)
    page.get_by_label("Preferred cruise").fill("50")
    page.get_by_role("button", name="Plan & verify").click()
    page.get_by_text("ACCEPTED REFERENCE", exact=True).wait_for(timeout=150_000)
    verified = page.locator(".verification-grid").inner_text()
    assert "50.00 m/s" in verified and "NOT RUN" in verified
    print(verified.replace("\n", " "), flush=True)
    page.get_by_label("Wind speed (m/s)", exact=True).fill("5")
    page.get_by_role("button", name="Predict tracking").click()
    page.get_by_text("Predicted 3D tracking RMS", exact=False).wait_for()
    with page.expect_download() as prediction:
        page.get_by_role("button", name="Export predicted positions").click()
    prediction.value.save_as(output / "prediction.json")
    track = json.loads((output / "prediction.json").read_text())
    assert track["maxError"] > 0 and len(track["points"]) > 100
    print(f"Wind 5 m/s: predicted RMS {track['rms']:.3f} m, peak {track['maxError']:.3f} m", flush=True)
    page.get_by_text("Compare steady winds on this route", exact=True).click()
    assert page.get_by_role("button", name="Export wind study").is_visible()
    with page.expect_download() as download:
        page.get_by_role("button", name="Report JSON").click()
    download.value.save_as(output / "scenario.json")
    report = json.loads((output / "scenario.json").read_text())
    assert report["project"]["cruise"] == 50 and len(report["solverSourceSha256"]) == 64
    assert float(report["result"]["summary"]["speed_preferences"]["planned_route_peak_sampled_m_s"]) <= 3
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
    page.get_by_role("button", name="Geometry", exact=True).click()
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
    page.get_by_label("Aircraft model").select_option("hex")
    expect(page.get_by_label("Aircraft model")).to_have_value("hex", timeout=60_000)
    page.get_by_role("button", name="Plan & verify").click()
    page.get_by_text("ACCEPTED REFERENCE", exact=True).wait_for(timeout=150_000)
    print("PASS freehand hexacopter, replay, scenario round trip, hourly forecast fixture and sweeps", flush=True)


def main():
    from playwright.sync_api import sync_playwright
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chromium", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--ulog", type=Path)
    args = parser.parse_args()
    with ThreadingHTTPServer(("127.0.0.1", 0), ResearchHandler) as server:
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            with sync_playwright() as driver, tempfile.TemporaryDirectory() as directory:
                browser = driver.chromium.launch(executable_path=args.chromium, args=["--no-sandbox"])
                page = browser.new_page(viewport={"width": 1440, "height": 920}, accept_downloads=True)
                errors = []
                page.on("pageerror", lambda error: errors.append(str(error)))
                check(page, f"http://127.0.0.1:{server.server_port}", args.csv, args.ulog, Path(directory))
                assert not errors, errors
                browser.close()
        finally:
            server.shutdown()


if __name__ == "__main__":
    main()
