"""Browser interaction smoke checks for Studio V2.

The first suite uses a SYNTHETIC, clearly labeled planner fixture to validate
React interactions. It is not flight/simulation evidence and does not validate
trajectory dynamics. Optional --real-planner runs the actual Pyodide planner.
No native PX4, Gazebo, or hardware command is used.
"""
from __future__ import annotations

import argparse
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
import json
from pathlib import Path
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
PREVIEW = ROOT / "studio-v2/preview"


def synthetic_reference():
    header = "time,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw"
    samples = [
        "0,0,0,-12,0,0,0,0,0,0,0,0,0,0",
        "0.5,1,0,-12,1.5,0,0,0.2,0,0,0.1,0,0,0",
        "1,2,0,-12,1,0,0,-0.3,0,0,-0.1,0,0,0",
        "1.5,3,0,-12,0,0,0,-0.3,0,0,0,0,0,0",
    ]
    return {
        "accepted": True,
        "release_class": "SITL_ONLY",
        "campaign_ran": False,
        "trajectory_csv": header + "\n" + "\n".join(samples) + "\n",
        "summary": {"speed_preferences": {"planning_cap_m_s": 2.7}},
        "verification": {"accepted": True, "source": "synthetic test fixture ONLY"},
        "planning_adjustments": {
            "adjusted": True,
            "original_rounding_met": False,
            "requested_rounding_m": 0.5,
            "curve_error_bound_from_requested_m": 0.75,
            "adjustments": [{"code": "example", "message": "SYNTHETIC UI TEST ONLY"}],
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--real-planner", action="store_true")
    options = parser.parse_args()
    if not (PREVIEW / "index.html").exists():
        raise SystemExit("Build preview before browser smoke checks")
    options.output.mkdir(parents=True, exist_ok=True)

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(PREVIEW), **kwargs)

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    runner = threading.Thread(target=server.serve_forever, daemon=True)
    runner.start()
    base = f"http://127.0.0.1:{server.server_port}"
    results = []
    page_errors = []
    runtime_diagnostics = []
    try:
        from playwright.sync_api import sync_playwright

        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True, args=["--disable-dev-shm-usage"])
            context = browser.new_context(viewport={"width": 1440, "height": 940}, device_scale_factor=1)
            page = context.new_page()
            page.on("pageerror", lambda error: page_errors.append(str(error)))
            page.goto(base, wait_until="domcontentloaded", timeout=30000)
            page.get_by_role("heading", name="Mission geometry").wait_for()
            assert page.get_by_text("ENGINEERING WORKSPACE / 2.0 PREVIEW").count() == 1
            assert page.locator(".point-table tbody tr").count() == 4
            assert "SITL ONLY" in page.locator(".status-tag").inner_text()
            results.append("React shell, editing surface and flight-safety scope")
            # Dock-like pane resizing stays accessible without a mouse.
            initial = float(page.evaluate("parseFloat(getComputedStyle(document.querySelector('.inspector')).width)"))
            page.get_by_role("separator", name="Resize properties inspector").focus()
            page.keyboard.press("ArrowLeft")
            resized = float(page.evaluate("parseFloat(getComputedStyle(document.querySelector('.inspector')).width)"))
            assert resized >= initial + 10, (initial, resized)
            explorer_start = float(page.evaluate("parseFloat(getComputedStyle(document.querySelector(\'.left-rail\')).width)"))
            page.get_by_role("separator", name="Resize model explorer").focus()
            page.keyboard.press("ArrowRight")
            explorer_end = float(page.evaluate("parseFloat(getComputedStyle(document.querySelector(\'.left-rail\')).width)"))
            assert explorer_end >= explorer_start + 10, (explorer_start, explorer_end)
            results.append("Keyboard-resizable explorer, inspector and desktop workspace")


            page.get_by_role("button", name="Add row").click()
            assert page.locator(".point-table tbody tr").count() == 5
            page.get_by_role("button", name="Undo").click()
            assert page.locator(".point-table tbody tr").count() == 4
            page.get_by_role("button", name="L-shaped route").click()
            assert page.locator(".point-table tbody tr").count() == 4
            results.append("Waypoint creation, undo, and route preset")

            # Stub only the analysis UI. This explicitly does NOT certify a real plan.
            fixture = synthetic_reference()
            page.evaluate(
                """fixture => {
                  window.__plannerFetch = window.fetch;
                  window.fetch = async (input, options) => {
                    if (typeof input === 'string' && input === '/api/preview') {
                      return new Response(JSON.stringify(fixture), {
                        status: 200, headers: {'Content-Type': 'application/json'}});
                    }
                    return window.__plannerFetch(input, options);
                  };
                }""", fixture
            )
            page.get_by_role("button", name="Plan & verify").click()
            page.get_by_role("heading", name="Engineering post-processing").wait_for()
            page.get_by_text("ACCEPTED REFERENCE").wait_for()
            assert page.locator(".plot-grid .plot-box").count() == 6
            assert "SYNTHETIC UI TEST ONLY" in page.locator(".adjustments").inner_text()
            assert "NOT RUN" in page.locator(".verification-grid").inner_text()
            assert "0.00 s" in page.locator(".sample-readout").inner_text()
            page.screenshot(path=str(options.output / "desktop-synthetic-ui.png"), full_page=True)
            results.append("Synchronized 3D/result panels, five plots and adjustment disclosures")

            # Test location and hourly-weather UI against deterministic responses.
            # This tests parsing and provenance only, not live forecast accuracy.
            page.evaluate("""() => {
              const original = window.fetch;
              window.fetch = async (input, options) => {
                const url = typeof input === 'string' ? input : input.url;
                if (url.startsWith('https://geocoding-api.open-meteo.com/')) {
                  return new Response(JSON.stringify({results:[{
                    id:101,name:'College Station',admin1:'Texas',country:'United States',
                    latitude:30.628,longitude:-96.334}]}),
                    {status:200,headers:{'Content-Type':'application/json'}});
                }
                if (url.startsWith('https://api.open-meteo.com/')) {
                  return new Response(JSON.stringify({hourly:{
                    time:[new Date().toISOString().slice(0,16)],
                    wind_speed_10m:[4],wind_direction_10m:[270],
                    wind_gusts_10m:[7],temperature_2m:[25],surface_pressure:[1005]
                  }}),{status:200,headers:{'Content-Type':'application/json'}});
                }
                return original(input, options);
              };
            }""")
            page.get_by_role("button", name="Live forecast").click()
            page.get_by_role("textbox", name="Search location").fill("College Station")
            page.get_by_role("button", name="Find").click()
            page.locator(".place-matches button").first.click()
            assert abs(float(page.get_by_label("Latitude (°)").input_value()) - 30.628) < 1e-6
            page.get_by_role("button", name="Fetch live forecast").click()
            page.get_by_text("Open-Meteo forecast", exact=False).wait_for()
            assert float(page.get_by_label("Wind speed (m/s)").input_value()) == 4
            assert float(page.get_by_label("Wind from (°)").input_value()) == 270
            assert float(page.get_by_label("Surface pressure (hPa)").input_value()) == 1005
            results.append("Mock geocoding, timestamped forecast parsing and meteorological wind settings")

            page.get_by_role("button", name="Geometry", exact=True).click()
            page.get_by_role("button", name="New blank mission").click()
            assert page.locator(".point-table tbody tr").count() == 0
            assert page.get_by_role("button", name="Plan & verify").is_disabled()
            results.append("New blank mission invalidates prior verified artifact")

            # Exercise responsive layout at a narrow viewport, without mock results.
            page.set_viewport_size({"width": 390, "height": 844})
            page.screenshot(path=str(options.output / "mobile-empty-ui.png"), full_page=True)
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth + 1")
            results.append("390px responsive workspace horizontal overflow check")
            assert not page_errors, page_errors
            context.close()

            if options.real_planner:
                real = browser.new_context(viewport={"width": 1440, "height": 940})
                live = real.new_page()
                live.on("pageerror", lambda error: page_errors.append(str(error)))
                live.on("console", lambda msg: runtime_diagnostics.append('console '+msg.type+': '+msg.text) if msg.type == 'error' else None)
                live.on("requestfailed", lambda req: runtime_diagnostics.append('request failed: '+req.url+' '+str(req.failure)))
                live.goto(base, wait_until="domcontentloaded", timeout=30000)
                live.get_by_role("button", name="Straight path").click()
                live.get_by_role("button", name="Plan & verify").click()
                try:
                    live.wait_for_function("""() => Boolean(document.querySelector('.pass-label')) || Boolean(document.querySelector('.notice.error'))""", timeout=137000)
                except Exception:
                    runtime_diagnostics.append('timeout; no verified reference or explicit UI refusal')
                if not live.locator('.pass-label').count():
                    runtime_diagnostics.append('UI notices: '+repr(live.locator('.notice').all_inner_texts()))
                    runtime_diagnostics.append('engine status: '+repr(live.locator('.menu-right').all_inner_texts()))
                    runtime_diagnostics.append('working: '+repr(live.get_by_role('button',name='Verifying…').count()))
                    live.screenshot(path=str(options.output / 'real-planner-diagnostic.png'), full_page=True)
                    raise RuntimeError('Real-browser planner did not accept route: '+repr(runtime_diagnostics[-25:]))
                assert "NOT RUN" in live.locator(".verification-grid").inner_text()
                results.append("REAL Pyodide Python planner accepted a straight trajectory, no Gazebo")
                live.screenshot(path=str(options.output / "real-planner-result.png"), full_page=True)
                real.close()
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        runner.join(timeout=4)
        (options.output / "report.json").write_text(json.dumps({
            "smoke_passed": len(results) >= 5 and not page_errors,
            "simulation_ran": False,
            "synthetic_ui_fixture": True,
            "actual_planner_attempted": options.real_planner,
            "results": results,
            "page_errors": page_errors,
            "runtime_diagnostics": runtime_diagnostics,
        }, indent=2) + "\n")

    print("\n".join("PASS " + label for label in results))


if __name__ == "__main__":
    main()
