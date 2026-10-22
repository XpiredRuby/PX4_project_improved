from pathlib import Path
import argparse
import os
import tempfile
import json
import hashlib
from playwright.sync_api import sync_playwright

parser = argparse.ArgumentParser(
    description="Check saved-session recovery against the local trajectory studio"
)
parser.add_argument("--output", type=Path, required=True)
base = parser.parse_args().output.resolve()
base.mkdir(parents=True, exist_ok=False)
temp = base / "temp"
temp.mkdir()
tempfile.tempdir = str(temp)
os.environ["TEMP"] = str(temp)
os.environ["TMP"] = str(temp)
errors = []
with sync_playwright() as p:
    c = p.chromium.launch_persistent_context(
        str(base / "profile"),
        executable_path=r"C:\Program Files\Google\Chrome\Application\chrome.exe",
        headless=True,
        viewport={"width": 1440, "height": 1100},
    )
    page = c.new_page()
    page.on("pageerror", lambda e: errors.append(str(e)))
    stale = "0" * 32
    assert (
        page.request.get("http://127.0.0.1:8766/api/rehearsal/" + stale).status == 404
    )
    page.add_init_script(
        "localStorage.setItem('px4-studio-active-job','" + stale + "')"
    )
    page.goto("http://127.0.0.1:8766", wait_until="networkidle")
    page.locator("#simulationStatus").filter(
        has_text="Previous rehearsal is unavailable"
    ).wait_for()
    assert page.evaluate("localStorage.getItem('px4-studio-active-job')===null")
    assert page.locator("#aircraftMode").is_visible()
    assert page.locator("#aircraftMode").input_value() == "demo"
    assert page.locator("#aircraftControls").is_hidden()
    page.locator("#linePreset").click()
    with page.expect_response(
        lambda r: r.url.endswith("/api/preview"), timeout=125000
    ) as response:
        page.locator("#compile").click()
    result = response.value.json()
    assert result["accepted"]
    page.locator("#results").wait_for(state="visible")
    assert page.locator("#rehearse").is_disabled()
    page.locator("#confirmPlan").check()
    assert page.locator("#rehearse").is_enabled()
    assert not errors, errors
    page.screenshot(path=str(base / "recovered-desktop.png"), full_page=True)
    html = page.request.get("http://127.0.0.1:8766").body()
    report = {
        "stage": "PASSED",
        "checks": [
            "unavailable_saved_rehearsal_clears_monitoring_state",
            "reference_model_visible_without_questionnaire",
            "new_plan_and_exact_confirmation_available_after_recovery",
        ],
        "page_errors": errors,
        "html_sha256": hashlib.sha256(html).hexdigest(),
        "simulation_started": False,
        "implementation_sha256": hashlib.sha256(
            Path(__file__).read_bytes()
        ).hexdigest(),
    }
    (base / "state.json").write_text(json.dumps(report, indent=2))
    c.close()
print(json.dumps(report))
