"""Declared reference-simulator input matrix shared by offline and SITL checks."""
from copy import deepcopy
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def drawing(points, *, closed=False, speed=1., hard=(), exact=(), altitude=20.):
    raw = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
    raw["path"].update(closure="periodic" if closed else "open_stop")
    raw["path"]["points"] = [
        {"point_id": f"p{i}", "north_m": float(n), "east_m": float(e),
         "down_m": -altitude, "hard": i in hard, "exact": i in exact}
        for i, (n, e) in enumerate(points)]
    raw["motion"].update(target_speed_m_s=speed)
    return {"source": "drawing", "document": raw}


def acceptance_cases():
    """Each case declares its expected result before running the compiler."""
    circle = [(5*math.sin(i*math.pi/10), 5-5*math.cos(i*math.pi/10)) for i in range(21)]
    circle[-1] = (0., 0.)
    cases = {
        "straight": drawing([(0, 0), (12, 0)]),
        "l-turn": drawing([(0, 0), (8, 0), (8, 8)]),
        "tight-turn": drawing([(0, 0), (6, 0), (6, 1), (1, 1)]),
        "s-curve": drawing([(0, 0), (4, 3), (8, -3), (12, 0)]),
        "circle": drawing(circle, closed=True),
        "hard-stop": drawing([(0, 0), (6, 0), (6, 6)], hard=(1,)),
        "fly-through": drawing([(0, 0), (5, 2), (10, 0)], exact=(1,)),
        "slow-route": drawing([(0, 0), (12, 0)], speed=.5),
    }
    interview = json.loads((ROOT / "missions/interview_circle_example.json").read_text())
    interview.update(altitude_m=20., yaw_mode="fixed", fixed_yaw_deg=0.)
    cases["interview-circle"] = {"source": "interview", "document": interview}
    survey = deepcopy(interview)
    for key in ("circle", "loop_count"):
        survey.pop(key)
    survey.update(pattern="survey", survey={"length_m": 8., "width_m": 4., "lane_spacing_m": 4.})
    cases["survey"] = {"source": "interview", "document": survey}
    accepted = [(name, payload, True) for name, payload in cases.items()]
    invalid = []
    for name in ("deadline", "overspeed", "outside-volume", "zero-length", "conflicting-stop", "nonfinite"):
        payload = deepcopy(cases["straight"])
        doc = payload["document"]
        if name == "deadline":
            doc["motion"].update(time_mode="maximum", requested_duration_s=1.)
        elif name == "overspeed":
            doc["motion"].update(maximum_speed_m_s=10., target_speed_m_s=10.)
        elif name == "outside-volume":
            doc["path"]["points"][-1]["north_m"] = 100.
        elif name == "zero-length":
            doc["path"]["points"][-1]["north_m"] = 0.
        elif name == "conflicting-stop":
            doc["path"]["points"][0].update(hard=True, exact=True)
        else:
            doc["motion"]["maximum_speed_m_s"] = math.nan
        invalid.append(("refuse-" + name, payload, False))
    return accepted + invalid
