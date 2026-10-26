"""Best-effort browser preview: preserve geometry, stop/turn when smooth flight fails.

Aircraft/environment limits stay immutable. Changes to user preferences are
reported; acceptance refers to the effective intent, never a rewritten request.
The constructive fallback uses the independently bounded lifecycle polynomial.
"""
from __future__ import annotations

from copy import deepcopy
import csv
import hashlib
import io
import json
import math
from pathlib import Path

from .aircraft import assess_profile, mission_budget
from .contracts import (
    EnvironmentSnapshot, MissionIntent, VehicleModel, canonical_sha256,
    load_contract, validate_planning_inputs,
)
from .export import FIELDS
from .flatness import maximum_tilt_rad, thrust_interval_n
from .lifecycle import PERIOD, _evaluate, _phase, build_mission
from .plan_interview import intent_from_interview

ROOT = Path(__file__).resolve().parents[1]
MAX_ROUTE_ROWS = 20_000
MAX_CSV_BYTES = 12 * 1024 * 1024
MAX_SKETCH_VERTICES = 48


class PreviewBudgetError(ValueError):
    pass


def _finite(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(field + " needs a finite number")
    return float(value)


def _change(changes, code, message, **detail):
    changes.append(dict(code=code, message=message, **detail))


def _models(payload):
    setup = assess_profile(payload["aircraft_profile"]) if "aircraft_profile" in payload else None
    if setup is not None and not setup["ready"]:
        raise ValueError("Complete the aircraft setup before planning; its physical limits are missing.")
    vehicle = load_contract(ROOT / "config/planner/x500_sitl_vehicle_model.json", VehicleModel.from_dict)
    env = load_contract(ROOT / "config/planner/empty_world_environment.json", dict)
    if setup:
        p = setup["profile"]
        vehicle = VehicleModel.from_dict(setup["vehicle_model"])
        env.update(environment_id="profile-env-" + setup["profile_sha256"][:16],
                   max_radius_m=p["max_radius_m"], max_height_above_launch_m=p["max_height_m"],
                   max_wind_m_s=p["expected_wind_m_s"])
    return vehicle, EnvironmentSnapshot.from_dict(env), setup


def _motion(value, vehicle, changes):
    maximum = _finite(value["maximum_speed_m_s"], "Maximum speed")
    if maximum <= 0:
        raise ValueError("Maximum speed must be greater than zero")
    target = value.get("target_speed_m_s")
    chosen = min(maximum, vehicle.maximum_speed_m_s)
    if chosen != maximum:
        _change(changes, "speed_limit", f"Speed limit adjusted to {chosen:g} m/s for this aircraft.",
                requested=maximum, effective=chosen)
    value["maximum_speed_m_s"] = chosen
    if target is not None:
        target = _finite(target, "Cruising speed")
        if target <= 0:
            raise ValueError("Cruising speed must be greater than zero")
        effective = min(target, chosen)
        if effective != target:
            reason = "your Never exceed setting" if maximum < target and maximum <= vehicle.maximum_speed_m_s else "the aircraft model ceiling"
            _change(changes, "cruise_speed", f"Requested cruise {target:g} m/s is limited to {effective:g} m/s by {reason}, before the 10% planning margin.",
                    requested=target, effective=effective, reason=reason,
                    requested_cap_m_s=maximum, aircraft_cap_m_s=vehicle.maximum_speed_m_s)
        value["target_speed_m_s"] = effective
    if value.get("requested_duration_s") is not None:
        duration = _finite(value["requested_duration_s"], "Route time")
        if duration <= 0:
            value.update(time_mode="minimize", requested_duration_s=None)
            _change(changes, "route_time", "Automatic timing replaces the non-positive route time.")
    if value.get("yaw_mode") == "fixed":
        heading = _finite(value["fixed_yaw_deg"], "Heading")
        wrapped = (heading + 180) % 360 - 180
        if wrapped != heading:
            value["fixed_yaw_deg"] = wrapped
            _change(changes, "heading_wrap", "Heading expressed within one full turn.")
    if value.get("yaw_mode") == "unconstrained":
        value.update(yaw_mode="tangent", fixed_yaw_deg=None)
        _change(changes, "heading", "Heading follows the route instead of being unspecified.")


def _prepare(payload, vehicle, environment, changes):
    if (not isinstance(payload, dict) or set(payload) - {"source", "document", "aircraft_profile"}
            or not {"source", "document"} <= set(payload)):
        raise ValueError("preview requires source, document and optionally aircraft_profile")
    document = deepcopy(payload["document"])
    if not isinstance(document, dict):
        raise ValueError("Route input must be an object")
    source = payload["source"]
    if source == "interview":
        _motion(document, vehicle, changes)
        height = _finite(document["altitude_m"], "Flight height")
        bounded = min(max(height, .1), environment.max_height_above_launch_m)
        if height != bounded:
            document["altitude_m"] = bounded
            _change(changes, "height", f"Flight height adjusted to {bounded:g} m.",
                    requested=height, effective=bounded)
        deviation = _finite(document["maximum_deviation_m"], "Rounding")
        document["maximum_deviation_m"] = max(0., deviation)
        if deviation < 0:
            _change(changes, "rounding", "Negative rounding replaced with an exact-path target.")
        if document.get("pattern") == "waypoints":
            waypoints = document.get("waypoints")
            if not isinstance(waypoints, list) or not waypoints:
                raise ValueError("Enter at least one route point.")
            first = waypoints[0]
            if _finite(first["north_m"], "North") or _finite(first["east_m"], "East"):
                waypoints.insert(0, dict(north_m=0., east_m=0.))
                _change(changes, "launch_leg", "Added a leg from START to your first point.")
            if len(waypoints) == 1:
                waypoints.append(deepcopy(first))
        if document.get("pattern") == "circle":
            circle = document["circle"]
            if not _finite(circle["center_north_m"], "Circle center") and not _finite(circle["center_east_m"], "Circle center"):
                document.pop("circle")
                document.pop("loop_count", None)
                document.update(pattern="waypoints", waypoints=[dict(north_m=0., east_m=0.)]*2)
                _change(changes, "hover", "A zero-radius circle becomes a stationary hover.")
        # Bound circle discretization cost even for a zero fit preference.
        if document.get("pattern") == "circle":
            circle = document["circle"]
            radius = math.hypot(circle["center_north_m"],circle["center_east_m"])
            minimum = max(.01,4*radius*(1-math.cos(math.pi/120)))
            if document["maximum_deviation_m"] < minimum:
                document["maximum_deviation_m"] = minimum
                _change(changes,"circle_resolution","Used a finite circle approximation so a zero rounding request can still be planned.",
                        effective_tolerance_m=minimum)
            if 0 < radius < 1:
                if "entry" in document or "exit" in document:
                    raise ValueError("Tiny circle entry/exit needs explicit waypoints.")
                loops = document.pop("loop_count",1)
                if type(loops) is not int or not 1 <= loops <= 20:
                    raise ValueError("Circle laps must be an integer from 1 to 20.")
                document.pop("circle")
                angle = math.atan2(-circle["center_east_m"],-circle["center_north_m"])
                generated = [dict(north_m=circle["center_north_m"]+radius*math.cos(angle+math.tau*i/32),
                                  east_m=circle["center_east_m"]+radius*math.sin(angle+math.tau*i/32))
                             for i in range(33)]
                generated[0] = generated[-1] = dict(north_m=0.,east_m=0.)
                document.update(pattern="waypoints",waypoints=generated)
                intent, _ = intent_from_interview(document)
                intent["path"]["closure"] = "periodic"
                _change(changes,"tiny_circle","Kept the small circle and allowed stops if needed, instead of rejecting its radius.")
            else:
                intent, loops = intent_from_interview(document)
        else:
            intent, loops = intent_from_interview(document)
    elif source == "drawing":
        intent, loops = document, 1
        _motion(intent["motion"], vehicle, changes)
        deviation = _finite(intent["path"]["maximum_deviation_m"], "Rounding")
        intent["path"]["maximum_deviation_m"] = max(0., deviation)
        if deviation < 0:
            _change(changes, "rounding", "Negative rounding replaced with an exact-path target.")
        raw_points = intent["path"]["points"]
        if not isinstance(raw_points, list) or not raw_points:
            raise ValueError("Enter at least one route point.")
        if len(raw_points) == 1:
            raw_points.append(dict(raw_points[0], point_id="hover-end"))
        # A loop request explicitly asks for the missing closing leg.
        if intent["path"]["closure"] == "periodic":
            a, b = raw_points[0], raw_points[-1]
            if any(_finite(a[k], k) != _finite(b[k], k) for k in ("north_m","east_m","down_m")):
                raw_points.append(dict(a, point_id="closest-closure"))
                _change(changes, "closure", "Connected END back to START to close the loop.")
    else:
        raise ValueError("source must be drawing or interview")
    # Parse the complete strict schema before changing geometry or trying solvers.
    conversion_deviation = intent["path"]["maximum_deviation_m"]
    effective_motion = deepcopy(intent["motion"])
    requested = json.loads(json.dumps(MissionIntent.from_dict(intent).to_dict()))
    original_motion = payload["document"]["motion"] if source == "drawing" else payload["document"]
    # Keep original preferences separate from the effective, verified contract.
    # Repaired malformed values (e.g. wrapped heading) remain disclosed above.
    for key in ("maximum_speed_m_s", "target_speed_m_s"):
        requested["motion"][key] = original_motion.get(key)
    requested["path"]["maximum_deviation_m"] = max(0.,_finite(payload["document"].get("maximum_deviation_m", payload["document"].get("path",{}).get("maximum_deviation_m")),"Rounding"))
    if source == "interview" and height != bounded:
        for p in requested["path"]["points"]:
            p["down_m"] = -height
    intent = deepcopy(requested)
    intent["motion"] = effective_motion
    intent["path"]["maximum_deviation_m"] = conversion_deviation
    if source == "interview" and payload["document"].get("pattern") == "circle" and not any(k in payload["document"] for k in ("entry","exit")):
        circle = payload["document"]["circle"]
        radius = math.hypot(circle["center_north_m"],circle["center_east_m"])
        count = len(requested["path"]["points"])-1
        chord_bound = radius*(1-math.cos(math.pi/count)) if radius else 0.
        _change(changes,"circle_approximation",f"Circle approximation is within {chord_bound*100:.2f} cm of the requested shape.",source_shape_error_bound_m=chord_bound)
    moved = 0
    for point in intent["path"]["points"]:
        original = [point[k] for k in ("north_m","east_m","down_m")]
        radius = math.hypot(*original[:2])
        if radius > environment.max_radius_m:
            ratio = environment.max_radius_m * (1-1e-12) / radius
            point["north_m"] *= ratio
            point["east_m"] *= ratio
        point["down_m"] = -min(max(-point["down_m"], .1), environment.max_height_above_launch_m)
        moved += any(point[k] != n for k, n in zip(("north_m","east_m","down_m"), original, strict=True))
    if moved:
        _change(changes, "envelope_projection", f"Moved {moved} point(s) into the permitted height and area.",
                point_count=moved)
    first = intent["path"]["points"][0]
    if first["north_m"] or first["east_m"]:
        intent["path"]["points"].insert(0, dict(first, point_id="closest-launch", north_m=0., east_m=0., hard=False, exact=False))
        _change(changes, "launch_leg", "Added a leg from START to your first point.")
    return requested, intent, loops


def _repair_bound(original, effective):
    """Explicit monotone vertex coupling; also covers an inserted launch leg."""
    old, new = original["path"]["points"], effective["path"]["points"]
    if len(new) == len(old) + 1:
        new = new[1:]
        prefix = math.dist([old[0][k] for k in ("north_m","east_m","down_m")],
                           [effective["path"]["points"][0][k] for k in ("north_m","east_m","down_m")])
    else:
        prefix = 0.
    if len(new) != len(old):
        # Walk both polylines monotonically by normalized arc length. Between
        # the union of their vertex parameters, both curves are affine, so the
        # convex norm reaches its maximum at one of these breakpoints. This is
        # a conservative continuous, order-aware coupling, not nearest-point
        # sampling (which can miss a reordered self-crossing).
        def parameterize(points):
            poses = [[p[k] for k in ("north_m", "east_m", "down_m")] for p in points]
            lengths = [0.]
            for a, b in zip(poses, poses[1:], strict=False):
                lengths.append(lengths[-1] + math.dist(a, b))
            total = lengths[-1]
            return poses, [v / total if total else i / (len(poses)-1)
                           for i, v in enumerate(lengths)]
        a, ta = parameterize(old)
        b, tb = parameterize(new)
        def at(poses, times, t, cursor):
            while cursor < len(times)-2 and times[cursor+1] <= t:
                cursor += 1
            span = times[cursor+1] - times[cursor]
            u = (t-times[cursor])/span if span else 0.
            return [x + u*(y-x) for x, y in zip(poses[cursor], poses[cursor+1], strict=True)], cursor
        bound, ia, ib = prefix, 0, 0
        for t in sorted(set(ta + tb)):
            pa, ia = at(a, ta, t, ia)
            pb, ib = at(b, tb, t, ib)
            bound = max(bound, math.dist(pa, pb))
        return bound
    return max(prefix, *(math.dist([p[k] for k in ("north_m","east_m","down_m")],
                                  [q[k] for k in ("north_m","east_m","down_m")])
                         for p, q in zip(old, new, strict=True)))


def _simplify_sketch(intent, limit=MAX_SKETCH_VERTICES):
    """Keep ordered endpoints and the most significant bends, within a budget.

    Dense pointer events are not flight waypoints. Use arc-length coupling to
    select bends; retain explicit marks when they fit the budget. The untouched
    request remains the shadow, and _repair_bound certifies the actual change.
    """
    points = intent["path"]["points"]
    if len(points) <= 2:
        return deepcopy(intent)
    poses = [[p[k] for k in ("north_m", "east_m", "down_m")] for p in points]
    arc = [0.]
    for a, b in zip(poses, poses[1:], strict=False):
        arc.append(arc[-1] + math.dist(a, b))
    marked = {i for i, p in enumerate(points) if p["hard"] or p["exact"]}
    keep = {0, len(points)-1} | marked
    limit = max(limit, len(keep))
    target = max(.005, .25*intent["path"]["maximum_deviation_m"])
    def worst(left, right):
        span = arc[right]-arc[left]
        best = (0., None)
        for i in range(left+1, right):
            u = (arc[i]-arc[left])/span if span else 0.
            chord = [x+u*(y-x) for x, y in zip(poses[left], poses[right], strict=True)]
            error = math.dist(poses[i], chord)
            if error > best[0]:
                best = error, i
        return best
    import heapq
    queue = []
    def push(left, right):
        error, index = worst(left, right)
        if index is not None:
            heapq.heappush(queue, (-error, left, right, index))
    ordered = sorted(keep)
    for left, right in zip(ordered, ordered[1:], strict=False):
        push(left, right)
    while queue and len(keep) < limit:
        negative, left, right, index = heapq.heappop(queue)
        if -negative <= target:
            break
        keep.add(index)
        push(left, index)
        push(index, right)
    result = deepcopy(intent)
    result["path"]["points"] = [deepcopy(points[i]) for i in sorted(keep)]
    return result


def _attach(result, requested, effective, changes, method, reason=None):
    projection = _repair_bound(requested, effective)
    fit = result["summary"].get("fidelity_policy", {})
    curve = max((p["curve_bound_m"] for p in fit.get("pieces", [])), default=0.)
    check = next((c for c in result["verification"].get("checks",[]) if c.get("requirement_id") == "INT-007"),None)
    if check and check.get("status") == "PASS" and check.get("units") == "m":
        curve = max(curve,result["intent"]["path"]["maximum_deviation_m"]-check["margin"])
    shape_error = sum(c.get("source_shape_error_bound_m",0.) for c in changes)
    report = dict(
        source_shape_error_bound_m=shape_error,
        method=method, adjusted=bool(changes), adjustments=deepcopy(changes),
        requested_path_displacement_bound_m=projection,
        curve_error_bound_from_requested_m=projection + curve + shape_error,
        requested_rounding_m=requested["path"]["maximum_deviation_m"],
        original_rounding_met=projection + curve + shape_error <= requested["path"]["maximum_deviation_m"] + 1e-9,
        verification_scope="effective intent; original request remains separate",
        optimality="closest found in a bounded deterministic family; no global optimality claim",
        fallback_reason=reason,
    )
    result["requested_intent"] = requested
    result["planning_adjustments"] = report
    result["summary"]["planning_adjustments"] = report
    return result


def _polynomial(intent, loops, vehicle, environment, setup, *, hold_heading=False):
    """Exact ordered straight segments with C3 stop/turn joins and no yaw jumps."""
    contract = MissionIntent.from_dict(intent)
    validate_planning_inputs(contract, vehicle, environment)
    if environment.obstacle_map_verified:
        raise ValueError("This fallback requires verified segment clearance in a mapped environment.")
    source = intent["path"]["points"]
    ordered = source
    if intent["path"]["closure"] == "periodic":
        ordered = source + [p for _ in range(loops-1) for p in source[1:]]
    points = []
    for p in ordered:
        pose = [p[k] for k in ("north_m","east_m","down_m")]
        if not points or math.dist(pose, points[-1]) > 1e-10:
            points.append(pose)
    heading_mode = intent["motion"]["yaw_mode"]
    fixed = heading_mode == "fixed" or hold_heading
    heading = math.radians(intent["motion"]["fixed_yaw_deg"] or 0.) if fixed else 0.
    speed = .9 * min(intent["motion"]["maximum_speed_m_s"],
                      intent["motion"]["target_speed_m_s"] or vehicle.maximum_speed_m_s,
                      vehicle.maximum_speed_m_s)
    if setup and any(a[2] != b[2] for a, b in zip(points, points[1:], strict=False)):
        speed = min(speed, .9*min(setup["profile"]["maximum_climb_m_s"], setup["profile"]["maximum_descent_m_s"]))
    phases, row_estimate = [], 0
    pose = points[0] + [heading]

    def add(name, start, finish):
        nonlocal row_estimate
        phase = _phase(name, start, finish, speed, vehicle, environment)
        row_estimate += math.ceil(phase["duration_s"]/PERIOD)
        if row_estimate > MAX_ROUTE_ROWS:
            raise PreviewBudgetError("Route sample budget requires a smaller preview.")
        if not phase["verification"]["accepted"]:
            raise ValueError("Polynomial phase failed its independent physical checks.")
        phases.append(phase)

    for point in points[1:]:
        dx, dy = point[0]-pose[0], point[1]-pose[1]
        desired = pose[3]
        if not fixed and math.hypot(dx, dy) > 1e-10:
            raw = math.atan2(dy, dx)
            desired += math.atan2(math.sin(raw-desired), math.cos(raw-desired))
        if abs(desired-pose[3]) > 1e-9:
            turned = pose[:3] + [desired]
            add("TURN", pose, turned)
            pose = turned
        finish = point + [pose[3]]
        add("LEG", pose, finish)
        pose = finish
    if not phases:
        add("HOLD", pose, pose)
    rows, clock, spans = [], 0., []
    for phase in phases:
        start, finish = clock, clock + phase["duration_s"]
        if rows and any(abs(rows[-1][key]-v) > 1e-9 for key,v in zip(("x","y","z","yaw"), phase["start"], strict=True)):
            raise ValueError("Discontinuous polynomial phase join")
        count = math.ceil(phase["duration_s"]/PERIOD)
        for i in range(count+1):
            if rows and i == 0:
                continue
            row = _evaluate(phase["start"], phase["end"], phase["duration_s"], phase["duration_s"]*i/count)
            row.update(time=clock + phase["duration_s"]*i/count, planner_contract_version=1)
            rows.append(row)
        spans.append(dict(name=phase["name"], start_time_s=start, end_time_s=finish))
        clock = finish
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    csv_text = stream.getvalue()
    serialized = list(csv.DictReader(csv_text.splitlines()))
    if len(serialized) != len(rows) or any(
        float(raw[k]) != row[k] or not math.isfinite(float(raw[k]))
        for raw,row in zip(serialized,rows, strict=True) for k in FIELDS
    ):
        raise ValueError("Polynomial CSV serialization audit failed")
    if any(b["time"] <= a["time"] for a,b in zip(rows,rows[1:], strict=False)):
        raise ValueError("Polynomial route clocks must increase")
    digest = hashlib.sha256(csv_text.encode()).hexdigest()
    full = build_mission(csv_text, contract, vehicle, environment,
                         None if setup is None else setup["profile"])
    full["verification"]["route"] = "independent continuous Bernstein checks of every stop/turn polynomial"
    for phase in full["phases"]:
        if phase["name"] == "ROUTE":
            phase["verification_source"] = "verified stop/turn polynomials and audited CSV SHA-256"
    if len(csv_text.encode()) + len(full["trajectory_csv"].encode()) > MAX_CSV_BYTES:
        raise PreviewBudgetError("Route exports exceed the browser preview budget")
    budget = error_budget = None
    if setup:
        p = setup["profile"]
        try:
            budget = mission_budget(setup, clock, -points[0][2], -points[-1][2])
        except ValueError as exc:
            raise PreviewBudgetError(str(exc)) from exc
        lifecycle = full["total_duration_s"] - clock
        required = clock*p["route_time_multiplier"] + max(lifecycle+p["settle_time_s"],p["lifecycle_time_upper_bound_s"]) + p["contingency_time_s"]
        full["energy_budget"] = dict(total_budgeted_s=required, allowed_flight_s=budget["allowed_flight_s"],
            upper_bound_energy_wh=required*p["power_upper_bound_w"]/3600.,
            reserve_fraction=p["reserve_fraction"], passed=required <= budget["allowed_flight_s"])
        if not full["energy_budget"]["passed"]:
            raise PreviewBudgetError("Mission needs a shorter route or lower height to fit the energy budget")
        allowance = p["position_uncertainty_m"] + p["tracking_error_m"]
        if intent["path"]["maximum_deviation_m"] < allowance:
            raise ValueError("Planning allowance does not cover declared navigation/tracking uncertainty")
        error_budget = dict(total_tolerance_m=intent["path"]["maximum_deviation_m"],
            planning_tolerance_m=intent["path"]["maximum_deviation_m"]-allowance,
            position_uncertainty_m=p["position_uncertainty_m"], tracking_error_m=p["tracking_error_m"])
    phase_checks = {name:all(p["verification"]["checks"][name] for p in phases)
                    for name in phases[0]["verification"]["checks"]}
    if not all(phase_checks.values()):
        raise ValueError("Polynomial route failed continuous verification")
    verification = dict(
        schema_version=1, accepted=True, release_class="SITL_ONLY",
        method="stop_turn_septic_bernstein",
        checks=[dict(requirement_id="INT-007",status="PASS",units="m",
                     margin=intent["path"]["maximum_deviation_m"],detail="Exact ordered segment geometry; polynomial progress is monotone."),
                dict(requirement_id="ORDER",status="PASS",detail="Vertices and laps visited in order; stationary turns add no geometric deviation."),
                dict(requirement_id="DYNAMICS",status="PASS",detail="Every polynomial has continuous Bernstein limits and rotor-authority checks."),
                dict(requirement_id="JOINS",status="PASS",detail="Position/yaw continuous; velocity, acceleration and jerk zero at every join."),
                dict(requirement_id="SERIALIZATION",status="PASS",detail="All serialized samples and fields round-trip exactly.")],
        continuous_phase_checks=phase_checks, phases=phases,
        controller_trajectory_sha256=digest,
    )
    bounds = [p["verification"] for p in phases]
    max_a = max(b["acceleration_bound_m_s2"] for b in bounds)
    thrust = thrust_interval_n(max_a+vehicle.disturbance_acceleration_m_s2,vehicle.mass_kg)
    retiming = dict(selected_duration_s=clock, minimum_feasible_duration_s=None,
        limiting_constraint="stop_and_turn", speed_limit_m_s=speed,
        conservative_speed_bound_m_s=max(b["speed_bound_m_s"] for b in bounds),
        conservative_acceleration_bound_m_s2=max_a,
        conservative_jerk_bound_m_s3=max(b["jerk_bound_m_s3"] for b in bounds),
        tilt_bound_deg=math.degrees(maximum_tilt_rad(max_a+vehicle.disturbance_acceleration_m_s2)),
        minimum_thrust_bound_n=thrust[0], maximum_thrust_bound_n=thrust[1],
        loop_count=loops, segment_count=len(phases), yaw_mode="fixed" if fixed else "tangent",
        limit_scale=.9, minimum_duration_scope="chosen polynomial family; not a time lower bound")
    summary = dict(schema_version=1, release_class="SITL_ONLY", retiming=retiming,
        fidelity_policy=dict(hard_limit_m=intent["path"]["maximum_deviation_m"],soft_target_fraction=.25,
            automatic_stops_added=True, pieces=[dict(soft_target_m=.25*intent["path"]["maximum_deviation_m"],
                curve_bound_m=0.,soft_target_met=True,candidates_evaluated=1,fallback="stop_and_turn")]),
        controller_trajectory_sha256=digest, candidate_trajectory_sha256=canonical_sha256(phases),
        mission_intent_sha256=canonical_sha256(contract), vehicle_model_sha256=canonical_sha256(vehicle),
        environment_sha256=canonical_sha256(environment), verification_report_sha256=canonical_sha256(verification),
        point_count=len(rows), controller_cruise_height_m=-points[0][2],
        route_phases=spans, duration_scope="verified constructive trajectory; global optimality not established")
    stride = max(1,math.ceil((len(rows)-1)/1000))
    indices = {0,len(rows)-1,*range(0,len(rows),stride)}
    # Keep exact stops/turns visible in the plotted path.
    for span in spans:
        indices.add(min(range(len(rows)), key=lambda i:abs(rows[i]["time"]-span["end_time_s"])))
    return dict(accepted=True,release_class="SITL_ONLY",campaign_ran=False,simulation_status="NOT_RUN",
        flight_approval=False,simulation_compatible=False,
        intent=contract.to_dict(),vehicle_model=vehicle.to_dict(),environment=environment.to_dict(),
        aircraft_setup=setup,mission_budget=budget,path_error_budget=error_budget,
        summary=summary,verification=verification,samples=[rows[i] for i in sorted(indices)],
        display_samples_decimated=stride>1,trajectory_csv=csv_text,mission=full,
        preview_settings=dict(sample_period_s=PERIOD,limit_scale=.9,planner_backend="stop_turn"))


def closest_preview(payload, strict_preview):
    if not isinstance(payload,dict) or set(payload)-{"source","document","aircraft_profile"}:
        raise ValueError("Unknown preview fields")
    vehicle, environment, setup = _models(payload)
    changes = []
    requested, effective, loops = _prepare(payload,vehicle,environment,changes)
    if len(effective["path"]["points"]) > 32:
        before = len(effective["path"]["points"])
        effective = _simplify_sketch(effective)
        after = len(effective["path"]["points"])
        if after < before:
            _change(changes, "sketch_simplification",
                    f"Kept the main shape using {after} flight points from {before} drawing samples. The original drawing remains visible for comparison.",
                    original_point_count=before, effective_point_count=after)
    base = dict(source="drawing",document=effective)
    if setup:
        base["aircraft_profile"] = payload["aircraft_profile"]
    coordinates = [[p[k] for k in ("north_m","east_m","down_m")] for p in effective["path"]["points"]]
    length = sum(math.dist(a,b) for a,b in zip(coordinates,coordinates[1:], strict=False))
    reversal = False
    for a,b,d in zip(coordinates,coordinates[1:],coordinates[2:], strict=False):
        left, right = [b[i]-a[i] for i in range(3)],[d[i]-b[i] for i in range(3)]
        magnitude = math.hypot(*left)*math.hypot(*right)
        if magnitude > 1e-16 and sum(x*y for x,y in zip(left,right, strict=True))/magnitude < -.95:
            reversal = True
    quick = (len(coordinates)>8 or length>120 or reversal
             or any(c["code"] == "sketch_simplification" for c in changes)
             or effective["path"]["maximum_deviation_m"] == 0)
    reason = "Direct stop/turn construction preserves this difficult drawing without an expensive spline search." if quick else None
    try:
        if quick:
            raise ValueError(reason)
        result = strict_preview(base, loops_override=loops)
        if result.get("accepted"):
            pieces = result["summary"]["fidelity_policy"]["pieces"]
            if all(p["soft_target_met"] for p in pieces):
                return _attach(result,requested,effective,changes,"smooth_spline")
            reason = "The smooth fit missed the closer-fit target."
        else:
            reason = result.get("error","Smooth planning did not accept this route.")
    except (ValueError,RuntimeError,ArithmeticError) as exc:
        reason = str(exc)
    # Relax the time preference before changing spatial geometry.
    if effective["motion"]["requested_duration_s"] is not None:
        effective["motion"].update(time_mode="minimize",requested_duration_s=None)
        _change(changes,"route_time","Allowed more route time to stay within physical limits.")
        try:
            if quick:
                raise ValueError("Use the constructive fallback for this route.")
            result = strict_preview(dict(base,document=effective),loops_override=loops)
            if result.get("accepted") and all(p["soft_target_met"] for p in result["summary"]["fidelity_policy"]["pieces"]):
                return _attach(result,requested,effective,changes,"retimed_spline",reason)
        except (ValueError,RuntimeError,ArithmeticError):
            pass
    # Full stops make exact polyline corners feasible without cutting them.
    if effective["motion"]["yaw_mode"] not in {"tangent","fixed"}:
        effective["motion"].update(yaw_mode="tangent",fixed_yaw_deg=None,
            look_at_north_m=None,look_at_east_m=None,look_at_down_m=None,yaw_profile=None)
        _change(changes,"heading","Heading now follows each leg, with a smooth turn while stopped.")
    if effective["path"]["closure"] == "open_depart":
        effective["path"]["closure"] = "open_stop"
        _change(changes,"end_stop","Added a final stop so the mission can return and land.")
    # This planner is exact in space, so a large requested rounding tube must
    # not make a low-altitude route fail the separate full-mission composer.
    floor = min(-p["down_m"] for p in effective["path"]["points"])
    if effective["path"]["maximum_deviation_m"] >= floor:
        effective["path"]["maximum_deviation_m"] = floor/2
        _change(changes,"rounding","Used a tighter rounding allowance to keep the route above ground.")
    if setup:
        p = setup["profile"]
        allowance = p["position_uncertainty_m"]+p["tracking_error_m"]
        if effective["path"]["maximum_deviation_m"] <= allowance:
            effective["path"]["maximum_deviation_m"] = allowance+.01
            _change(changes,"rounding","Increased the requested tolerance to cover the declared navigation and tracking uncertainty.")
    required_height = effective["path"]["maximum_deviation_m"]+.01
    if setup and any(-p["down_m"] <= required_height for p in effective["path"]["points"]):
        if required_height > environment.max_height_above_launch_m:
            raise ValueError("The declared uncertainty cannot fit this model's height envelope.")
        for p in effective["path"]["points"]:
            p["down_m"] = -max(-p["down_m"],required_height)
        _change(changes,"height_clearance","Raised low points enough to cover the declared uncertainty above ground.")
    stopped = []
    for p in effective["path"]["points"][1:-1]:
        if not p["hard"] or p["exact"]:
            stopped.append(p["point_id"])
        p.update(hard=True, exact=False)
    _change(changes,"stop_and_turn","Added full stops at corners and smooth turns before continuing; your point order is preserved.", point_ids=stopped)
    for hold in (False,True):
        candidate = deepcopy(effective)
        if hold and candidate["motion"]["yaw_mode"] != "fixed":
            candidate["motion"].update(yaw_mode="fixed",fixed_yaw_deg=0.)
        try:
            result = _polynomial(candidate,loops,vehicle,environment,setup,hold_heading=hold)
            if hold:
                _change(changes,"heading","Held the starting heading to preserve the full route within the preview budget.")
            return _attach(result,requested,candidate,changes,"stop_and_turn",reason)
        except PreviewBudgetError:
            continue
    # Resource/energy-limited fallback keeps every vertex and lap in order.
    # Shrinking is explicit, and its displacement certificate covers the
    # original request. Aircraft limits and battery evidence never change.
    original_effective = deepcopy(effective)

    def scaled(factor):
        candidate = deepcopy(original_effective)
        for p in candidate["path"]["points"]:
            p["north_m"] *= factor
            p["east_m"] *= factor
            if setup:
                p["down_m"] = -max(.1,candidate["path"]["maximum_deviation_m"]+.01,-p["down_m"]*factor)
        candidate["motion"].update(yaw_mode="fixed",fixed_yaw_deg=candidate["motion"]["fixed_yaw_deg"] or 0.)
        return candidate

    for factor in (.5,.25,.125,.0625,.03125,.015625):
        effective = scaled(factor)
        try:
            result = _polynomial(effective,loops,vehicle,environment,setup,hold_heading=True)
            # Refine the largest feasible scale in this family instead of
            # stopping at the first coarse power-of-two shrink.
            low, high = factor, min(1.,factor*2)
            for _ in range(6):
                middle = (low+high)/2
                candidate = scaled(middle)
                try:
                    refined = _polynomial(candidate,loops,vehicle,environment,setup,hold_heading=True)
                except PreviewBudgetError:
                    high = middle
                else:
                    low, result, effective = middle, refined, candidate
            factor = low
            _change(changes,"route_scale",f"Reduced the route to {factor*100:.1f}% of its size to fit the preview or energy budget.",
                    scale=factor)
            _change(changes,"heading","Held heading while following the adjusted route.")
            return _attach(result,requested,effective,changes,"scaled_stop_and_turn",reason)
        except PreviewBudgetError:
            continue
    raise ValueError("No verified preview fits this model and its energy/output budget. The model limits were not relaxed.")
