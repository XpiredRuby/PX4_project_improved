"""Read-only live qualification checks and per-command physical flight guard."""
from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import math
from pathlib import Path
import threading
import time

from .contracts import load_contract
from .flight_release import (digest, local_file, source_digest, timestamp,
                             validate_request, write_json)


def distance_m(lat, lon, site):
    lat1, lat2 = math.radians(lat), math.radians(site["latitude_deg"])
    dl = math.radians(lon - site["longitude_deg"])
    a = math.sin((lat1-lat2)/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dl/2)**2
    return 6378137. * 2 * math.asin(min(1., math.sqrt(max(0., a))))


class FlightAuthority:
    """One-use local authorization, bound to reviewed bytes and live PX4.

    Hashes prevent accidental substitution; the trusted local operator and
    qualification reviewers remain the authority. UID2 is not authentication.
    """
    def __init__(self, path, expected_digest, operator, mentor, mentor_present, connection):
        self.path, self.root = Path(path).resolve(), Path(path).resolve().parent
        self.expected_digest = expected_digest
        self.operator, self.mentor, self.connection = operator, mentor, connection
        if mentor_present is not True:
            raise ValueError("the owner requires the PhD mentor present for physical operations")
        self.release = load_contract(self.path, dict)
        self.request = load_contract(self.root / "qualification.json", dict)
        self.lock = threading.RLock()
        self.started_at = None
        self.last_check = -math.inf
        self.error = None
        self.validate_offline()
        if (operator != self.request["launch"]["operator"]
                or mentor != self.request["launch"]["mentor"]):
            raise ValueError("launch operator/mentor differ from qualification")
        if not isinstance(connection, str) or not connection or len(connection) > 256:
            raise ValueError("explicit MAVLink connection is required")
        self.profile = load_contract(self.root / "plan/aircraft_profile.json", dict)
        self.confirmed_at = time.monotonic()

    def validate_offline(self):
        if (self.release.get("schema_version") != 1
                or self.release.get("release_class") != "PHYSICAL_FLIGHT_PREPARED"
                or digest(self.path) != self.expected_digest):
            raise ValueError("explicit confirmation does not match a prepared physical release")
        if self.release.get("source_sha256") != source_digest():
            raise ValueError("software changed after release preparation")
        for name, expected in self.release["files_sha256"].items():
            if digest(local_file(self.root, name)) != expected:
                raise ValueError(f"prepared evidence changed: {name}")
        parameters = validate_request(self.request, self.root)
        if parameters != self.release["expected_px4_parameters"]:
            raise ValueError("prepared parameters do not match qualification")
        if datetime.now(timezone.utc).timestamp() >= timestamp(self.release["expires_at"]):
            raise ValueError("physical release expired")
        if (self.root / "CONSUMED.json").exists():
            raise ValueError("release has already been consumed; prepare a fresh reviewed launch")
        if (self.root / "ABORT").exists():
            raise ValueError("release has an abort request")

    def bind_controller(self, controller):
        self.validate_offline()
        if (asdict(controller.config) != self.release["controller_config"]
                or controller.connection_string != self.connection
                or controller.rotor_count != self.release["rotor_count"]
                or digest(controller.trajectory_path) != digest(self.root / "plan/trajectory.csv")):
            raise ValueError("runtime controller differs from reviewed release")
        controller.propulsion_output_indices = self.request["motor_output_indices"]
        controller.propulsion_disarmed_values = self.request["motor_disarmed_values"]
        controller.propulsion_stop_tolerance = self.request["motor_stopped_tolerance"]
        self.require_rc = True
        controller.release_guard = lambda: self.check(controller)

    def connected(self, controller, audited_parameters):
        """Runs before receiver thread; never sets parameters, modes or arm."""
        from pymavlink import mavutil
        from px4_policy import read_px4_parameters
        self.validate_offline()
        expected = self.request["aircraft"]
        master = controller.master
        if (master.target_system != expected["target_system"]
                or master.target_component != expected["target_component"]):
            raise RuntimeError("connected PX4 system/component is not the qualified aircraft")
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                     mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                                     0, 0, 0, 0, 0, 0)
        deadline, version = time.monotonic() + 5., None
        while time.monotonic() < deadline:
            message = master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=.2)
            if message is not None and controller._message_is_from_target(message):
                version = message
                break
        if version is None:
            raise RuntimeError("qualified firmware identity was not received")
        actual = dict(uid2_hex=bytes(getattr(version, "uid2", [])).hex(),
                      flight_sw_version=int(version.flight_sw_version),
                      flight_custom_version_hex=bytes(version.flight_custom_version).hex(),
                      board_version=int(version.board_version))
        if any(actual[k] != expected[k] for k in actual):
            raise RuntimeError("live aircraft UID2/firmware/board differs from qualification")
        capabilities = int(version.capabilities)
        needed = (mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
                  | mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED)
        if capabilities & needed != needed:
            raise RuntimeError("PX4 does not advertise the required Offboard command capabilities")
        approved = self.release["expected_px4_parameters"]
        actual_parameters = read_px4_parameters(controller, sorted(approved))
        for name, expected_value in approved.items():
            if not math.isclose(actual_parameters[name], expected_value, rel_tol=1e-6, abs_tol=1e-6):
                raise RuntimeError(f"live PX4 parameter differs from qualification: {name}")
        if any(not math.isclose(actual_parameters[k], v, rel_tol=1e-6, abs_tol=1e-6)
               for k, v in audited_parameters.items()):
            raise RuntimeError("PX4 safety configuration changed during preflight audit")
        write_json(self.root / "live_preflight.json", dict(aircraft=actual,
                   parameters=actual_parameters, checked_at=datetime.now(timezone.utc).isoformat(),
                   release_sha256=self.expected_digest, connection=self.connection))

    def before_arm(self, controller):
        self.validate_offline()
        if not 0 <= time.monotonic() - self.confirmed_at <= 300:
            raise RuntimeError("local launch confirmation expired after five minutes")
        self.check(controller, force=True)
        from offboard_runner import heartbeat_snapshot, landing_snapshot, propulsion_stopped
        landed, age, _ = landing_snapshot(controller)
        if (heartbeat_snapshot(controller)[2] is not False or landed != 1 or age > 1.5
                or not propulsion_stopped(controller)):
            raise RuntimeError("fresh disarmed ground/stopped propulsion evidence missing immediately before arm")
        # Exclusive creation makes concurrent/replayed launch attempts fail.
        with (self.root / "CONSUMED.json").open("x", encoding="utf-8") as handle:
            import json
            json.dump(dict(release_sha256=self.expected_digest, operator=self.operator,
                           mentor=self.mentor, consumed_at=datetime.now(timezone.utc).isoformat()), handle)
        self.started_at = time.monotonic()

    def check(self, controller, *, force=False):
        """Cheap cached guard runs before attitude/velocity sends and runner polls.

        Once airborne, expiry is not a command to stop; aborts use existing LAND
        cleanup. Pilot mode takeover is respected by that cleanup.
        """
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            now = time.monotonic()
            if not force and now - self.last_check < .1:
                return
            try:
                self._check(controller, now)
            except Exception as exc:
                self.error = str(exc)
                raise
            self.last_check = now

    def _check(self, controller, now):
        if (self.root / "ABORT").exists():
            raise RuntimeError("operator requested abort; yielding to LAND cleanup")
        p = self.profile
        with controller.state_lock:
            state = controller.state
            battery_at, remaining, voltage = (state.battery_received_at,
                                              state.battery_remaining_percent,
                                              state.battery_voltage_v)
            rc_at, channels, rssi = state.rc_received_at, state.rc_channel_count, state.rc_rssi
            gps_at, lat, lon = state.gps_received_at, state.gps_lat_deg, state.gps_lon_deg
            armed = state.armed
        timestamps = [("battery", battery_at), ("GPS", gps_at)]
        if self.require_rc:
            timestamps.append(("manual control", rc_at))
        for label, at in timestamps:
            if at is None or not 0 <= now - at <= 1.5:
                raise RuntimeError(f"physical flight {label} telemetry missing or stale")
        if self.require_rc and not (channels >= 4 and 0 < rssi < 255):
            raise RuntimeError("fresh RC link with known nonzero RSSI is required")
        contingency = p["contingency_time_s"] * p["power_upper_bound_w"] / (36. * p["usable_battery_wh"])
        floor = 100 * p["reserve_fraction"] + contingency
        if self.started_at is None:
            floor = max(floor, self.request["launch"]["minimum_battery_percent"])
            if armed:
                raise RuntimeError("physical release requires an initially disarmed vehicle")
        if (not math.isfinite(remaining) or not floor <= remaining <= 100
                or not math.isfinite(voltage) or voltage < self.request["launch"]["minimum_pack_voltage_v"]):
            raise RuntimeError("qualified battery reserve/voltage is insufficient")
        site = self.request["site"]
        if not all(math.isfinite(v) for v in (lat, lon)):
            raise RuntimeError("launch GPS coordinates are unavailable")
        limit = site["launch_radius_m"] if self.started_at is None else site["cleared_radius_m"] - p["position_uncertainty_m"]
        if distance_m(lat, lon, site) > limit:
            raise RuntimeError("aircraft is outside the qualified site boundary")
        if self.started_at is not None:
            allowed = p["usable_battery_wh"] * 3600 * (1-p["reserve_fraction"]) / p["power_upper_bound_w"]
            if now - self.started_at >= allowed - p["contingency_time_s"]:
                raise RuntimeError("qualified flight time exhausted; using contingency for LAND")
