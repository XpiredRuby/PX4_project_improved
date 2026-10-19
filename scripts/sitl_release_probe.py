"""SITL-only exercise of release hooks and the physical runtime guard.

No physical qualification is manufactured. RC presence is deliberately not
claimed by this fixture. Endpoint and /proc ownership are checked before use.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time

from mission_planner.acknowledgement import load_sitl_acknowledgement
from mission_planner.contracts import load_contract
from mission_planner.flight_preflight import FlightAuthority
from mission_planner.flight_release import digest, source_digest, write_json


class SITLReleaseProbe:
    check = FlightAuthority.check
    _check = FlightAuthority._check

    def __init__(self, folder, abort_after_s=None):
        self.root = Path(folder).resolve()
        self.summary = load_contract(self.root / 'plan/summary.json', dict)
        self.profile = load_contract(self.root / 'plan/aircraft_profile.json', dict)
        self.expected_digest = digest(self.root / 'plan/summary.json')
        self.source = source_digest()
        self.release = dict(release_class='SITL_ONLY',
                            expected_cruise_height_m=self.summary['controller_cruise_height_m'])
        self.lock = threading.RLock()
        self.error, self.started_at, self.last_check = None, None, -math.inf
        self.require_rc = False  # This simulation has no human RC qualification.
        self.abort_after_s = abort_after_s
        self.request = dict(launch=dict(minimum_battery_percent=95, minimum_pack_voltage_v=1.))
        self.confirmed_at = time.monotonic()
        self.validate_offline()

    def validate_offline(self):
        owned = load_contract(self.root / 'owned_process.json', dict)
        command = Path(f"/proc/{owned['pid']}/cmdline").read_bytes()
        if (b'/build/px4_sitl_default/bin/px4\x00' not in command
                or str(self.root / 'runtime').encode() not in command
                or os.getpgid(owned['pid']) != owned['pgid']):
            raise RuntimeError('SITL release probe requires the exact owned local simulator')
        if source_digest() != self.source or digest(self.root / 'plan/summary.json') != self.expected_digest:
            raise RuntimeError('SITL release fixture changed')
        load_sitl_acknowledgement(self.root / 'plan/sitl_acknowledgement.json',
                                 self.root / 'plan/summary.json', self.root / 'plan/trajectory.csv')

    def bind_controller(self, c):
        if c.connection_string != 'udp:127.0.0.1:14540':
            raise RuntimeError('SITL release probe only accepts its loopback endpoint')
        self.validate_offline()
        self.config = asdict(c.config)
        self.release['controller_config'] = self.config
        c.release_guard = lambda: self.step(c)

    def connected(self, c, audited_parameters):
        self.validate_offline()
        message = None
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            candidate = c.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=.2)
            if candidate is not None and c._message_is_from_target(candidate):
                message = candidate
                break
        if message is None:
            raise RuntimeError('SITL release fixture needs live GPS')
        self.request['site'] = dict(latitude_deg=message.lat/1e7, longitude_deg=message.lon/1e7,
                                    launch_radius_m=5., cleared_radius_m=110.)
        write_json(self.root / 'release_probe.json', dict(release_class='SITL_ONLY',
                   source_sha256=self.source, expected_cruise_height_m=self.release['expected_cruise_height_m'],
                   runtime_guard='battery, GPS site, energy time, abort',
                   physical_qualification=False, rc_qualification=False))

    def before_arm(self, c):
        self.validate_offline()
        if asdict(c.config) != self.config or time.monotonic()-self.confirmed_at > 300:
            raise RuntimeError('SITL release fixture configuration/confirmation changed')
        self.check(c, force=True)
        with (self.root / 'SITL_RELEASE_CONSUMED.json').open('x') as f:
            json.dump(dict(release_class='SITL_ONLY', consumed_at=datetime.now(timezone.utc).isoformat()), f)
        self.started_at = time.monotonic()

    def step(self, c):
        if (self.abort_after_s is not None and str(c.phase) == 'TRAJECTORY'
                and c.phase_clock_s >= self.abort_after_s):
            (self.root / 'ABORT').touch()
            write_json(self.root / 'abort_injection.json', dict(release_class='SITL_ONLY',
                       phase=str(c.phase), phase_clock_s=c.phase_clock_s,
                       observed_monotonic=time.monotonic()))
            self.abort_after_s = None
        self.check(c)
