"""Strict, deterministic contracts for trajectory-planning evidence.

These contracts deliberately contain no optimizer or flight-control behavior.
They define the immutable boundary between intent, configuration, verification,
simulation evidence, and execution.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Callable, ClassVar, TypeVar


CONTRACT_VERSION = 1
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
OBJECTIVE_PRIORITY = (
    "legal_and_safety",
    "vehicle_and_environment",
    "intent_fidelity",
    "tracking_margin_and_energy",
    "duration",
)
ContractT = TypeVar("ContractT")


def _object(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be an object")
    return value


def _array(value: Any, field: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{field} must be an array")
    return value


def _strict(
    value: dict[str, Any],
    *,
    required: set[str],
    optional: set[str] = frozenset(),
    field: str,
) -> None:
    unknown = sorted(set(value) - required - optional)
    missing = sorted(required - set(value))
    if unknown:
        raise ValueError(f"{field} has unknown fields: {', '.join(unknown)}")
    if missing:
        raise ValueError(f"{field} missing fields: {', '.join(missing)}")


def _identifier(value: Any, field: str) -> str:
    if not isinstance(value, str) or not IDENTIFIER_PATTERN.fullmatch(value):
        raise ValueError(f"{field} must be a stable identifier")
    return value


def _text(value: Any, field: str, *, maximum: int = 256) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > maximum:
        raise ValueError(f"{field} must be non-empty and at most {maximum} characters")
    return value.strip()


def _choice(value: Any, allowed: set[str], field: str) -> str:
    if not isinstance(value, str) or value not in allowed:
        raise ValueError(f"{field} must be one of {', '.join(sorted(allowed))}")
    return value


def _number(
    value: Any,
    field: str,
    *,
    minimum: float | None = None,
    exclusive_minimum: float | None = None,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    if minimum is not None and result < minimum:
        raise ValueError(f"{field} must be at least {minimum}")
    if exclusive_minimum is not None and result <= exclusive_minimum:
        raise ValueError(f"{field} must be greater than {exclusive_minimum}")
    return result


def _optional_number(
    value: Any,
    field: str,
    *,
    minimum: float | None = None,
    exclusive_minimum: float | None = None,
) -> float | None:
    if value is None:
        return None
    return _number(
        value,
        field,
        minimum=minimum,
        exclusive_minimum=exclusive_minimum,
    )


def _boolean(value: Any, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be boolean")
    return value


def _version(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field} must be integer {CONTRACT_VERSION}")
    if value != CONTRACT_VERSION:
        raise ValueError(f"{field} must be {CONTRACT_VERSION}")
    return value


def _digest(value: Any, field: str) -> str:
    if not isinstance(value, str) or not SHA256_PATTERN.fullmatch(value):
        raise ValueError(f"{field} must be a lowercase SHA-256 digest")
    return value


def canonical_json_bytes(value: Any) -> bytes:
    """Return deterministic UTF-8 JSON and reject non-finite numbers."""
    if hasattr(value, "to_dict"):
        value = value.to_dict()
    elif hasattr(value, "__dataclass_fields__"):
        value = asdict(value)
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def load_contract(
    path: str | Path,
    parser: Callable[[Any], ContractT],
) -> ContractT:
    """Load strict JSON without Python's non-standard NaN/Infinity extension."""
    source = Path(path)

    def reject_constant(value: str) -> None:
        raise ValueError(f"contract contains non-finite JSON constant: {value}")

    try:
        raw = json.loads(
            source.read_text(encoding="utf-8"),
            parse_constant=reject_constant,
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load contract {source}: {exc}") from exc
    return parser(raw)


@dataclass(frozen=True)
class CoordinateFrame:
    name: str
    units: str
    origin_id: str
    altitude_datum: str

    @classmethod
    def from_dict(cls, raw: Any, field: str = "frame") -> "CoordinateFrame":
        value = _object(raw, field)
        fields = {"name", "units", "origin_id", "altitude_datum"}
        _strict(value, required=fields, field=field)
        return cls(
            name=_choice(value["name"], {"LOCAL_NED"}, f"{field}.name"),
            units=_choice(value["units"], {"m"}, f"{field}.units"),
            origin_id=_identifier(value["origin_id"], f"{field}.origin_id"),
            altitude_datum=_choice(
                value["altitude_datum"],
                {"launch"},
                f"{field}.altitude_datum",
            ),
        )


@dataclass(frozen=True)
class PathPoint:
    point_id: str
    north_m: float
    east_m: float
    down_m: float
    hard: bool

    @classmethod
    def from_dict(cls, raw: Any, index: int) -> "PathPoint":
        field = f"path.points[{index}]"
        value = _object(raw, field)
        fields = {"point_id", "north_m", "east_m", "down_m", "hard"}
        _strict(value, required=fields, field=field)
        return cls(
            point_id=_identifier(value["point_id"], f"{field}.point_id"),
            north_m=_number(value["north_m"], f"{field}.north_m"),
            east_m=_number(value["east_m"], f"{field}.east_m"),
            down_m=_number(value["down_m"], f"{field}.down_m"),
            hard=_boolean(value["hard"], f"{field}.hard"),
        )


@dataclass(frozen=True)
class PathIntent:
    kind: str
    closure: str
    maximum_deviation_m: float
    points: tuple[PathPoint, ...]

    @classmethod
    def from_dict(cls, raw: Any) -> "PathIntent":
        value = _object(raw, "path")
        fields = {"kind", "closure", "maximum_deviation_m", "points"}
        _strict(value, required=fields, field="path")
        points = tuple(
            PathPoint.from_dict(point, index)
            for index, point in enumerate(_array(value["points"], "path.points"))
        )
        if not 2 <= len(points) <= 10_000:
            raise ValueError("path.points must contain 2..10000 entries")
        identifiers = [point.point_id for point in points]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("path point identifiers must be unique")
        closure = _choice(
            value["closure"],
            {"open_stop", "open_depart", "periodic"},
            "path.closure",
        )
        if closure == "periodic":
            first, last = points[0], points[-1]
            separation = math.dist(
                (first.north_m, first.east_m, first.down_m),
                (last.north_m, last.east_m, last.down_m),
            )
            if separation > _number(
                value["maximum_deviation_m"],
                "path.maximum_deviation_m",
                minimum=0.0,
            ):
                raise ValueError("periodic path endpoints exceed maximum deviation")
        return cls(
            kind=_choice(value["kind"], {"waypoints", "polyline"}, "path.kind"),
            closure=closure,
            maximum_deviation_m=_number(
                value["maximum_deviation_m"],
                "path.maximum_deviation_m",
                minimum=0.0,
            ),
            points=points,
        )


@dataclass(frozen=True)
class MotionIntent:
    target_speed_m_s: float | None
    maximum_speed_m_s: float
    yaw_mode: str
    fixed_yaw_deg: float | None
    time_mode: str
    requested_duration_s: float | None

    @classmethod
    def from_dict(cls, raw: Any) -> "MotionIntent":
        value = _object(raw, "motion")
        fields = {
            "target_speed_m_s",
            "maximum_speed_m_s",
            "yaw_mode",
            "fixed_yaw_deg",
            "time_mode",
            "requested_duration_s",
        }
        _strict(value, required=fields, field="motion")
        target = _optional_number(
            value["target_speed_m_s"],
            "motion.target_speed_m_s",
            exclusive_minimum=0.0,
        )
        maximum = _number(
            value["maximum_speed_m_s"],
            "motion.maximum_speed_m_s",
            exclusive_minimum=0.0,
        )
        if target is not None and target > maximum:
            raise ValueError("motion target speed exceeds maximum speed")
        time_mode = _choice(
            value["time_mode"],
            {"minimize", "exact", "maximum", "none"},
            "motion.time_mode",
        )
        duration = _optional_number(
            value["requested_duration_s"],
            "motion.requested_duration_s",
            exclusive_minimum=0.0,
        )
        if (time_mode in {"exact", "maximum"}) != (duration is not None):
            raise ValueError("requested duration is inconsistent with time mode")
        yaw_mode = _choice(
            value["yaw_mode"],
            {"fixed", "tangent", "unconstrained"},
            "motion.yaw_mode",
        )
        fixed_yaw = _optional_number(
            value["fixed_yaw_deg"],
            "motion.fixed_yaw_deg",
        )
        if (yaw_mode == "fixed") != (fixed_yaw is not None):
            raise ValueError("fixed yaw value is inconsistent with yaw mode")
        if fixed_yaw is not None and not -180.0 <= fixed_yaw <= 180.0:
            raise ValueError("motion.fixed_yaw_deg must be in [-180, 180]")
        return cls(
            target_speed_m_s=target,
            maximum_speed_m_s=maximum,
            yaw_mode=yaw_mode,
            fixed_yaw_deg=fixed_yaw,
            time_mode=time_mode,
            requested_duration_s=duration,
        )


@dataclass(frozen=True)
class MissionIntent:
    schema_version: int
    mission_id: str
    release_class: str
    frame: CoordinateFrame
    path: PathIntent
    motion: MotionIntent
    vehicle_model_id: str
    environment_id: str
    objective_priority: tuple[str, ...]

    REQUIRED: ClassVar[set[str]] = {
        "schema_version",
        "mission_id",
        "release_class",
        "frame",
        "path",
        "motion",
        "vehicle_model_id",
        "environment_id",
        "objective_priority",
    }

    @classmethod
    def from_dict(cls, raw: Any) -> "MissionIntent":
        value = _object(raw, "mission_intent")
        _strict(value, required=cls.REQUIRED, field="mission_intent")
        priority = tuple(_array(value["objective_priority"], "objective_priority"))
        if priority != OBJECTIVE_PRIORITY:
            raise ValueError("objective_priority must use the safety-first order")
        return cls(
            schema_version=_version(value["schema_version"], "schema_version"),
            mission_id=_identifier(value["mission_id"], "mission_id"),
            release_class=_choice(
                value["release_class"], {"SITL_ONLY"}, "release_class"
            ),
            frame=CoordinateFrame.from_dict(value["frame"]),
            path=PathIntent.from_dict(value["path"]),
            motion=MotionIntent.from_dict(value["motion"]),
            vehicle_model_id=_identifier(
                value["vehicle_model_id"], "vehicle_model_id"
            ),
            environment_id=_identifier(value["environment_id"], "environment_id"),
            objective_priority=priority,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class VehicleModel:
    schema_version: int
    vehicle_model_id: str
    configuration: str
    mass_kg: float
    maximum_speed_m_s: float
    maximum_acceleration_m_s2: float
    maximum_jerk_m_s3: float
    maximum_tilt_deg: float
    maximum_body_rate_deg_s: float
    minimum_thrust_n: float
    maximum_thrust_n: float
    identified_for_physical_flight: bool

    @classmethod
    def from_dict(cls, raw: Any) -> "VehicleModel":
        value = _object(raw, "vehicle_model")
        fields = set(cls.__dataclass_fields__)
        _strict(value, required=fields, field="vehicle_model")
        minimum_thrust = _number(
            value["minimum_thrust_n"], "minimum_thrust_n", minimum=0.0
        )
        maximum_thrust = _number(
            value["maximum_thrust_n"], "maximum_thrust_n", exclusive_minimum=0.0
        )
        if minimum_thrust >= maximum_thrust:
            raise ValueError("minimum thrust must be below maximum thrust")
        return cls(
            schema_version=_version(value["schema_version"], "schema_version"),
            vehicle_model_id=_identifier(
                value["vehicle_model_id"], "vehicle_model_id"
            ),
            configuration=_text(value["configuration"], "configuration"),
            mass_kg=_number(value["mass_kg"], "mass_kg", exclusive_minimum=0.0),
            maximum_speed_m_s=_number(
                value["maximum_speed_m_s"],
                "maximum_speed_m_s",
                exclusive_minimum=0.0,
            ),
            maximum_acceleration_m_s2=_number(
                value["maximum_acceleration_m_s2"],
                "maximum_acceleration_m_s2",
                exclusive_minimum=0.0,
            ),
            maximum_jerk_m_s3=_number(
                value["maximum_jerk_m_s3"],
                "maximum_jerk_m_s3",
                exclusive_minimum=0.0,
            ),
            maximum_tilt_deg=_number(
                value["maximum_tilt_deg"],
                "maximum_tilt_deg",
                exclusive_minimum=0.0,
            ),
            maximum_body_rate_deg_s=_number(
                value["maximum_body_rate_deg_s"],
                "maximum_body_rate_deg_s",
                exclusive_minimum=0.0,
            ),
            minimum_thrust_n=minimum_thrust,
            maximum_thrust_n=maximum_thrust,
            identified_for_physical_flight=_boolean(
                value["identified_for_physical_flight"],
                "identified_for_physical_flight",
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class EnvironmentSnapshot:
    schema_version: int
    environment_id: str
    frame: CoordinateFrame
    max_radius_m: float
    max_height_above_launch_m: float
    max_drop_below_launch_m: float
    max_wind_m_s: float
    minimum_clearance_m: float
    obstacle_map_sha256: str | None
    obstacle_map_verified: bool

    @classmethod
    def from_dict(cls, raw: Any) -> "EnvironmentSnapshot":
        value = _object(raw, "environment")
        fields = set(cls.__dataclass_fields__)
        _strict(value, required=fields, field="environment")
        map_digest = value["obstacle_map_sha256"]
        if map_digest is not None:
            map_digest = _digest(map_digest, "obstacle_map_sha256")
        verified = _boolean(value["obstacle_map_verified"], "obstacle_map_verified")
        if verified != (map_digest is not None):
            raise ValueError("map verification and map digest are inconsistent")
        return cls(
            schema_version=_version(value["schema_version"], "schema_version"),
            environment_id=_identifier(value["environment_id"], "environment_id"),
            frame=CoordinateFrame.from_dict(value["frame"]),
            max_radius_m=_number(
                value["max_radius_m"], "max_radius_m", exclusive_minimum=0.0
            ),
            max_height_above_launch_m=_number(
                value["max_height_above_launch_m"],
                "max_height_above_launch_m",
                exclusive_minimum=0.0,
            ),
            max_drop_below_launch_m=_number(
                value["max_drop_below_launch_m"],
                "max_drop_below_launch_m",
                minimum=0.0,
            ),
            max_wind_m_s=_number(
                value["max_wind_m_s"], "max_wind_m_s", minimum=0.0
            ),
            minimum_clearance_m=_number(
                value["minimum_clearance_m"],
                "minimum_clearance_m",
                minimum=0.0,
            ),
            obstacle_map_sha256=map_digest,
            obstacle_map_verified=verified,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class VerificationCheck:
    requirement_id: str
    status: str
    hard: bool
    margin: float | None
    units: str | None
    detail: str

    @classmethod
    def from_dict(cls, raw: Any, index: int) -> "VerificationCheck":
        field = f"checks[{index}]"
        value = _object(raw, field)
        fields = {
            "requirement_id",
            "status",
            "hard",
            "margin",
            "units",
            "detail",
        }
        _strict(value, required=fields, field=field)
        units = value["units"]
        if units is not None:
            units = _text(units, f"{field}.units", maximum=32)
        return cls(
            requirement_id=_identifier(
                value["requirement_id"], f"{field}.requirement_id"
            ),
            status=_choice(value["status"], {"PASS", "FAIL"}, f"{field}.status"),
            hard=_boolean(value["hard"], f"{field}.hard"),
            margin=_optional_number(value["margin"], f"{field}.margin"),
            units=units,
            detail=_text(value["detail"], f"{field}.detail", maximum=1024),
        )


@dataclass(frozen=True)
class VerificationReport:
    schema_version: int
    verifier_version: str
    candidate_trajectory_sha256: str
    accepted: bool
    checks: tuple[VerificationCheck, ...]

    @classmethod
    def from_dict(cls, raw: Any) -> "VerificationReport":
        value = _object(raw, "verification_report")
        fields = set(cls.__dataclass_fields__)
        _strict(value, required=fields, field="verification_report")
        checks = tuple(
            VerificationCheck.from_dict(check, index)
            for index, check in enumerate(_array(value["checks"], "checks"))
        )
        if not checks:
            raise ValueError("checks must not be empty")
        identifiers = [check.requirement_id for check in checks]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("verification requirement identifiers must be unique")
        computed_acceptance = all(
            check.status == "PASS" for check in checks if check.hard
        )
        accepted = _boolean(value["accepted"], "accepted")
        if accepted != computed_acceptance:
            raise ValueError("accepted does not match hard verification checks")
        return cls(
            schema_version=_version(value["schema_version"], "schema_version"),
            verifier_version=_identifier(
                value["verifier_version"], "verifier_version"
            ),
            candidate_trajectory_sha256=_digest(
                value["candidate_trajectory_sha256"],
                "candidate_trajectory_sha256",
            ),
            accepted=accepted,
            checks=checks,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PlanArtifact:
    schema_version: int
    release_class: str
    mission_intent_sha256: str
    vehicle_model_sha256: str
    environment_sha256: str
    candidate_trajectory_sha256: str
    verification_report_sha256: str
    simulation_evidence_sha256: str
    approved: bool
    approval_id: str | None
    artifact_sha256: str

    PAYLOAD_FIELDS: ClassVar[tuple[str, ...]] = (
        "schema_version",
        "release_class",
        "mission_intent_sha256",
        "vehicle_model_sha256",
        "environment_sha256",
        "candidate_trajectory_sha256",
        "verification_report_sha256",
        "simulation_evidence_sha256",
        "approved",
        "approval_id",
    )

    @classmethod
    def create(
        cls,
        *,
        mission_intent_sha256: str,
        vehicle_model_sha256: str,
        environment_sha256: str,
        candidate_trajectory_sha256: str,
        verification_report_sha256: str,
        simulation_evidence_sha256: str,
    ) -> "PlanArtifact":
        payload = {
            "schema_version": CONTRACT_VERSION,
            "release_class": "SITL_ONLY",
            "mission_intent_sha256": _digest(
                mission_intent_sha256, "mission_intent_sha256"
            ),
            "vehicle_model_sha256": _digest(
                vehicle_model_sha256, "vehicle_model_sha256"
            ),
            "environment_sha256": _digest(
                environment_sha256, "environment_sha256"
            ),
            "candidate_trajectory_sha256": _digest(
                candidate_trajectory_sha256, "candidate_trajectory_sha256"
            ),
            "verification_report_sha256": _digest(
                verification_report_sha256, "verification_report_sha256"
            ),
            "simulation_evidence_sha256": _digest(
                simulation_evidence_sha256, "simulation_evidence_sha256"
            ),
            "approved": False,
            "approval_id": None,
        }
        return cls(**payload, artifact_sha256=canonical_sha256(payload))

    @classmethod
    def from_dict(cls, raw: Any) -> "PlanArtifact":
        value = _object(raw, "plan_artifact")
        fields = set(cls.__dataclass_fields__) - {"PAYLOAD_FIELDS"}
        _strict(value, required=fields, field="plan_artifact")
        payload = {field: value[field] for field in cls.PAYLOAD_FIELDS}
        artifact = cls(
            schema_version=_version(value["schema_version"], "schema_version"),
            release_class=_choice(
                value["release_class"], {"SITL_ONLY"}, "release_class"
            ),
            mission_intent_sha256=_digest(
                value["mission_intent_sha256"], "mission_intent_sha256"
            ),
            vehicle_model_sha256=_digest(
                value["vehicle_model_sha256"], "vehicle_model_sha256"
            ),
            environment_sha256=_digest(
                value["environment_sha256"], "environment_sha256"
            ),
            candidate_trajectory_sha256=_digest(
                value["candidate_trajectory_sha256"],
                "candidate_trajectory_sha256",
            ),
            verification_report_sha256=_digest(
                value["verification_report_sha256"],
                "verification_report_sha256",
            ),
            simulation_evidence_sha256=_digest(
                value["simulation_evidence_sha256"],
                "simulation_evidence_sha256",
            ),
            approved=_boolean(value["approved"], "approved"),
            approval_id=value["approval_id"],
            artifact_sha256=_digest(value["artifact_sha256"], "artifact_sha256"),
        )
        if artifact.approved or artifact.approval_id is not None:
            raise ValueError("SITL_ONLY artifacts cannot carry flight approval")
        if artifact.artifact_sha256 != canonical_sha256(payload):
            raise ValueError("artifact SHA-256 does not match canonical payload")
        return artifact

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def validate_contract_set(
    intent: MissionIntent,
    vehicle: VehicleModel,
    environment: EnvironmentSnapshot,
    report: VerificationReport,
    *,
    candidate_trajectory_sha256: str,
) -> None:
    """Validate references and conservative limits across contract boundaries."""
    trajectory_digest = _digest(
        candidate_trajectory_sha256, "candidate_trajectory_sha256"
    )
    if intent.vehicle_model_id != vehicle.vehicle_model_id:
        raise ValueError("mission and vehicle model identifiers do not match")
    if intent.environment_id != environment.environment_id:
        raise ValueError("mission and environment identifiers do not match")
    if intent.frame != environment.frame:
        raise ValueError("mission and environment coordinate frames do not match")
    if intent.motion.maximum_speed_m_s > vehicle.maximum_speed_m_s:
        raise ValueError("mission speed exceeds the vehicle model envelope")
    if report.candidate_trajectory_sha256 != trajectory_digest:
        raise ValueError("verification report references another trajectory")
    if not report.accepted:
        raise ValueError("verification report did not accept the trajectory")

    for index, point in enumerate(intent.path.points):
        radius = math.hypot(point.north_m, point.east_m)
        height = -point.down_m
        if radius > environment.max_radius_m:
            raise ValueError(f"path.points[{index}] exceeds environment radius")
        if height > environment.max_height_above_launch_m:
            raise ValueError(f"path.points[{index}] exceeds environment height")
        if point.down_m > environment.max_drop_below_launch_m:
            raise ValueError(f"path.points[{index}] exceeds environment drop")
