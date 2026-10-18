"""Record that an operator reviewed a SITL artifact.

The record binds the trajectory and report digests and expires. It is not a
flight approval: the release class stays ``SITL_ONLY`` and the statement is
fixed. A missing, stale, or mismatched record refuses execution.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from .contracts import load_contract


STATEMENT = "not a clearance to fly"


def _identifier(value: str, field: str) -> str:
    if not isinstance(value, str) or not value or len(value) > 128:
        raise ValueError(f"{field} must be a short identifier")
    if any(character.isspace() for character in value):
        raise ValueError(f"{field} must not contain whitespace")
    return value


def _digest(value: str, field: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(f"{field} must be a lowercase SHA-256 digest")
    return value


def _expiry(value: str) -> datetime:
    if not isinstance(value, str):
        raise ValueError("expires_at must be an ISO-8601 UTC timestamp")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError("expires_at must be an ISO-8601 UTC timestamp") from exc
    if parsed.tzinfo is None:
        raise ValueError("expires_at must include a timezone")
    return parsed.astimezone(timezone.utc)


def issue_sitl_acknowledgement(
    *,
    controller_trajectory_sha256: str,
    candidate_trajectory_sha256: str,
    verification_report_sha256: str,
    expires_at: str,
    acknowledged_by: str,
) -> dict:
    """Return a review record. The caller has to choose the reviewer and expiry."""
    _expiry(expires_at)
    return {
        "schema_version": 1,
        "release_class": "SITL_ONLY",
        "statement": STATEMENT,
        "acknowledged_by": _identifier(acknowledged_by, "acknowledged_by"),
        "expires_at": expires_at,
        "controller_trajectory_sha256": _digest(
            controller_trajectory_sha256, "controller_trajectory_sha256"
        ),
        "candidate_trajectory_sha256": _digest(
            candidate_trajectory_sha256, "candidate_trajectory_sha256"
        ),
        "verification_report_sha256": _digest(
            verification_report_sha256, "verification_report_sha256"
        ),
    }


def load_sitl_acknowledgement(
    acknowledgement_path: Path,
    summary_path: Path,
    trajectory_path: Path,
    *,
    now: datetime | None = None,
) -> dict:
    """Accept a review record only when it matches this summary and is unexpired."""
    moment = now or datetime.now(timezone.utc)
    if moment.tzinfo is None:
        raise ValueError("acknowledgement time must include a timezone")
    moment = moment.astimezone(timezone.utc)

    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant: {value}")

    try:
        summary = json.loads(
            Path(summary_path).read_text(encoding="utf-8"),
            parse_constant=reject_constant,
        )
        raw = json.loads(
            Path(acknowledgement_path).read_text(encoding="utf-8"),
            parse_constant=reject_constant,
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load SITL acknowledgement: {exc}") from exc
    if not isinstance(summary, dict) or not isinstance(raw, dict):
        raise ValueError("SITL acknowledgement and summary must be objects")
    if summary.get("release_class") != "SITL_ONLY":
        raise ValueError("SITL acknowledgement requires a SITL_ONLY summary")
    record = issue_sitl_acknowledgement(
        controller_trajectory_sha256=raw.get("controller_trajectory_sha256", ""),
        candidate_trajectory_sha256=raw.get("candidate_trajectory_sha256", ""),
        verification_report_sha256=raw.get("verification_report_sha256", ""),
        expires_at=raw.get("expires_at", ""),
        acknowledged_by=raw.get("acknowledged_by", ""),
    )
    if raw.get("schema_version") != 1 or raw.get("release_class") != "SITL_ONLY":
        raise ValueError("SITL acknowledgement release class or version is wrong")
    if raw.get("statement") != STATEMENT:
        raise ValueError("SITL acknowledgement must say it is not a clearance to fly")
    extra = set(raw) - set(record)
    if extra:
        raise ValueError(
            "SITL acknowledgement has unknown fields: " + ", ".join(sorted(extra))
        )
    for field in (
        "controller_trajectory_sha256",
        "candidate_trajectory_sha256",
        "verification_report_sha256",
    ):
        if summary.get(field) != record[field]:
            raise ValueError(f"SITL acknowledgement does not match summary {field}")
    actual = hashlib.sha256(Path(trajectory_path).read_bytes()).hexdigest()
    if actual != record["controller_trajectory_sha256"]:
        raise ValueError("SITL acknowledgement does not match the selected trajectory")
    if _expiry(record["expires_at"]) <= moment:
        raise ValueError("SITL acknowledgement has expired")
    return record


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--expires-at", required=True)
    parser.add_argument("--acknowledged-by", required=True)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    summary = load_contract(args.summary, dict)
    record = issue_sitl_acknowledgement(
        controller_trajectory_sha256=summary["controller_trajectory_sha256"],
        candidate_trajectory_sha256=summary["candidate_trajectory_sha256"],
        verification_report_sha256=summary["verification_report_sha256"],
        expires_at=args.expires_at,
        acknowledged_by=args.acknowledged_by,
    )
    args.output.write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
