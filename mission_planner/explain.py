"""Structured explanations for plans that cannot be flown."""

from __future__ import annotations


class PlanningError(ValueError):
    """A fail-closed planning refusal with the constraint that caused it.

    The message is safe to show to an operator. It describes the limit that
    was hit and, when duration is the problem, the shortest duration that
    would satisfy it. It is not a clearance to fly.
    """

    def __init__(
        self,
        message: str,
        *,
        limiting_constraint: str | None = None,
        minimum_feasible_duration_s: float | None = None,
    ) -> None:
        super().__init__(message)
        self.limiting_constraint = limiting_constraint
        self.minimum_feasible_duration_s = minimum_feasible_duration_s

    def to_dict(self) -> dict:
        return {
            "accepted": False,
            "release_class": "SITL_ONLY",
            "clearance": "not a clearance to fly",
            "summary": str(self),
            "limiting_constraint": self.limiting_constraint,
            "minimum_feasible_duration_s": self.minimum_feasible_duration_s,
        }
