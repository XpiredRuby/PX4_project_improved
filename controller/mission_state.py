from enum import Enum


class TextEnum(str, Enum):
    """String-compatible enum that serializes cleanly into CSV logs."""

    def __str__(self):
        return self.value


class MissionPhase(TextEnum):
    TAKEOFF = "TAKEOFF"
    TRAJECTORY = "TRAJECTORY"
    RETURN_HOME = "RETURN_HOME"
    ALIGN = "ALIGN"
    HANDOFF = "HANDOFF"


class NavigationState(TextEnum):
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    HOLD = "HOLD"
    LOST = "LOST"


class FailureAction(TextEnum):
    LAND = "LAND"
    PX4_FAILSAFE = "PX4_FAILSAFE"



class MissionOutcome(TextEnum):
    PREARM_REJECTED = "PREARM_REJECTED"
    SUCCESS = "SUCCESS"
    ABORTED_TO_LAND = "ABORTED_TO_LAND"
    PX4_FAILSAFE = "PX4_FAILSAFE"
    UNSAFE_TOUCHDOWN = "UNSAFE_TOUCHDOWN"


MISSION_TRANSITIONS = {
    MissionPhase.TAKEOFF: frozenset((MissionPhase.TRAJECTORY,)),
    MissionPhase.TRAJECTORY: frozenset((MissionPhase.RETURN_HOME,)),
    MissionPhase.RETURN_HOME: frozenset((MissionPhase.ALIGN,)),
    MissionPhase.ALIGN: frozenset((MissionPhase.HANDOFF,)),
    MissionPhase.HANDOFF: frozenset(),
}


NAVIGATION_TRANSITIONS = {
    NavigationState.HEALTHY: frozenset(
        (NavigationState.DEGRADED, NavigationState.HOLD, NavigationState.LOST)
    ),
    NavigationState.DEGRADED: frozenset(
        (NavigationState.HEALTHY, NavigationState.HOLD, NavigationState.LOST)
    ),
    NavigationState.HOLD: frozenset(
        (NavigationState.HEALTHY, NavigationState.LOST)
    ),
    NavigationState.LOST: frozenset(),
}


def require_transition(current, requested, transition_map, label):
    """Return normalized enum state or reject an impossible transition."""
    state_type = type(next(iter(transition_map)))
    current = state_type(current)
    requested = state_type(requested)
    if requested == current:
        return current
    if requested not in transition_map[current]:
        raise RuntimeError(
            f"Invalid {label} transition {current.value}->{requested.value}"
        )
    return requested
