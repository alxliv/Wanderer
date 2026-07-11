"""Events pushed by the airframe to the Pilot.

Events are the *asynchronous* half of the cockpit contract: things the
airframe reports on its own initiative, delivered to pilot code as callbacks.
The set grows with the tactical vocabulary (reflex fires, procedure
completions); the delivery machinery in `api.py` does not change when it does.
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Callable


class TacticalState(IntEnum):
    """Mirror of the airframe's TacticalCore FSM (firmware/common/tactical.h)."""

    SAFE = 0      # disarmed, motors gated off
    ACTIVE = 1    # armed, emitting the commanded velocity
    FALLBACK = 2  # "zombie" mode: no live commander, ramping to zero
    FAULT = 3     # latched fault, motors off until explicitly cleared


@dataclass(frozen=True)
class Event:
    """Base class for airframe-initiated notifications."""


@dataclass(frozen=True)
class StateChanged(Event):
    """The tactical FSM moved. Notably: entering FALLBACK means the airframe
    decided the Pilot went quiet and is stopping itself."""

    old: TacticalState
    new: TacticalState


@dataclass(frozen=True)
class FaultRaised(Event):
    """A fault latched (includes e-stop). Motion is gated off until
    `Cockpit.clear_fault()` succeeds."""

    code: int


EventHandler = Callable[[Event], None]
