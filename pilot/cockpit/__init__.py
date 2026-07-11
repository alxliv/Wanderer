"""Cockpit API — the Pilot's (Pi5) interface to the airframe (Pico2).

This package is the *contract* between the strategic and tactical layers.
The wire protocol underneath (text lines, binary frames, whatever) is an
implementation detail hidden behind `CockpitLink`; pilot code imports only
from here and never sees the UART.

Model (docs/Wanderer_Command_Architecture.md):

- Commands are blocking request/response with a short timeout. The cockpit
  is a dependable channel: every command is answered at once, or something
  is wrong and an exception is raised.
- Unexpected airframe events (state changes, faults; later: reflex fires,
  procedure completions) arrive as callbacks, never as return values.
- There is NO telemetry stream to the Pilot. The Pilot pulls what it needs
  (state, odometry) via queries at its own cadence. The 100 Hz telemetry
  broadcast goes to the Base via the transponder, not through this API.
- If the Pilot goes quiet, the airframe enters fallback ("zombie") mode by
  itself and ramps to a stop. Liveness is any valid command; while driving,
  the velocity stream itself is the heartbeat. `ping()` exists for the
  quiet phases where the Pilot wants to keep the lease without moving.
"""

from .api import Cockpit, FirmwareVersion, Odometry
from .errors import CockpitError, CockpitLinkError, CockpitNack, CockpitTimeout
from .events import Event, EventHandler, FaultRaised, StateChanged, TacticalState
from .link import CockpitLink, Reply, Request

__all__ = [
    "Cockpit",
    "CockpitError",
    "CockpitLink",
    "CockpitLinkError",
    "CockpitNack",
    "CockpitTimeout",
    "Event",
    "EventHandler",
    "FaultRaised",
    "FirmwareVersion",
    "Odometry",
    "Reply",
    "Request",
    "StateChanged",
    "TacticalState",
]
