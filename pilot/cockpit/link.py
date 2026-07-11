"""The abstraction boundary between the cockpit API and the wire.

`Cockpit` (api.py) speaks in `Request`/`Reply` values and `Event` objects;
a `CockpitLink` implementation turns those into whatever actually crosses
the UART (text lines, binary frames — the API neither knows nor cares).

Contract for implementations:

- `execute()` is blocking and synchronous: encode the request, send it,
  return the decoded answer. Raise `CockpitTimeout` if the airframe does
  not answer in time, `CockpitNack` if it answers with a refusal,
  `CockpitLinkError` if the transport is broken. At most one request is
  in flight at a time (the API layer guarantees this).
- Airframe-initiated events are handed to the sink set via
  `set_event_sink()`. The sink is cheap and non-blocking (the API layer
  queues and dispatches on its own thread); call it from wherever the
  link's reader runs.
- Any successfully executed request counts as Pilot liveness on the
  airframe side ("a commander is talking to me"), regardless of its op.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

from .events import Event

# Ops of the first milestone ("Pi5 drives the rover over UART with the
# deadman working"). The vocabulary grows here — one string per new command —
# without the link interface changing.
OP_PING = "ping"
OP_ARM = "arm"
OP_DISARM = "disarm"
OP_ESTOP = "estop"
OP_CLEAR_FAULT = "clear_fault"
OP_DRIVE = "drive"
OP_STOP = "stop"
OP_GET_STATE = "get_state"
OP_GET_ODOMETRY = "get_odometry"
OP_GET_VERSION = "get_version"


@dataclass(frozen=True)
class Request:
    op: str
    params: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Reply:
    values: Mapping[str, Any] = field(default_factory=dict)


class CockpitLink(ABC):
    """One concrete transport to the airframe (UART, or a simulator)."""

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def execute(self, request: Request, timeout: float) -> Reply:
        """Send one request, block for its answer, return it decoded."""

    @abstractmethod
    def set_event_sink(self, sink: Callable[[Event], None]) -> None:
        """Register where airframe-initiated events are delivered."""
