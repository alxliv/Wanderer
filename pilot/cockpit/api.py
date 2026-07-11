"""The Cockpit class — the Pilot's hands on the airframe.

Every method is blocking request/response: it returns the airframe's answer
immediately or raises (see errors.py). Airframe-initiated events are
delivered via callbacks registered with `on_event()`, from a dedicated
dispatcher thread so a slow handler can never stall the link's reader.

Units at this boundary are SI floats (m/s, rad/s, meters): the API speaks
the planner's language. Whatever integer millimeter encoding the wire uses
is the link's business.
"""

import queue
import threading
from dataclasses import dataclass
from typing import Callable, Optional, Type

from . import link as _link
from .errors import CockpitLinkError
from .events import Event, EventHandler, TacticalState
from .link import CockpitLink, Reply, Request

# The cockpit answers at once; a miss means the channel is in trouble.
DEFAULT_COMMAND_TIMEOUT_S = 0.1


@dataclass(frozen=True)
class Odometry:
    """Dead-reckoning seed pulled from the airframe (it drifts — fuse it,
    don't trust it; docs/Wanderer_Command_Architecture.md §8)."""

    left_ticks: int
    right_ticks: int
    left_m_s: float
    right_m_s: float


@dataclass(frozen=True)
class FirmwareVersion:
    major: int
    minor: int


class Cockpit:
    """Blocking command interface + event callbacks over one CockpitLink.

    Thread-safety: commands may be issued from any thread; they are
    serialized (single in-flight command, matching the link contract).
    Event handlers run on the dispatcher thread — keep them quick and
    never issue cockpit commands from inside one (hand off to your own
    loop instead).
    """

    def __init__(self, link: CockpitLink, *,
                 command_timeout: float = DEFAULT_COMMAND_TIMEOUT_S):
        self._link = link
        self._timeout = command_timeout
        self._command_lock = threading.Lock()
        self._handlers: list[tuple[Optional[Type[Event]], EventHandler]] = []
        self._handlers_lock = threading.Lock()
        self._event_queue: "queue.Queue[Optional[Event]]" = queue.Queue()
        self._dispatcher: Optional[threading.Thread] = None
        self._open = False

    # ---- lifecycle ----------------------------------------------------

    def open(self) -> None:
        if self._open:
            return
        self._link.set_event_sink(self._event_queue.put)
        self._link.open()
        self._dispatcher = threading.Thread(
            target=self._dispatch_events, name="cockpit-events", daemon=True)
        self._dispatcher.start()
        self._open = True

    def close(self) -> None:
        if not self._open:
            return
        self._open = False
        self._link.close()
        self._event_queue.put(None)  # wake and end the dispatcher
        if self._dispatcher is not None:
            self._dispatcher.join(timeout=1.0)
            self._dispatcher = None

    def __enter__(self) -> "Cockpit":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- events --------------------------------------------------------

    def on_event(self, handler: EventHandler,
                 event_type: Optional[Type[Event]] = None) -> Callable[[], None]:
        """Register a callback for airframe events.

        `event_type=None` receives everything; otherwise only instances of
        that type. Returns an unsubscribe function.
        """
        entry = (event_type, handler)
        with self._handlers_lock:
            self._handlers.append(entry)

        def unsubscribe() -> None:
            with self._handlers_lock:
                if entry in self._handlers:
                    self._handlers.remove(entry)

        return unsubscribe

    # ---- housekeeping ----------------------------------------------------

    def ping(self) -> None:
        """No-op that refreshes the Pilot's liveness lease.

        While driving, the velocity stream itself keeps the lease; ping is
        for holding the cockpit's attention during quiet thinking phases.
        (Deliberately not automated: the deadman must reflect the Pilot's
        real health, so only the Pilot's own loop feeds it.)
        """
        self._execute(_link.OP_PING)

    def arm(self) -> None:
        """SAFE -> ACTIVE. No motion starts until drive() is called."""
        self._execute(_link.OP_ARM)

    def disarm(self) -> None:
        """Back to SAFE from any movement-capable state; refused in FAULT."""
        self._execute(_link.OP_DISARM)

    def estop(self) -> None:
        """Latch an emergency-stop fault: motors gate off *now*, and stay
        off until clear_fault(). Honored in every state."""
        self._execute(_link.OP_ESTOP)

    def clear_fault(self) -> None:
        """Clear a latched fault -> SAFE. Nacked while the condition persists."""
        self._execute(_link.OP_CLEAR_FAULT)

    # ---- tier 1: the control surface -------------------------------------

    def drive(self, linear_m_s: float, angular_rad_s: float) -> None:
        """Set the standing body-velocity order (skid steer: no strafe).

        The airframe holds this until superseded. A fresh drive() is also
        the only thing that resumes from FALLBACK — the Pilot must
        re-assert intent, the airframe never silently re-accelerates.
        Nacked when not armed.
        """
        self._execute(_link.OP_DRIVE,
                      linear_m_s=float(linear_m_s),
                      angular_rad_s=float(angular_rad_s))

    def stop(self) -> None:
        """Zero the standing velocity order. Stays ACTIVE (unlike disarm)."""
        self._execute(_link.OP_STOP)

    # ---- queries (the Pilot pulls; nothing streams) -----------------------

    def state(self) -> TacticalState:
        reply = self._execute(_link.OP_GET_STATE)
        return TacticalState(reply.values["state"])

    def odometry(self) -> Odometry:
        reply = self._execute(_link.OP_GET_ODOMETRY)
        v = reply.values
        return Odometry(left_ticks=v["left_ticks"], right_ticks=v["right_ticks"],
                        left_m_s=v["left_m_s"], right_m_s=v["right_m_s"])

    def version(self) -> FirmwareVersion:
        reply = self._execute(_link.OP_GET_VERSION)
        return FirmwareVersion(major=reply.values["major"],
                               minor=reply.values["minor"])

    # ---- internals ---------------------------------------------------------

    def _execute(self, op: str, **params) -> Reply:
        if not self._open:
            raise CockpitLinkError("cockpit is not open")
        with self._command_lock:
            return self._link.execute(Request(op, params), self._timeout)

    def _dispatch_events(self) -> None:
        while True:
            event = self._event_queue.get()
            if event is None:
                return
            with self._handlers_lock:
                handlers = list(self._handlers)
            for event_type, handler in handlers:
                if event_type is None or isinstance(event, event_type):
                    try:
                        handler(event)
                    except Exception:
                        # A pilot bug in one handler must not kill delivery
                        # to the others or the dispatcher itself.
                        import traceback
                        traceback.print_exc()
