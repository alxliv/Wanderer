"""Blocking commands and asynchronous events for controlling the airframe.

Commands return the airframe's response or raise an error from ``errors.py``.
Airframe events are delivered to callbacks registered with ``on_event()``.
Callbacks run on a dispatcher thread, separate from the link reader.

The API uses SI units. The link handles wire-format conversions.
"""

import queue
import threading
from dataclasses import dataclass
from typing import Callable, Optional, Type

from . import link as _link
from .errors import CockpitLinkError
from .events import Event, EventHandler, TacticalState
from .link import CockpitLink, Reply, Request

# Maximum time to wait for a command response.
DEFAULT_COMMAND_TIMEOUT_S = 0.1


@dataclass(frozen=True)
class Odometry:
    """Encoder tick counts and wheel speeds reported by the airframe."""

    left_ticks: int
    right_ticks: int
    left_m_s: float
    right_m_s: float


@dataclass(frozen=True)
class FirmwareVersion:
    major: int
    minor: int


class Cockpit:
    """Blocking command interface and event callbacks over one link.

    Commands are thread-safe and execute one at a time. Event handlers run on
    the dispatcher thread; they should return quickly and must not issue
    cockpit commands.
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

    # Lifecycle

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
        self._event_queue.put(None)  # Stop the dispatcher.
        if self._dispatcher is not None:
            self._dispatcher.join(timeout=1.0)
            self._dispatcher = None

    def __enter__(self) -> "Cockpit":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # Events

    def on_event(self, handler: EventHandler,
                 event_type: Optional[Type[Event]] = None) -> Callable[[], None]:
        """Register a callback for airframe events.

        If ``event_type`` is omitted, the callback receives every event.
        Returns a function that unregisters the callback.
        """
        entry = (event_type, handler)
        with self._handlers_lock:
            self._handlers.append(entry)

        def unsubscribe() -> None:
            with self._handlers_lock:
                if entry in self._handlers:
                    self._handlers.remove(entry)

        return unsubscribe

    # State management

    def ping(self) -> None:
        """Refresh the Pilot's liveness lease without changing state.

        Drive commands refresh the lease during motion. Call ``ping()`` during
        idle periods. This is intentionally not automatic so the deadman can
        detect an unresponsive Pilot.
        """
        self._execute(_link.OP_PING)

    def arm(self) -> None:
        """Enter ACTIVE state without starting motion."""
        self._execute(_link.OP_ARM)

    def disarm(self) -> None:
        """Enter SAFE state. The airframe refuses this command in FAULT."""
        self._execute(_link.OP_DISARM)

    def estop(self) -> None:
        """Stop the motors and latch FAULT until ``clear_fault()`` succeeds."""
        self._execute(_link.OP_ESTOP)

    def clear_fault(self) -> None:
        """Clear FAULT and enter SAFE if the fault condition is gone."""
        self._execute(_link.OP_CLEAR_FAULT)

    # Motion control

    def drive(self, linear_m_s: float, angular_rad_s: float) -> None:
        """Set the forward and angular velocity command.

        The command remains active until replaced. After FALLBACK, only a new
        ``drive()`` command resumes motion. The airframe rejects this command
        unless it is armed.
        """
        self._execute(_link.OP_DRIVE,
                      linear_m_s=float(linear_m_s),
                      angular_rad_s=float(angular_rad_s))

    def stop(self) -> None:
        """Set velocity to zero while remaining ACTIVE."""
        self._execute(_link.OP_STOP)

    # Queries

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

    # Internal helpers

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
                        # Keep dispatching if a handler fails.
                        import traceback
                        traceback.print_exc()
