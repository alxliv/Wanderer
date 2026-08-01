"""A simulated airframe behind the CockpitLink interface.

Mirrors the tactical FSM (firmware/common/tactical.h/.cpp): Safe / Active /
Fallback / Fault, liveness fed by any valid command, fallback on commander
silence, resume only on a fresh drive. It also mirrors the airframe's
wheel-speed limit (cockpit_handler.cpp `drive_scale`), so a saturating drive
behaves here as it does on the rover. It is both a test double for pilot code
and an executable statement of what the real UART link + firmware must do.
(Simplification: the fallback ramp is instantaneous — velocity semantics, not
motion dynamics, are what this sim pins down.)
"""

import threading
import time
from typing import Callable, Optional

from . import link as _link
from .errors import CockpitLinkError, CockpitNack
from .events import Event, FaultRaised, StateChanged, TacticalState
from .link import CockpitLink, Reply, Request

FAULT_ESTOP = 1
TICKS_PER_M = 10000.0
TICK_PERIOD_S = 0.02  # 50 Hz FSM tick, plenty for liveness resolution

# The rover's real geometry, from firmware/airframe/src/config.h:
# TRACK_WIDTH_M / 2, and DEFAULT_MAX_SPEED_MM_S / 1000.
HALF_TRACK_M = 0.15
MAX_WHEEL_M_S = 0.6


class SimulatedCockpitLink(CockpitLink):
    def __init__(self, *, liveness_timeout_s: float = 0.75,
                 half_track_m: float = HALF_TRACK_M,
                 max_wheel_m_s: float = MAX_WHEEL_M_S):
        self._liveness_timeout = liveness_timeout_s
        self._half_track = half_track_m
        self._max_wheel = max_wheel_m_s
        self._sink: Optional[Callable[[Event], None]] = None
        self._lock = threading.Lock()
        self._running = False
        self._ticker: Optional[threading.Thread] = None

        self._state = TacticalState.SAFE
        self._fault_code = 0
        self._v_left = 0.0    # commanded wheel velocities, m/s
        self._v_right = 0.0
        self._last_seen = 0.0
        self._left_ticks = 0.0
        self._right_ticks = 0.0
        self._last_tick_time = 0.0

    # ---- CockpitLink ----------------------------------------------------

    def set_event_sink(self, sink: Callable[[Event], None]) -> None:
        self._sink = sink

    def open(self) -> None:
        with self._lock:
            self._running = True
            self._last_tick_time = time.monotonic()
        self._ticker = threading.Thread(
            target=self._tick_loop, name="sim-airframe", daemon=True)
        self._ticker.start()

    def close(self) -> None:
        with self._lock:
            self._running = False
        if self._ticker is not None:
            self._ticker.join(timeout=1.0)
            self._ticker = None

    def execute(self, request: Request, timeout: float) -> Reply:
        with self._lock:
            if not self._running:
                raise CockpitLinkError("link is closed")
            # Liveness: ANY valid frame from the commander counts.
            self._last_seen = time.monotonic()
            return self._handle(request)

    # ---- the simulated tactical layer (call with lock held) --------------

    def _handle(self, request: Request) -> Reply:
        op, p = request.op, request.params
        if op == _link.OP_PING:
            return Reply()
        if op == _link.OP_ARM:
            if self._state == TacticalState.FAULT:
                raise CockpitNack("fault_latched", "clear the fault first")
            if self._state == TacticalState.SAFE:
                self._enter(TacticalState.ACTIVE)
            return Reply()  # ACTIVE/FALLBACK: no-op ok; arm never exits FALLBACK
        if op == _link.OP_DISARM:
            if self._state == TacticalState.FAULT:
                raise CockpitNack("fault_latched", "clear the fault first")
            self._enter(TacticalState.SAFE)
            return Reply()
        if op == _link.OP_ESTOP:
            if self._state == TacticalState.FAULT:
                return Reply()  # re-raise: first cause preserved, no events
            self._fault_code = FAULT_ESTOP
            self._emit(FaultRaised(code=FAULT_ESTOP))  # `!fault` before `!state`
            self._enter(TacticalState.FAULT)
            return Reply()
        if op == _link.OP_CLEAR_FAULT:
            if self._state != TacticalState.FAULT:
                raise CockpitNack("no_fault", "nothing is latched")
            self._fault_code = 0
            self._enter(TacticalState.SAFE)
            return Reply()
        if op == _link.OP_DRIVE:
            # A fresh drive is the ONLY thing that resumes from FALLBACK.
            if self._state == TacticalState.FALLBACK:
                self._enter(TacticalState.ACTIVE)
            if self._state != TacticalState.ACTIVE:
                raise CockpitNack("not_armed", f"state is {self._state.name}")
            v, w = p["linear_m_s"], p["angular_rad_s"]
            left = v - w * self._half_track
            right = v + w * self._half_track
            scale = self._drive_scale(left, right)
            self._v_left = left * scale
            self._v_right = right * scale
            if scale < 1.0:
                # Mirrors the firmware: the applied pair is reported ONLY when
                # it differs from the request, so its presence is the
                # saturation signal (spec section 3).
                return Reply({"linear_m_s": v * scale,
                              "angular_rad_s": w * scale})
            return Reply()
        if op == _link.OP_STOP:
            if self._state != TacticalState.ACTIVE:
                raise CockpitNack("not_armed", f"state is {self._state.name}")
            self._v_left = self._v_right = 0.0
            return Reply()
        if op == _link.OP_GET_STATE:
            return Reply({"state": int(self._state)})
        if op == _link.OP_GET_ODOMETRY:
            return Reply({
                "left_ticks": int(self._left_ticks),
                "right_ticks": int(self._right_ticks),
                "left_m_s": self._v_left if self._moving() else 0.0,
                "right_m_s": self._v_right if self._moving() else 0.0,
            })
        if op == _link.OP_GET_VERSION:
            return Reply({"major": 0, "minor": 1})
        raise CockpitNack("bad_op", op)

    def _drive_scale(self, left: float, right: float) -> float:
        """Factor bringing an over-limit wheel PAIR back inside the limit.

        Both wheels scale by the same factor, so the ratio between them — and
        with it the commanded turn radius — survives, and only the speed
        drops. Clipping each wheel separately would change the difference
        between them, which *is* the turn. Mirrors `drive_scale()` in
        firmware/common/cockpit_handler.cpp.
        """
        if self._max_wheel <= 0.0:
            return 1.0
        peak = max(abs(left), abs(right))
        if peak <= self._max_wheel:
            return 1.0
        return self._max_wheel / peak

    def _moving(self) -> bool:
        return self._state == TacticalState.ACTIVE

    def _enter(self, new: TacticalState) -> None:
        if new == self._state:
            return
        old, self._state = self._state, new
        if new in (TacticalState.SAFE, TacticalState.FAULT,
                   TacticalState.FALLBACK):
            self._v_left = self._v_right = 0.0  # instant "ramp"
        self._emit(StateChanged(old=old, new=new))

    def _emit(self, event: Event) -> None:
        if self._sink is not None:
            self._sink(event)

    def _tick_loop(self) -> None:
        while True:
            time.sleep(TICK_PERIOD_S)
            with self._lock:
                if not self._running:
                    return
                now = time.monotonic()
                dt = now - self._last_tick_time
                self._last_tick_time = now
                if self._moving():
                    self._left_ticks += self._v_left * dt * TICKS_PER_M
                    self._right_ticks += self._v_right * dt * TICKS_PER_M
                if (self._state == TacticalState.ACTIVE
                        and now - self._last_seen > self._liveness_timeout):
                    self._enter(TacticalState.FALLBACK)
