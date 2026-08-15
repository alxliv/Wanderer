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

import math
import threading
import time
from typing import Callable, Optional

from . import link as _link
from .errors import CockpitLinkError, CockpitNack
from .events import (Event, FaultRaised, ProcedureFinished, StateChanged,
                     TacticalState)
from .link import CockpitLink, Reply, Request

FAULT_ESTOP = 1
TICKS_PER_M = 10000.0
TICK_PERIOD_S = 0.02  # 50 Hz FSM tick, plenty for liveness resolution

# The rover's real geometry, from firmware/airframe/src/config.h:
# TRACK_WIDTH_M / 2, and DEFAULT_MAX_SPEED_MM_S / 1000.
HALF_TRACK_M = 0.1004867
MAX_WHEEL_M_S = 0.6
TURN_RATE_RAD_S = 0.5235988
TURN_OVERSHOOT_RAD = 0.0349066
TURN_LINEAR_LIMIT_M_S = 0.2
HEADING_KP = 2.0
HEADING_KI = 0.5
HEADING_KD = 0.1
HEADING_I_MAX_RAD_S = 0.3
HEADING_OMEGA_MAX_RAD_S = 0.5
MOTOR_LEFT_GAIN_PERMILLE = 1000
MOTOR_RIGHT_GAIN_PERMILLE = 841
MOTOR_DEADBAND_LEFT = 80
MOTOR_DEADBAND_RIGHT = 40


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class SimulatedCockpitLink(CockpitLink):
    def __init__(self, *, liveness_timeout_s: float = 0.75,
                  half_track_m: float = HALF_TRACK_M,
                  max_wheel_m_s: float = MAX_WHEEL_M_S,
                  gyro_yaw_scale: float = 1.0,
                  yaw_disturbance_rad_s: float = 0.0):
        self._liveness_timeout = liveness_timeout_s
        self._half_track = half_track_m
        self._max_wheel = max_wheel_m_s
        self._gyro_yaw_scale = gyro_yaw_scale
        self._yaw_disturbance = yaw_disturbance_rad_s
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
        self._turn_active = False
        self._turn_angle = 0.0
        self._turn_linear = 0.0
        self._turn_last_heading = 0.0
        self._turn_progress = 0.0
        self._turn_deadline = 0.0
        self._move_active = False
        self._move_distance = 0.0
        self._move_linear = 0.0
        self._move_start_left = 0.0
        self._move_start_right = 0.0
        self._move_heading_ref = 0.0
        self._move_integral = 0.0
        self._move_last_control = 0.0
        self._move_deadline = 0.0
        self._heading = 0.0
        self._heading_rate = 0.0
        self._heading_bias = 0.0
        self._heading_valid = True
        self._imu_healthy = True

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
            self._finish_turn("ABORTED", "disarm", restore=False)
            self._finish_move("ABORTED", "disarm")
            self._enter(TacticalState.SAFE)
            return Reply()
        if op == _link.OP_ESTOP:
            if self._state == TacticalState.FAULT:
                return Reply()  # re-raise: first cause preserved, no events
            self._finish_turn("ABORTED", "estop", restore=False)
            self._finish_move("ABORTED", "estop")
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
            if self._turn_active or self._move_active:
                raise CockpitNack("busy", "procedure active")
            # A fresh drive is the ONLY thing that resumes from FALLBACK.
            if self._state == TacticalState.FALLBACK:
                self._enter(TacticalState.ACTIVE)
            if self._state != TacticalState.ACTIVE:
                raise CockpitNack("not_armed", f"state is {self._state.name}")
            v, w = p["linear_m_s"], p["angular_rad_s"]
            left = v + w * self._half_track
            right = v - w * self._half_track
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
            self._finish_turn("ABORTED", "stop", restore=False)
            self._finish_move("ABORTED", "stop")
            self._v_left = self._v_right = 0.0
            return Reply()
        if op == _link.OP_PROC:
            if "distance_m" in p:
                return self._start_move(float(p["distance_m"]),
                                        float(p["linear_m_s"]))
            return self._start_turn(float(p["angle_rad"]),
                                    float(p["linear_m_s"]))
        if op == _link.OP_ABORT:
            self._finish_turn("ABORTED", "command", restore=True)
            self._finish_move("ABORTED", "command")
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
        if op == _link.OP_GET_HEADING:
            return Reply({"psi_rad": self._heading,
                          "rate_rad_s": self._heading_rate,
                          "bias_rad_s": self._heading_bias,
                          "valid": self._heading_valid})
        if op == _link.OP_ZERO_HEADING:
            if not self._heading_valid:
                raise CockpitNack("imu_not_ready", "")
            self._heading = 0.0
            return Reply()
        if op == _link.OP_GET_VERSION:
            return Reply({"major": 0, "minor": 15})
        if op == _link.OP_GET_GEOMETRY:
            # The airframe owns its geometry (spec section 3); the sim, as the
            # firmware's mirror, reports the same numbers it simulates with.
            return Reply({"ticks_per_meter": TICKS_PER_M,
                           "track_m": 2.0 * self._half_track})
        if op == _link.OP_GET_MOTOR_CONFIG:
            return Reply({"left_gain_permille": MOTOR_LEFT_GAIN_PERMILLE,
                          "right_gain_permille": MOTOR_RIGHT_GAIN_PERMILLE,
                          "left_deadband_permille": MOTOR_DEADBAND_LEFT,
                          "right_deadband_permille": MOTOR_DEADBAND_RIGHT})
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

    def _start_turn(self, angle: float, linear: float) -> Reply:
        if not math.isfinite(angle) or not math.isfinite(linear) or angle == 0.0:
            raise CockpitNack("bad_args", "expected finite nonzero angle")
        if self._state == TacticalState.FALLBACK:
            self._enter(TacticalState.ACTIVE)
        if self._state != TacticalState.ACTIVE:
            raise CockpitNack("not_armed", f"state is {self._state.name}")
        if not self._heading_valid or not self._imu_healthy:
            raise CockpitNack("imu_not_ready", "")
        if self._turn_active:
            self._finish_turn("SUPERSEDED", restore=True)
        if self._move_active:
            self._finish_move("SUPERSEDED", "")
        linear = max(-TURN_LINEAR_LIMIT_M_S,
                     min(TURN_LINEAR_LIMIT_M_S, linear))
        omega = TURN_RATE_RAD_S if angle > 0.0 else -TURN_RATE_RAD_S
        left = linear + omega * self._half_track
        right = linear - omega * self._half_track
        scale = self._drive_scale(left, right)
        self._v_left = left * scale
        self._v_right = right * scale
        self._turn_active = True
        self._turn_angle = angle
        self._turn_linear = linear
        self._turn_last_heading = self._heading
        self._turn_progress = 0.0
        timeout_s = abs(angle) / (TURN_RATE_RAD_S * scale) + 3.0
        self._turn_deadline = time.monotonic() + timeout_s
        return Reply({"linear_m_s": linear, "timeout_s": timeout_s})

    def _finish_turn(self, outcome: str, reason: str = "", *,
                     restore: bool) -> None:
        if not self._turn_active:
            return
        self._turn_active = False
        if restore and self._state == TacticalState.ACTIVE:
            self._v_left = self._v_right = self._turn_linear
        elif self._state == TacticalState.ACTIVE:
            self._v_left = self._v_right = 0.0
        self._emit(ProcedureFinished(name="turn", outcome=outcome,
                                     reason=reason))

    def _tick_turn(self, now: float) -> None:
        if not self._turn_active:
            return
        if self._state != TacticalState.ACTIVE:
            reason = "deadman" if self._state == TacticalState.FALLBACK else "state"
            self._finish_turn("ABORTED", reason, restore=False)
            return
        if not self._heading_valid or not self._imu_healthy:
            self._finish_turn("ABORTED", "imu_stale", restore=False)
            return
        self._turn_progress += _wrap_pi(self._heading - self._turn_last_heading)
        self._turn_last_heading = self._heading
        progress = (self._turn_progress if self._turn_angle > 0.0
                    else -self._turn_progress)
        target = max(abs(self._turn_angle) - TURN_OVERSHOOT_RAD, 0.0)
        if progress >= target:
            self._finish_turn("DONE", restore=True)
        elif now >= self._turn_deadline:
            self._finish_turn("ABORTED", "timeout", restore=True)

    def _start_move(self, distance: float, linear: float) -> Reply:
        if (not math.isfinite(distance) or not math.isfinite(linear)
                or distance == 0.0 or linear == 0.0
                or (distance < 0.0) != (linear < 0.0)):
            raise CockpitNack("bad_args", "distance and linear must agree")
        if self._state != TacticalState.ACTIVE:
            raise CockpitNack("not_armed", f"state is {self._state.name}")
        if not self._heading_valid or not self._imu_healthy:
            raise CockpitNack("imu_not_ready", "")
        if self._turn_active:
            self._finish_turn("SUPERSEDED", restore=True)
        if self._move_active:
            self._finish_move("SUPERSEDED", "")
        linear *= self._drive_scale(linear, linear)
        self._v_left = self._v_right = linear
        self._move_active = True
        self._move_distance = distance
        self._move_linear = linear
        self._move_start_left = self._left_ticks
        self._move_start_right = self._right_ticks
        self._move_heading_ref = self._heading
        self._move_integral = 0.0
        self._move_last_control = time.monotonic()
        timeout_s = abs(distance / linear) + 3.0
        self._move_deadline = self._move_last_control + timeout_s
        return Reply({"linear_m_s": linear, "timeout_s": timeout_s})

    def _finish_move(self, outcome: str, reason: str) -> None:
        if not self._move_active:
            return
        self._move_active = False
        if self._state == TacticalState.ACTIVE:
            self._v_left = self._v_right = 0.0
        self._emit(ProcedureFinished(name="move", outcome=outcome,
                                      reason=reason))

    def _tick_move(self, now: float) -> None:
        if not self._move_active:
            return
        if self._state != TacticalState.ACTIVE:
            reason = "deadman" if self._state == TacticalState.FALLBACK else "state"
            self._finish_move("ABORTED", reason)
            return
        if not self._heading_valid or not self._imu_healthy:
            self._finish_move("ABORTED", "imu_stale")
            return
        travelled = ((self._left_ticks - self._move_start_left)
                     + (self._right_ticks - self._move_start_right)) / (2.0 * TICKS_PER_M)
        progress = travelled if self._move_distance > 0.0 else -travelled
        if progress >= abs(self._move_distance):
            self._finish_move("DONE", "")
            return
        if now >= self._move_deadline:
            self._finish_move("ABORTED", "timeout")
            return
        dt = now - self._move_last_control
        self._move_last_control = now
        error = _wrap_pi(self._move_heading_ref - self._heading)
        if 0.0 < dt < 0.1:
            integral_limit = HEADING_I_MAX_RAD_S / HEADING_KI
            self._move_integral = max(-integral_limit, min(
                integral_limit, self._move_integral + error * dt))
        omega = (HEADING_KP * error + HEADING_KI * self._move_integral
                 - HEADING_KD * self._heading_rate)
        omega = max(-HEADING_OMEGA_MAX_RAD_S,
                    min(HEADING_OMEGA_MAX_RAD_S, omega))
        left = self._move_linear + omega * self._half_track
        right = self._move_linear - omega * self._half_track
        scale = self._drive_scale(left, right)
        self._v_left, self._v_right = left * scale, right * scale

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
                    encoder_yaw_rate = ((self._v_left - self._v_right)
                                        / (2.0 * self._half_track))
                    self._heading_rate = (encoder_yaw_rate * self._gyro_yaw_scale
                                          + self._yaw_disturbance)
                    self._heading = _wrap_pi(self._heading
                                             + self._heading_rate * dt)
                    self._left_ticks += self._v_left * dt * TICKS_PER_M
                    self._right_ticks += self._v_right * dt * TICKS_PER_M
                else:
                    self._heading_rate = 0.0
                if (self._state == TacticalState.ACTIVE
                        and now - self._last_seen > self._liveness_timeout):
                    self._enter(TacticalState.FALLBACK)
                self._tick_turn(now)
                self._tick_move(now)
