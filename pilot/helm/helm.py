"""The helm — a bench console that drives the airframe in captain language.

Runs on the Pi above the pilot layer, speaks plain words to the human and
the cockpit protocol downward through the Cockpit API. Its whole job is
ergonomics: you type an order once; the helm holds the latched
{speed, bank} setpoint by streaming `drive` inside the airframe's deadman
window, which a human at a raw terminal cannot do.

Vocabulary (no jargon; + is clockwise, - counterclockwise):

  arm / disarm / estop / clear      state transitions (arming is always manual)
    speed <m/s>                       select a positive linear speed (default 0.1)
    f | b                             move forward | backward at selected speed
  bank <deg/s>                      rotate at this rate, hold until changed
  turn <deg>                        rotate BY this many degrees, then stop
                                    rotating (firmware procedure)
    s                                 stop movement and rotation
  state | odom | version | geometry queries
  help | quit

Events do not print here -- the command window stays synchronous. They are
appended, timestamped, to the event log; watch it in a second window:

  tail -f ~/wanderer/events.log
"""

import argparse
import math
import os
import sys
import threading
import time
from datetime import datetime

try:
    import readline as _readline  # Enables editing/history for input() on Unix.
except ImportError:
    pass

# Runnable from pilot/ as `python3 -m helm` or `python3 helm/helm.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cockpit.api import Cockpit                              # noqa: E402
from cockpit.errors import (CockpitError, CockpitNack,       # noqa: E402
                            CockpitTimeout)
from cockpit.events import (Event, FaultRaised, ProcedureFinished,  # noqa: E402
                            StateChanged, TacticalState)
if __package__:
    from . import presets                                    # noqa: E402
else:
    import presets                                           # noqa: E402

from cockpit.uart_link import FAULT_NAMES                    # noqa: E402


class Helm:
    def __init__(self, cockpit: Cockpit, log_path: str):
        self._cockpit = cockpit
        self._log_path = os.path.expanduser(log_path)
        os.makedirs(os.path.dirname(self._log_path), exist_ok=True)
        self._log_lock = threading.Lock()
        # The latched setpoint, captain convention: positive speed magnitude,
        # direction (-1/0/+1), and clockwise-positive bank deg/s. The streamer
        # is the only writer to the wire.
        self._sp_lock = threading.Lock()
        self._speed = presets.DEFAULT_SPEED
        self._direction = 0
        self._bank_dps = 0.0
        # Engaged = the human has asserted motion intent since the last
        # SAFE/FALLBACK/FAULT. Only an engaged helm streams `drive`; a
        # disengaged one pings. This is what keeps the streamer from
        # silently resuming motion after FALLBACK (spec s5): the airframe
        # would accept our next drive, so *we* must withhold it until the
        # human speaks again.
        self._engaged = False
        self._state = TacticalState.SAFE
        self._geometry = None
        self._running = False
        self._streamer = None
        self._turn_cv = threading.Condition()
        self._turn_result = None

    # ---- lifecycle -------------------------------------------------------

    def start(self) -> None:
        self._cockpit.on_event(self._on_event)
        deadline = time.monotonic() + presets.AIRFRAME_STARTUP_TIMEOUT_S
        while True:
            try:
                self._cockpit.ping()
                break
            except CockpitTimeout:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise
                time.sleep(min(presets.AIRFRAME_STARTUP_RETRY_PERIOD_S,
                               remaining))
        v = self._cockpit.version()
        self._geometry = self._cockpit.geometry()
        self._state = self._cockpit.state()
        print(f"airframe fw {v.major}.{v.minor}  "
              f"tpm={self._geometry.ticks_per_meter:g}  "
              f"track={self._geometry.track_m:g} m  "
              f"state={self._state.name}")
        print(f"events -> {self._log_path}   (tail -f it in a second window)")
        print("type 'help' for vocabulary; arming is manual")
        self._log("[helm] session start")
        self._running = True
        self._streamer = threading.Thread(target=self._stream_loop,
                                          name="helm-stream", daemon=True)
        self._streamer.start()

    def shutdown(self) -> None:
        self._running = False
        if self._streamer is not None:
            self._streamer.join(timeout=1.0)
        # Polite exit: never leave the rover armed behind a closed console.
        try:
            if self._state in (TacticalState.ACTIVE, TacticalState.FALLBACK):
                self._cockpit.disarm()
        except CockpitError:
            pass
        self._log("[helm] session end")

    # ---- the streamer (sole writer of drive) -----------------------------

    def _stream_loop(self) -> None:
        last_ping = 0.0
        while self._running:
            time.sleep(presets.STREAM_PERIOD_S)
            try:
                if self._engaged:
                    with self._sp_lock:
                        speed, omega = self._linear_speed(), self._omega()
                    self._cockpit.drive(speed, omega)
                else:
                    now = time.monotonic()
                    if now - last_ping >= presets.IDLE_PING_PERIOD_S:
                        self._cockpit.ping()
                        last_ping = now
            except CockpitNack as e:
                if e.code in ("not_armed", "fault_latched"):
                    self._engaged = False  # airframe said no; stand down
                self._log(f"[helm] stream refused: {e.code}")
            except CockpitError as e:
                self._log(f"[helm] stream error: {e}")

    def _omega(self) -> float:
        # Captain: + is clockwise. Wire: + is counterclockwise (robotics
        # convention). The helm negates here and NOWHERE else.
        return -math.radians(self._bank_dps)

    def _linear_speed(self) -> float:
        return self._direction * self._speed

    # ---- events -> log ---------------------------------------------------

    def _on_event(self, event: Event) -> None:
        if isinstance(event, StateChanged):
            self._state = event.new
            self._log(f"!state {event.old.name} -> {event.new.name}")
            if event.new in (TacticalState.FALLBACK, TacticalState.FAULT,
                             TacticalState.SAFE):
                with self._sp_lock:
                    self._direction = 0
                    self._bank_dps = 0.0
                self._engaged = False
        elif isinstance(event, FaultRaised):
            self._log(f"!fault {FAULT_NAMES.get(event.code, event.code)}")
        elif isinstance(event, ProcedureFinished):
            detail = f" reason={event.reason}" if event.reason else ""
            self._log(f"!proc {event.name} {event.outcome}{detail}")
            with self._turn_cv:
                self._turn_result = event
                self._turn_cv.notify_all()
        else:
            self._log(f"!{event}")

    def _log(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with self._log_lock:
            with open(self._log_path, "a") as f:
                f.write(f"{stamp}  {text}\n")

    # ---- commands --------------------------------------------------------

    def command(self, line: str) -> bool:
        """Execute one console line. Returns False when it is time to quit."""
        tok = line.strip().lower().split()
        if not tok:
            return True
        try:
            return self._dispatch(tok)
        except CockpitNack as e:
            print(f"refused: {e.code}" + (f" ({e.message})" if e.message else ""))
        except CockpitError as e:
            print(f"link error: {e}")
        except (ValueError, IndexError):
            print(f"bad arguments; try 'help'")
        return True

    def _dispatch(self, tok) -> bool:
        cmd = tok[0]
        if cmd in ("quit", "exit"):
            return False
        if cmd == "help":
            print(__doc__)
        elif cmd == "arm":
            self._cockpit.arm()
            self._engaged = True
            print("armed")
        elif cmd == "disarm":
            self._stop_motion()
            self._engaged = False
            self._cockpit.disarm()
            print("safe")
        elif cmd == "estop":
            self._stop_motion()
            self._engaged = False
            self._cockpit.estop()
            print("EMERGENCY STOP — fault latched; 'clear' then 'arm' to recover")
        elif cmd == "clear":
            self._cockpit.clear_fault()
            print("fault cleared, state SAFE")
        elif cmd == "s":
            self._stop_motion()
            self._assert_intent()
            print("all stop")
        elif cmd == "speed":
            self._set_speed(float(tok[1]))
        elif cmd == "f":
            self._move(1)
        elif cmd == "b":
            self._move(-1)
        elif cmd == "bank":
            dps = float(tok[1])
            with self._sp_lock:
                self._bank_dps = dps
            self._assert_intent()
            print(f"bank {dps:g} deg/s "
                  + ("(clockwise)" if dps > 0 else
                     "(counterclockwise)" if dps < 0 else "(straight)"))
        elif cmd == "turn":
            self._turn(float(tok[1]))
        elif cmd == "state":
            st = self._cockpit.state()
            self._state = st
            print(st.name)
        elif cmd == "odom":
            o = self._cockpit.odometry()
            print(f"lt={o.left_ticks} rt={o.right_ticks} "
                  f"vl={o.left_m_s:.3f} vr={o.right_m_s:.3f}")
        elif cmd == "version":
            v = self._cockpit.version()
            print(f"fw {v.major}.{v.minor}")
        elif cmd == "geometry":
            g = self._geometry
            print(f"tpm={g.ticks_per_meter:g} track={g.track_m:g} m")
        else:
            print(f"unknown command {cmd!r}; try 'help'")
        return True

    def _set_speed(self, v: float) -> None:
        if not math.isfinite(v) or v <= 0.0:
            raise ValueError
        with self._sp_lock:
            self._speed = v
        print(f"speed set to {v:g} m/s")

    def _move(self, direction: int) -> None:
        with self._sp_lock:
            self._direction = direction
            speed = self._speed
        self._assert_intent()
        word = "forward" if direction > 0 else "backward"
        print(f"moving {word} at {speed:g} m/s")

    def _stop_motion(self) -> None:
        with self._sp_lock:
            self._direction = 0
            self._bank_dps = 0.0

    def _assert_intent(self) -> None:
        """A motion order from the human re-engages the streamer.

        This is the console-level echo of spec s5: after FALLBACK only a
        fresh drive resumes, and that drive must represent the human
        speaking again -- which is exactly what calling this from a typed
        command means.
        """
        self._engaged = True

    # ---- firmware-owned turn procedure -----------------------------------

    def _turn(self, deg: float) -> None:
        """Rotate BY `deg` degrees (+ clockwise), then stop rotating.

        The Pico closes heading from encoder ticks. Helm pauses its Tier 1
        drive stream, keeps the deadman alive with pings, and waits for the
        procedure outcome. Ctrl-C asks the firmware to abort the turn.
        """
        if deg == 0.0:
            return
        with self._sp_lock:
            linear_m_s = self._linear_speed()
        with self._turn_cv:
            self._turn_result = None
        self._engaged = False
        try:
            started = self._cockpit.start_turn(-math.radians(deg), linear_m_s)
            with self._sp_lock:
                if self._direction != 0:
                    accepted_speed = abs(started.linear_m_s)
                    if accepted_speed < self._speed:
                        self._speed = accepted_speed
                        print(f"slowing to {self._speed:g} m/s for the turn "
                              "(stays there after)")
            print(f"turning {deg:g}\u00b0 ...", flush=True)
            deadline = time.monotonic() + started.timeout_s + 1.0
            with self._turn_cv:
                while self._turn_result is None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self._turn_cv.wait(remaining)
                result = self._turn_result
            if result is None:
                self._cockpit.abort()
                print("turn outcome timed out")
            elif result.outcome == "DONE":
                print("turn complete")
            else:
                detail = f" ({result.reason})" if result.reason else ""
                print(f"turn {result.outcome.lower()}{detail}")
        except KeyboardInterrupt:
            print("\nturn aborted")
            self._cockpit.abort()
        finally:
            with self._sp_lock:
                self._bank_dps = 0.0
            self._engaged = self._state == TacticalState.ACTIVE


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="helm", description="Wanderer bench console (captain language)")
    ap.add_argument("--sim", action="store_true",
                    help="use built-in simulator mode (no UART)")
    ap.add_argument("--port", default=presets.SERIAL_DEVICE,
                    help="serial device of the cockpit UART, "
                    f"default: {presets.SERIAL_DEVICE}")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", default=presets.EVENT_LOG,
                    help=f"event log path (default {presets.EVENT_LOG})")
    args = ap.parse_args(argv)

    if args.sim:
        from cockpit.sim import SimulatedCockpitLink
        link = SimulatedCockpitLink()
        using_uart = False
        print("SIMULATOR (--sim)")
    else:
        from cockpit.uart_link import UartCockpitLink
        link = UartCockpitLink(args.port, args.baud)
        using_uart = True
        print(f"cockpit UART {args.port} @ {args.baud}")

    cockpit = Cockpit(link, command_timeout=0.25)
    helm = Helm(cockpit, args.log)
    with cockpit:
        helm.start()
        # Log lines from the real airframe also belong in the event log.
        if using_uart:
            link.set_log_sink(lambda text: helm._log(f"* {text}"))
        try:
            while True:
                try:
                    line = input(f"helm {helm._state.name}> ")
                except KeyboardInterrupt:
                    print("\n(type 'quit' to leave, 'estop' to kill motion)")
                    continue
                except EOFError:
                    break
                if not helm.command(line):
                    break
        finally:
            helm.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
