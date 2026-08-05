"""The helm — a bench console that drives the airframe in captain language.

Runs on the Pi above the pilot layer, speaks plain words to the human and
the cockpit protocol downward through the Cockpit API. Its whole job is
ergonomics: you type an order once; the helm holds the latched
{speed, bank} setpoint by streaming `drive` inside the airframe's deadman
window, which a human at a raw terminal cannot do.

Vocabulary (no jargon; + is clockwise, - counterclockwise):

  arm / disarm / estop / clear      state transitions (arming is always manual)
  speed <m/s>                       set forward speed (negative = reverse)
  full | half | slow                telegraph presets (presets.py)
  back full | back half | back slow reverse presets
  bank <deg/s>                      rotate at this rate, hold until changed
  turn <deg>                        rotate BY this many degrees, then stop
                                    rotating  [placeholder: closed from Linux
                                    over odometry until firmware Tier 2 lands]
  stop                              speed 0, bank 0
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

# Runnable as `python3 -m helm` from pilot/, like the tests.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cockpit.api import Cockpit                              # noqa: E402
from cockpit.errors import CockpitError, CockpitNack         # noqa: E402
from cockpit.events import (Event, FaultRaised, StateChanged,  # noqa: E402
                            TacticalState)
from helm import presets                                     # noqa: E402

from cockpit.uart_link import FAULT_NAMES                    # noqa: E402


class Helm:
    def __init__(self, cockpit: Cockpit, log_path: str):
        self._cockpit = cockpit
        self._log_path = os.path.expanduser(log_path)
        os.makedirs(os.path.dirname(self._log_path), exist_ok=True)
        self._log_lock = threading.Lock()
        # The latched setpoint, captain convention: speed m/s, bank deg/s
        # clockwise-positive. The streamer is the only writer to the wire.
        self._sp_lock = threading.Lock()
        self._speed = 0.0
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

    # ---- lifecycle -------------------------------------------------------

    def start(self) -> None:
        self._cockpit.on_event(self._on_event)
        self._cockpit.ping()
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
                        speed, omega = self._speed, self._omega()
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

    # ---- events -> log ---------------------------------------------------

    def _on_event(self, event: Event) -> None:
        if isinstance(event, StateChanged):
            self._state = event.new
            self._log(f"!state {event.old.name} -> {event.new.name}")
            if event.new in (TacticalState.FALLBACK, TacticalState.FAULT,
                             TacticalState.SAFE):
                with self._sp_lock:
                    self._speed = self._bank_dps = 0.0
                self._engaged = False
        elif isinstance(event, FaultRaised):
            self._log(f"!fault {FAULT_NAMES.get(event.code, event.code)}")
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
            self._set_speed_bank(0.0, 0.0)
            self._engaged = False
            self._cockpit.disarm()
            print("safe")
        elif cmd == "estop":
            self._set_speed_bank(0.0, 0.0)
            self._engaged = False
            self._cockpit.estop()
            print("EMERGENCY STOP — fault latched; 'clear' then 'arm' to recover")
        elif cmd == "clear":
            self._cockpit.clear_fault()
            print("fault cleared, state SAFE")
        elif cmd == "stop":
            self._set_speed_bank(0.0, 0.0)
            self._assert_intent()
            print("all stop")
        elif cmd == "speed":
            self._set_speed(float(tok[1]))
        elif cmd == "full":
            self._set_speed(presets.FULL_SPEED)
        elif cmd == "half":
            self._set_speed(presets.HALF_SPEED)
        elif cmd == "slow":
            self._set_speed(presets.SLOW_SPEED)
        elif cmd == "back":
            sub = tok[1]
            self._set_speed(-{"full": presets.FULL_SPEED,
                              "half": presets.HALF_SPEED,
                              "slow": presets.SLOW_SPEED}[sub])
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
        with self._sp_lock:
            self._speed = v
        self._assert_intent()
        word = "ahead" if v > 0 else "back" if v < 0 else "stopped"
        print(f"speed {v:g} m/s ({word})")

    def _set_speed_bank(self, v: float, b: float) -> None:
        with self._sp_lock:
            self._speed, self._bank_dps = v, b

    def _assert_intent(self) -> None:
        """A motion order from the human re-engages the streamer.

        This is the console-level echo of spec s5: after FALLBACK only a
        fresh drive resumes, and that drive must represent the human
        speaking again -- which is exactly what calling this from a typed
        command means.
        """
        self._engaged = True

    # ---- the turn maneuver (placeholder for firmware Tier 2) -------------

    def _turn(self, deg: float) -> None:
        """Rotate BY `deg` degrees (+ clockwise), then stop rotating.

        PLACEHOLDER implementation: closes the heading loop from Linux by
        polling odometry -- exactly what the architecture forbids for real
        operation (Linux jitter, UART in the loop). Acceptable on the bench
        at TURN_SPEED/TURN_RATE only; replaced by the firmware Tier 2
        `proc turn` in the next firmware milestone. Ctrl-C aborts the turn
        (stops rotation) without leaving the console.
        """
        if deg == 0.0:
            return
        g = self._geometry
        with self._sp_lock:
            if abs(self._speed) > presets.TURN_SPEED:
                self._speed = math.copysign(presets.TURN_SPEED, self._speed)
                print(f"slowing to {self._speed:g} m/s for the turn "
                      "(stays there after)")
            self._bank_dps = math.copysign(presets.TURN_RATE_DPS, deg)
        self._assert_intent()

        target = max(abs(deg) - presets.OVERSHOOT_COMP_DEG, 0.0)
        start = self._cockpit.odometry()
        deadline = time.monotonic() + abs(deg) / presets.TURN_RATE_DPS + 3.0
        print(f"turning {deg:g}\u00b0 [placeholder] ...", flush=True)
        try:
            while True:
                time.sleep(0.05)
                turned = self._heading_cw_deg(start)
                # Progress in the commanded direction only.
                progress = turned if deg > 0 else -turned
                if progress >= target:
                    break
                if time.monotonic() > deadline:
                    print("turn timed out (not rotating? check state/odom)")
                    break
        except KeyboardInterrupt:
            print("\nturn aborted")
        finally:
            with self._sp_lock:
                self._bank_dps = 0.0
        # One more poll after rotation stop for the honest number.
        time.sleep(2 * presets.STREAM_PERIOD_S)
        print(f"turned {self._heading_cw_deg(start):+.1f}\u00b0")

    def _heading_cw_deg(self, start) -> float:
        """Heading change since `start`, degrees, clockwise-positive.

        Differential odometry: theta_ccw = (right - left) / track, so
        clockwise is (left - right) / track. Geometry comes from the
        airframe (get_geometry), never from a pilot-side copy.
        """
        o = self._cockpit.odometry()
        g = self._geometry
        left_m = (o.left_ticks - start.left_ticks) / g.ticks_per_meter
        right_m = (o.right_ticks - start.right_ticks) / g.ticks_per_meter
        return math.degrees((left_m - right_m) / g.track_m)


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
