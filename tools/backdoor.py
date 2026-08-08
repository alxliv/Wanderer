#!/usr/bin/env python3
"""
Wanderer airframe backdoor client and automatic motor calibration.

Talks the system-backdoor line protocol (architecture 3a) to the Pico 2 over
USB CDC. Two modes:

    python tools/backdoor.py                    # interactive console
    python tools/backdoor.py --calibrate        # automatic calibration run

The calibration run measures, without the operator touching anything:

  * encoder wiring    -- which encoder belongs to which wheel, and whether
                         forward motion counts up (ENC_LEFT_SIGN / ENC_RIGHT_SIGN)
  * deadband          -- the lowest per-mille duty at which each wheel actually
                         breaks away, per wheel and per direction
  * L/R asymmetry     -- the difference between the two, which is what makes a
                         rover veer at low speed

It prints a config.h-ready block at the end.

SAFETY -- read before running with motor power on:
  The wheels WILL turn. Raise and securely support the chassis so both wheels
  spin free. Keep hands, cables and clothing clear. Every pulse is bounded by
  the firmware to BACKDOOR_MAX_WIGGLE_MS, and Ctrl-C sends `estop`, but the
  physical guard is the one that matters.

Requires pyserial:  pip install pyserial
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")


# Raspberry Pi vendor ID; the Pico 2 enumerates its CDC interface under it.
RPI_VID = 0x2E8A

# Firmware rails, mirrored from firmware/common/backdoor_handler.h. The tool
# never needs to enforce these (the firmware clamps and says so in its reply)
# but sweeping past them would just waste time on identical results.
MAX_DUTY_PERMILLE = 600
MAX_WIGGLE_MS = 3000


# --------------------------------------------------------------------------
# transport
# --------------------------------------------------------------------------

class Backdoor:
    """Line-oriented client. Every request returns the matching reply line."""

    def __init__(self, port: str, timeout: float = 2.0, verbose: bool = False):
        # USB CDC ignores baud rate; 115200 is convention, not configuration.
        self.ser = serial.Serial(port, 115200, timeout=timeout)
        self.verbose = verbose
        self.events: list[str] = []
        time.sleep(0.3)             # let the port settle after opening
        self.ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def _readline(self) -> str:
        raw = self.ser.readline()
        if not raw:
            raise TimeoutError("no reply from the airframe")
        line = raw.decode("utf-8", errors="replace").strip()
        if self.verbose and line:
            print(f"    < {line}")
        return line

    def request(self, line: str) -> str:
        """Send a request; skip `*` logs and `!` events until the reply."""
        if self.verbose:
            print(f"    > {line}")
        self.ser.write((line + "\r\n").encode("ascii"))
        self.ser.flush()
        while True:
            got = self._readline()
            if got.startswith("=ok") or got.startswith("=err"):
                return got
            if got.startswith("!"):
                self.events.append(got)
            # `*` bench logs and blanks fall through and are ignored

    def await_event(self, prefix: str, timeout: float = 6.0) -> str:
        """Block until an event line with `prefix` arrives."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # Drain anything already queued from an earlier step.
            for i, ev in enumerate(self.events):
                if ev.startswith(prefix):
                    return self.events.pop(i)
            try:
                got = self._readline()
            except TimeoutError:
                continue
            if got.startswith(prefix):
                return got
            if got.startswith("!"):
                self.events.append(got)
        raise TimeoutError(f"no {prefix} event within {timeout}s")

    # -- verbs --------------------------------------------------------------

    def ver(self) -> str:
        return self.request("ver")

    def dev(self, on: bool) -> str:
        return self.request(f"dev {'on' if on else 'off'}")

    def estop(self) -> str:
        return self.request("estop")

    def enc(self) -> tuple[int, int]:
        reply = self.request("enc")
        if not reply.startswith("=ok"):
            raise RuntimeError(f"enc refused: {reply}")
        fields = dict(tok.split("=", 1) for tok in reply.split()[2:] if "=" in tok)
        return int(fields["left"]), int(fields["right"])

    def wiggle(self, left: int, right: int, ms: int) -> str:
        reply = self.request(f"wiggle {left} {right} {ms}")
        if not reply.startswith("=ok"):
            raise RuntimeError(f"wiggle refused: {reply}")
        return reply


def find_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    candidates = [p for p in list_ports.comports() if p.vid == RPI_VID]
    if not candidates:
        ports = ", ".join(p.device for p in list_ports.comports()) or "none"
        sys.exit(f"No Raspberry Pi USB device found. Ports seen: {ports}\n"
                 f"Pass one explicitly with --port COM5")
    if len(candidates) > 1:
        listing = ", ".join(f"{p.device} ({p.description})" for p in candidates)
        sys.exit(f"Several Pi USB devices found: {listing}\nPick one with --port")
    return candidates[0].device


# --------------------------------------------------------------------------
# measurement primitives
# --------------------------------------------------------------------------

@dataclass
class Pulse:
    """One bench pulse and what the encoders did during it."""
    duty: int
    wheel: str          # "left" | "right" | "both"
    left_delta: int
    right_delta: int

    def moved(self, wheel: str, min_ticks: int) -> bool:
        d = self.left_delta if wheel == "left" else self.right_delta
        return abs(d) >= min_ticks


def pulse(bd: Backdoor, wheel: str, duty: int, ms: int, settle: float) -> Pulse:
    """Command one pulse from rest and report the tick deltas it produced.

    Always starts from a stopped wheel: breakaway is a STATIC friction
    threshold, and measuring it from a rolling start would read low.
    """
    time.sleep(settle)                      # let the wheel come fully to rest
    l0, r0 = bd.enc()
    left = duty if wheel in ("left", "both") else 0
    right = duty if wheel in ("right", "both") else 0
    bd.wiggle(left, right, ms)
    bd.await_event("!wiggle_done", timeout=ms / 1000.0 + 4.0)
    time.sleep(0.15)                        # let the last encoder edges land
    l1, r1 = bd.enc()
    return Pulse(duty, wheel, l1 - l0, r1 - r0)


def sweep_deadband(bd: Backdoor, wheel: str, sign: int, args) -> tuple[int | None, list[Pulse]]:
    """Walk duty upward until the wheel breaks away, then refine.

    Coarse pass finds the bracket cheaply; a fine pass inside that bracket
    gets the number without spending a pulse on every single per-mille step.
    """
    history: list[Pulse] = []
    coarse_hit = None

    for duty in range(args.start, args.max + 1, args.step):
        p = pulse(bd, wheel, sign * duty, args.pulse_ms, args.settle)
        history.append(p)
        print(f"    {wheel:<5} {sign*duty:>+5}  ->  L{p.left_delta:>+7}  R{p.right_delta:>+7}")
        if p.moved(wheel, args.min_ticks):
            coarse_hit = duty
            break

    if coarse_hit is None:
        return None, history

    # Refine downward from the hit, in fine steps, back toward the last
    # known-still duty. The lowest duty that still moves is the answer.
    lower = max(args.start, coarse_hit - args.step)
    best = coarse_hit
    for duty in range(lower, coarse_hit, args.fine):
        p = pulse(bd, wheel, sign * duty, args.pulse_ms, args.settle)
        history.append(p)
        moved = p.moved(wheel, args.min_ticks)
        print(f"    {wheel:<5} {sign*duty:>+5}  ->  L{p.left_delta:>+7}  R{p.right_delta:>+7}"
              f"   {'move' if moved else 'still'}")
        if moved:
            best = duty
            break

    return best, history


# --------------------------------------------------------------------------
# calibration run
# --------------------------------------------------------------------------

@dataclass
class Results:
    left_sign: int | None = None
    right_sign: int | None = None
    swapped: bool = False
    crosstalk: list[str] = field(default_factory=list)
    deadband: dict[str, int | None] = field(default_factory=dict)


def check_wiring(bd: Backdoor, args, res: Results) -> None:
    """Drive each wheel alone at a duty well above any plausible deadband and
    see which encoder responds, and in which direction."""
    print("\n[1/2] encoder wiring and direction")
    probe = min(args.max, max(args.wiring_duty, args.start))

    left_probe = pulse(bd, "left", probe, args.pulse_ms, args.settle)
    print(f"    left  wheel @ +{probe}  ->  L{left_probe.left_delta:>+7}  R{left_probe.right_delta:>+7}")
    right_probe = pulse(bd, "right", probe, args.pulse_ms, args.settle)
    print(f"    right wheel @ +{probe}  ->  L{right_probe.left_delta:>+7}  R{right_probe.right_delta:>+7}")

    lt = args.min_ticks

    # Did the wheel we commanded move its own encoder?
    left_ok = abs(left_probe.left_delta) >= lt
    right_ok = abs(right_probe.right_delta) >= lt

    # Did it move the OTHER one instead? That is swapped wiring, and it is
    # worth naming explicitly -- it is the failure that looks like "the robot
    # turns when told to go straight" and gets misdiagnosed as a deadband.
    if not left_ok and abs(left_probe.right_delta) >= lt:
        res.swapped = True
    if not right_ok and abs(right_probe.left_delta) >= lt:
        res.swapped = True

    if res.swapped:
        print("    !! commanding one wheel moved the OTHER encoder.")
        print("       Encoder pairs are swapped: exchange ENC_LEFT_PIN_BASE and")
        print("       ENC_RIGHT_PIN_BASE in config.h (or swap the plugs), then rerun.")
        return

    if not left_ok:
        print(f"    !! left wheel: no encoder response at {probe}. Deadband above the")
        print("       probe duty, motor unpowered, or encoder unplugged.")
    if not right_ok:
        print(f"    !! right wheel: no encoder response at {probe}. Same causes.")

    # A positive command must produce positive ticks. If it does not, the
    # existing sign constant is inverted relative to the hardware.
    if left_ok:
        res.left_sign = 1 if left_probe.left_delta > 0 else -1
    if right_ok:
        res.right_sign = 1 if right_probe.right_delta > 0 else -1

    # Crosstalk: both encoders moving on a single-wheel command usually means
    # the chassis is not restrained and the whole robot is shifting.
    if left_ok and abs(left_probe.right_delta) >= lt:
        res.crosstalk.append("left command moved the right encoder too")
    if right_ok and abs(right_probe.left_delta) >= lt:
        res.crosstalk.append("right command moved the left encoder too")


def measure_deadbands(bd: Backdoor, args, res: Results) -> None:
    print("\n[2/2] deadband sweep (breakaway duty, from rest)")
    for wheel in ("left", "right"):
        for label, sign in (("fwd", 1), ("rev", -1)):
            key = f"{wheel}_{label}"
            print(f"  {wheel} {label}:")
            value, _hist = sweep_deadband(bd, wheel, sign, args)
            res.deadband[key] = value
            if value is None:
                print(f"    -> no movement up to {args.max}")
            else:
                print(f"    -> breakaway {value} permille")


def report(res: Results, args) -> None:
    print("\n" + "=" * 68)
    print("CALIBRATION RESULT")
    print("=" * 68)

    if res.swapped:
        print("\nEncoders are cross-wired. Fix the wiring, then rerun; the deadband")
        print("numbers below (if any) cannot be trusted until it is corrected.")

    print("\nEncoder direction:")
    for name, sign in (("ENC_LEFT_SIGN", res.left_sign), ("ENC_RIGHT_SIGN", res.right_sign)):
        if sign is None:
            print(f"  {name:<16} undetermined (wheel never moved)")
        else:
            print(f"  {name:<16} {sign:+d}")

    if res.crosstalk:
        print("\nWarning -- encoder crosstalk observed:")
        for c in res.crosstalk:
            print(f"  * {c}")
        print("  The chassis is probably not held still. Re-secure it and rerun.")

    print("\nDeadband (per-mille duty at which the wheel breaks away):")
    print(f"  {'':<8}{'forward':>10}{'reverse':>10}")
    for wheel in ("left", "right"):
        f = res.deadband.get(f"{wheel}_fwd")
        r = res.deadband.get(f"{wheel}_rev")
        fs = str(f) if f is not None else "--"
        rs = str(r) if r is not None else "--"
        print(f"  {wheel:<8}{fs:>10}{rs:>10}")

    known = [v for v in res.deadband.values() if v is not None]
    lf, rf = res.deadband.get("left_fwd"), res.deadband.get("right_fwd")
    if lf is not None and rf is not None:
        skew = abs(lf - rf)
        stronger = "left" if lf < rf else "right"
        print(f"\nL/R asymmetry (forward): {skew} permille"
              f" -- the {stronger} wheel breaks away first.")
        if skew >= args.step:
            print("  At low speed a straight-line command will veer toward the")
            print(f"  {'right' if stronger == 'left' else 'left'} until both wheels are turning.")

    if known:
        worst = max(known)
        print("\nSuggested config.h additions:")
        print(f"  /* Measured {time.strftime('%Y-%m-%d')} with tools/backdoor.py. */")
        if res.left_sign is not None:
            print(f"  #define ENC_LEFT_SIGN      {res.left_sign:+d}")
        if res.right_sign is not None:
            print(f"  #define ENC_RIGHT_SIGN     {res.right_sign:+d}")
        print(f"  #define MOTOR_DEADBAND_LEFT  {res.deadband.get('left_fwd') or 0}")
        print(f"  #define MOTOR_DEADBAND_RIGHT {res.deadband.get('right_fwd') or 0}")
        print(f"  /* Lowest duty at which BOTH wheels reliably turn: {worst}. Commands")
        print(f"     below this move nothing; feed-forward past it or refuse them. */")

    print("\nNot measured by this run: DEFAULT_TICKS_PER_METER and")
    print("DEFAULT_MAX_SPEED_MM_S both need a known-distance roll on the floor.")


def calibrate(bd: Backdoor, args) -> int:
    print(f"airframe: {bd.ver()}")

    reply = bd.dev(True)
    if not reply.startswith("=ok"):
        print(f"\nCould not acquire dev mode: {reply}")
        if "commander_present" in reply:
            print("A cockpit commander is live on UART0. Disconnect the Pi5 (or stop")
            print("its cockpit process), wait a second, and rerun.")
        elif "not_safe" in reply:
            print("The FSM is armed. Send `safe` first.")
        elif "fault_latched" in reply:
            print("A fault is latched. Clear it over the cockpit before benching.")
        return 1
    print("dev mode acquired -- the backdoor holds the motion lease\n")

    res = Results()
    try:
        check_wiring(bd, args, res)
        if not res.swapped:
            measure_deadbands(bd, args, res)
    finally:
        bd.dev(False)
        print("\ndev mode released.")

    report(res, args)
    return 0


# --------------------------------------------------------------------------
# interactive console
# --------------------------------------------------------------------------

def console(bd: Backdoor) -> int:
    print(f"airframe: {bd.ver()}")
    print("Type backdoor verbs; `help` lists them, Ctrl-D or `quit` exits.\n")
    while True:
        try:
            line = input("backdoor> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not line:
            continue
        if line in ("quit", "exit"):
            return 0
        try:
            print(bd.request(line))
            while bd.events:
                print(bd.events.pop(0))
        except TimeoutError as e:
            print(f"! {e}")


# --------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Wanderer airframe backdoor client and motor calibration.")
    ap.add_argument("--port", help="serial port (default: autodetect the Pico)")
    ap.add_argument("--calibrate", action="store_true",
                    help="run the automatic calibration instead of the console")
    ap.add_argument("--verbose", "-v", action="store_true", help="echo the wire traffic")
    ap.add_argument("--yes", action="store_true",
                    help="skip the wheels-off-the-ground confirmation")

    sweep = ap.add_argument_group("sweep parameters")
    sweep.add_argument("--start", type=int, default=20, help="first duty tried (default 20)")
    sweep.add_argument("--max", type=int, default=MAX_DUTY_PERMILLE,
                       help=f"highest duty tried (default {MAX_DUTY_PERMILLE})")
    sweep.add_argument("--step", type=int, default=20, help="coarse step (default 20)")
    sweep.add_argument("--fine", type=int, default=5, help="refinement step (default 5)")
    sweep.add_argument("--pulse-ms", type=int, default=400,
                       help=f"pulse length, capped at {MAX_WIGGLE_MS} (default 400)")
    sweep.add_argument("--settle", type=float, default=0.6,
                       help="rest time before each pulse (default 0.6s)")
    sweep.add_argument("--min-ticks", type=int, default=15,
                       help="tick delta that counts as movement (default 15)")
    sweep.add_argument("--wiring-duty", type=int, default=400,
                       help="duty used for the wiring check (default 400)")
    args = ap.parse_args()

    args.pulse_ms = min(args.pulse_ms, MAX_WIGGLE_MS)
    args.max = min(args.max, MAX_DUTY_PERMILLE)

    port = find_port(args.port)
    print(f"Opening {port}")

    if args.calibrate and not args.yes:
        print("\n  The wheels will turn. Confirm the chassis is raised and securely")
        print("  supported, both wheels spin free, and motor power is ON.")
        if input("  Type YES to proceed: ").strip() != "YES":
            print("Aborted.")
            return 1

    bd = Backdoor(port, verbose=args.verbose)
    try:
        return calibrate(bd, args) if args.calibrate else console(bd)
    except KeyboardInterrupt:
        print("\ninterrupted -- sending estop")
        try:
            print(bd.estop())
        except Exception:
            pass
        return 130
    except (TimeoutError, RuntimeError) as e:
        print(f"\nerror: {e}")
        try:
            bd.estop()
        except Exception:
            pass
        return 1
    finally:
        bd.close()


if __name__ == "__main__":
    sys.exit(main())
