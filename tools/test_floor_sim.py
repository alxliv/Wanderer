"""Regression test for tools/backdoor.py --calibrate-floor, no hardware.

A fake serial port models a rover with a KNOWN wheel and a KNOWN loaded
rolling radius, creeping in chained pulses exactly as the real one does, with
a scripted operator supplying ticks/rev, wheel diameter and the tape reading.

Checks the two things that could silently be wrong: the seed arithmetic
(ticks_per_rev / pi*D) and the final result (ticks / tape distance).

    python tools/test_floor_sim.py
"""

import importlib.util
import os
import sys
import types

# --- ground truth ----------------------------------------------------------
TICKS_PER_REV = 2400.0
WHEEL_DIAMETER_MM = 65.0          # free diameter, what a caliper reads
DEFLECTION = 0.03                 # loaded tyre rolls on 3% less radius
import math as _m
TRUE_CIRC_MM = _m.pi * WHEEL_DIAMETER_MM * (1.0 - DEFLECTION)
TRUE_TICKS_PER_M = TICKS_PER_REV / (TRUE_CIRC_MM / 1000.0)
SEED_TICKS_PER_M = TICKS_PER_REV / (_m.pi * WHEEL_DIAMETER_MM / 1000.0)
# Speed model: mm/s = SLOPE * permille + INTERCEPT, zero below the LOADED
# deadband -- higher than the raised-bench figure, because on the floor the
# wheels carry the chassis.
SLOPE = 0.72
INTERCEPT = -40.0
DEADBAND = 140


def speed_mm_s(permille):
    d = abs(permille)
    if d < DEADBAND:
        return 0.0
    return SLOPE * d + INTERCEPT


class FakeSerial:
    """Speaks the backdoor protocol and integrates distance in real time.

    Tick counts advance according to how long the caller actually waits, so
    the tool's own mid-run sampling logic is what gets exercised -- not a
    replayed transcript.
    """

    def __init__(self, *a, **kw):
        self.out = []
        self.timeout = kw.get("timeout", 2.0)
        self.dev = False
        self.lt = self.rt = 0
        self.duty = 0
        self.t_start = None
        self.t_end = None
        self.last = None
        self.pending_done = False

    def hand_turn(self, ticks, wheel):
        """Simulate the operator turning one wheel by hand."""
        if wheel == "left":
            self.lt += int(ticks)
        else:
            self.rt += int(ticks)

    def reset_input_buffer(self): self.out.clear()
    def flush(self): pass
    def close(self): pass

    def readline(self):
        import time as _t
        if self.out:
            return self.out.pop(0).encode()
        # A wiggle is outstanding: the real firmware emits !wiggle_done only
        # when its deadline expires, so wait it out here too. Returning the
        # event early would let the caller sample a pulse that had not
        # finished, which is exactly the bug this models away.
        if self.pending_done:
            remaining = self.t_end - _t.monotonic()
            if remaining > 0:
                _t.sleep(remaining)
            self._advance()
            self.pending_done = False
            return b"!wiggle_done timeout\r\n"
        return b""

    def _reply(self, s): self.out.append(s + "\r\n")

    def _advance(self):
        """Integrate ticks up to now (or to the end of the wiggle)."""
        import time as _t
        if self.t_start is None:
            return
        now = min(_t.monotonic(), self.t_end)
        if self.last is None:
            self.last = self.t_start
        dt = max(0.0, now - self.last)
        self.last = now
        v = speed_mm_s(self.duty)
        sign = 1 if self.duty >= 0 else -1
        ticks = sign * v / 1000.0 * TRUE_TICKS_PER_M * dt
        self.lt += int(ticks)
        self.rt += int(ticks)

    def write(self, data):
        import time as _t
        line = data.decode().strip()
        tok = line.split()
        if not tok:
            return len(data)
        verb = tok[0].lower()
        if verb == "ver":
            self._reply("=ok ver fw=0.3 iface=backdoor max_duty=600 max_ms=3000")
        elif verb == "cfg":
            self._reply("=ok cfg enc_left_sign=-1 enc_right_sign=1 "
                        "motor_left_sign=1 motor_right_sign=1 "
                        "ticks_per_m=10000.0 max_speed_mm_s=600")
        elif verb == "dev":
            self.dev = tok[1].lower() == "on"
            self._reply("=ok dev")
        elif verb == "enc":
            if len(tok) > 1 and tok[1].lower() == "reset":
                self._advance()
                self.lt = self.rt = 0
                self._reply("=ok enc left=0 right=0")
            else:
                self._advance()
                self._reply(f"=ok enc left={self.lt} right={self.rt}")
        elif verb == "wiggle":
            l, ms = int(tok[1]), min(3000, int(float(tok[3])))
            l = max(-600, min(600, l))
            self.duty = l
            self.t_start = _t.monotonic()
            self.t_end = self.t_start + ms / 1000.0
            self.last = self.t_start
            self._reply(f"=ok wiggle l={l} r={l} ms={ms}")
            self.pending_done = True
        elif verb == "estop":
            self._reply("=ok estop")
        else:
            self._reply(f"=err {verb} unknown_command")
        return len(data)


fake = types.ModuleType("serial")
fake.Serial = FakeSerial
tools_mod = types.ModuleType("serial.tools")
lp = types.ModuleType("serial.tools.list_ports")
lp.comports = lambda: []
tools_mod.list_ports = lp
fake.tools = tools_mod
sys.modules["serial"] = fake
sys.modules["serial.tools"] = tools_mod
sys.modules["serial.tools.list_ports"] = lp

HERE = os.path.dirname(os.path.abspath(__file__))
tool = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "backdoor.py")
spec = importlib.util.spec_from_file_location("backdoor", tool)
bd_mod = importlib.util.module_from_spec(spec)
sys.modules["backdoor"] = bd_mod
spec.loader.exec_module(bd_mod)


class Args:
    turns = 5
    target_mm = 1000.0
    roll_duty, roll_pulse_ms, roll_max_pulses = 250, 300, 40
    veer_tolerance = 0.05
    yes = True


bd = bd_mod.Backdoor("SIM")
fake_port = bd.ser
bd.ver()
bd.dev(True)

pending = {"distance": None}

# The scripted operator. Walks the same prompts a human would see, so the
# wizard's own flow is exercised -- including the hand-turning step, which is
# faked by telling the FakeSerial to advance its counters between the "ready"
# and "done" prompts.
hand_turn_pending = {"wheel": None}


def scripted(prompt=""):
    p = prompt.lower()
    if "currently set" in p:
        return "n"
    if "diameter" in p:
        return str(WHEEL_DIAMETER_MM)
    if "distance in mm" in p:
        return f"{pending['distance']:.1f}"
    if "type s" in p:
        return "s"
    if "turns are done" in p:
        # The operator has just turned the wheel by hand: put the ticks in.
        fake_port.hand_turn(TICKS_PER_REV * Args.turns, hand_turn_pending["wheel"])
        return ""
    if "ready to start counting" in p:
        # Which wheel is being asked for is in the preceding printed text;
        # alternate, left first.
        hand_turn_pending["wheel"] = ("left" if hand_turn_pending["wheel"] is None
                                      else "right")
        return ""
    if "press enter" in p:
        return ""
    return "yes"


bd_mod.input = scripted

# The "tape measure": whatever ticks were actually counted, converted at the
# TRUE loaded ticks/m. The tool must not see this number any other way.
_real_creep = bd_mod.creep_to_ticks


def creep_and_measure(client, args, res, target):
    ok = _real_creep(client, args, res, target)
    pending["distance"] = ((res.left_ticks + res.right_ticks) / 2.0
                           / TRUE_TICKS_PER_M * 1000.0)
    return ok


bd_mod.creep_to_ticks = creep_and_measure

res = bd_mod.FloorResults()
res.cfg_ticks_per_m = bd.cfg_ticks_per_m()

ok1 = (bd_mod.collect_geometry(bd, Args(), res)
       and bd_mod.measure_floor(bd, Args(), res))
bd_mod.report_floor(res, Args())

print("\n" + "=" * 68)
print("FLOOR SIMULATION SELF-CHECK")
print("=" * 68)
fails = 0


def check(cond, msg):
    global fails
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        fails += 1


check(ok1, "measurement completed")
check(res.ticks_per_rev and abs(res.ticks_per_rev[0] - TICKS_PER_REV) < 1.0,
      f"counted {res.ticks_per_rev[0]:.0f} ticks/rev by hand-turning"
      f" (true {TICKS_PER_REV:.0f})")
check(abs(res.seed_ticks_per_m - SEED_TICKS_PER_M) < 1.0,
      f"seed from geometry {res.seed_ticks_per_m:.0f}"
      f" == pi*D arithmetic {SEED_TICKS_PER_M:.0f}")
check(abs(res.ticks_per_m - TRUE_TICKS_PER_M) / TRUE_TICKS_PER_M < 0.01,
      f"measured ticks/m {res.ticks_per_m:.0f} within 1% of true"
      f" {TRUE_TICKS_PER_M:.0f}")

check(abs(res.distance_mm - Args.target_mm) / Args.target_mm < 0.15,
      f"rolled roughly the target distance: {res.distance_mm:.0f} mm"
      f" of {Args.target_mm:.0f} mm")
check(res.mean_speed_mm_s and res.mean_speed_mm_s > 0,
      f"mean speed reported: {res.mean_speed_mm_s:.0f} mm/s")
check(res.ticks_per_m > res.seed_ticks_per_m,
      "measured ticks/m exceeds the seed, the physically correct direction")

print(f"\n{'ALL FLOOR CHECKS PASSED' if fails == 0 else f'{fails} FAILURES'}")
sys.exit(1 if fails else 0)
