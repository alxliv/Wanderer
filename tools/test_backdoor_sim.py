"""Regression test for tools/backdoor.py, against a simulated airframe.

No hardware: a fake serial port speaks the backdoor line protocol and models a
rover with known, deliberately asymmetric deadbands and an inverted left
encoder. The test then asserts the calibration recovers exactly those numbers,
which is the only way to know the sweep logic is right without a rover on the
bench (on real hardware there is no ground truth to compare against).

    python tools/test_backdoor_sim.py

Defaults to ./backdoor.py next to this file; pass a path to override.
"""
import sys, types, importlib.util

# --- ground truth we expect the tool to discover ---------------------------
TRUE = {"left_fwd": 120, "left_rev": 140, "right_fwd": 180, "right_rev": 190}
# What config.h holds. The sim reports these via `cfg` AND applies them to its
# counts, exactly as the firmware does -- so with correct wiring a forward
# command raises the reported count whatever the sign is. LEFT_AGREES/
# RIGHT_AGREES say whether the configured sign actually matches the hardware.
LEFT_SIGN, RIGHT_SIGN = -1, 1
LEFT_AGREES, RIGHT_AGREES = True, False   # right sign is WRONG and must flip
MOT_LEFT_SIGN, MOT_RIGHT_SIGN = 1, -1     # what config.h holds for the motors
TICKS_PER_MS_AT_FULL = 4.0


class FakeSerial:
    def __init__(self, *a, **kw):
        self.out = []
        self.lt = 500          # nonzero start: the tool must use deltas
        self.rt = -300
        self.dev = False

    # -- serial API ---------------------------------------------------------
    def reset_input_buffer(self): self.out.clear()
    def flush(self): pass
    def close(self): pass

    def readline(self):
        return self.out.pop(0).encode() if self.out else b""

    def write(self, data):
        line = data.decode().strip()
        self._handle(line)
        return len(data)

    # -- simulated firmware -------------------------------------------------
    def _reply(self, s): self.out.append(s + "\r\n")

    def _handle(self, line):
        tok = line.split()
        if not tok:
            return
        verb = tok[0].lower()
        if verb == "ver":
            self._reply("=ok ver fw=0.3 iface=backdoor max_duty=600 max_ms=3000")
        elif verb == "dev":
            self.dev = (tok[1].lower() == "on")
            self._reply("=ok dev")
        elif verb == "cfg":
            self._reply(f"=ok cfg enc_left_sign={LEFT_SIGN} enc_right_sign={RIGHT_SIGN} "
                        f"motor_left_sign={MOT_LEFT_SIGN} motor_right_sign={MOT_RIGHT_SIGN} "
                        f"ticks_per_m=10000.0 max_speed_mm_s=600")
        elif verb == "enc":
            self._reply(f"=ok enc left={self.lt} right={self.rt}")
        elif verb == "help":
            # The payload rides on `*` lines; the `=ok` carries nothing.
            self._reply("*backdoor verbs: dev on|off, wiggle <l> <r> <ms>, enc [reset],")
            self._reply("*  estop, safe, ver, help")
            self._reply("=ok help")
        elif verb == "estop":
            self._reply("=ok estop")
        elif verb == "wiggle":
            l, r, ms = int(tok[1]), int(tok[2]), int(tok[3])
            l = max(-600, min(600, l)); r = max(-600, min(600, r))
            ms = min(3000, ms)
            if not self.dev:
                self._reply("=err wiggle dev_inactive dev on first"); return
            self._reply(f"=ok wiggle l={l} r={r} ms={ms}")
            self._move(l, r, ms)
            self._reply("!wiggle_done timeout")
        else:
            self._reply(f"=err {verb} unknown_command")

    def _move(self, l, r, ms):
        db_l = TRUE["left_fwd"] if l > 0 else TRUE["left_rev"]
        db_r = TRUE["right_fwd"] if r > 0 else TRUE["right_rev"]
        if abs(l) >= db_l:
            self.lt += int((1 if LEFT_AGREES else -1) * (l / 600) * TICKS_PER_MS_AT_FULL * ms)
        if abs(r) >= db_r:
            self.rt += int((1 if RIGHT_AGREES else -1) * (r / 600) * TICKS_PER_MS_AT_FULL * ms)


# --- load the tool with serial stubbed and sleeps neutered -----------------
fake_serial = types.ModuleType("serial")
fake_serial.Serial = FakeSerial
tools_mod = types.ModuleType("serial.tools")
lp = types.ModuleType("serial.tools.list_ports")
lp.comports = lambda: []
tools_mod.list_ports = lp
fake_serial.tools = tools_mod
sys.modules["serial"] = fake_serial
sys.modules["serial.tools"] = tools_mod
sys.modules["serial.tools.list_ports"] = lp

import os
HERE = os.path.dirname(os.path.abspath(__file__))
tool = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "backdoor.py")
spec = importlib.util.spec_from_file_location("backdoor", tool)
bdmod = importlib.util.module_from_spec(spec)
sys.modules["backdoor"] = bdmod       # dataclasses needs the module registered
spec.loader.exec_module(bdmod)
bdmod.time.sleep = lambda *_: None       # run the sweep at full speed


class Args:
    # max is deliberately unset: the tool has to learn the ceiling from the
    # banner, the same way it does against real firmware.
    start, max, step, fine = 20, None, 20, 5
    pulse_ms, settle, min_ticks, wiring_duty = 400, 0.0, 15, 400
    rotation_ms, skip_rotation_check = 3000, False


args = Args()
bd = bdmod.Backdoor("SIM")
print(f"airframe: {bd.ver()}")
bdmod.apply_limits(bd, args)
bd.dev(True)

# ---- phase 0: scripted operator answers -----------------------------------
# The rotation check is the one step no machine can perform, so the test
# drives it the only way it can: by scripting the human.
answers: list[str] = []
bdmod.input = lambda prompt="": answers.pop(0)

print("\n--- phase 0, both wheels confirmed good ---")
answers[:] = ["yes", "yes"]
res0 = bdmod.Results()
ok = bdmod.check_rotation(bd, args, res0)

print("\n--- phase 0, RIGHT wheel reversed ---")
answers[:] = ["yes", "n"]
res_bad = bdmod.Results()
bad_ok = bdmod.check_rotation(bd, args, res_bad)

res = bdmod.Results()
bdmod.check_wiring(bd, args, res)
bdmod.measure_deadbands(bd, args, res)
bdmod.report(res, args)

print("\n" + "=" * 68)
print("SIMULATION SELF-CHECK")
print("=" * 68)
fails = 0


def check(cond, msg):
    global fails
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        fails += 1


check(bd.request("help") == "=ok help", "help acknowledged")
check(len(bd.notes) == 2 and bd.notes[0].startswith("*backdoor verbs"),
      f"help payload kept, not discarded: {bd.notes}")
check(bd.max_duty == 600, f"max_duty read from the banner: {bd.max_duty}")
check(bd.max_ms == 3000, f"max_ms read from the banner: {bd.max_ms}")
check(args.max == 600, f"sweep ceiling taken from the board: {args.max}")
check(ok is True, "phase 0 passes when both wheels are confirmed forward")
check(res0.left_rotation_ok and res0.right_rotation_ok, "both wheel verdicts recorded")
check(bad_ok is False, "phase 0 ABORTS the run when a wheel is reversed")
check(res_bad.right_rotation_ok is False, "reversed wheel recorded as such")
check(res_bad.cfg_motor_right_sign == MOT_RIGHT_SIGN,
      f"read configured motor signs from cfg: {res_bad.cfg_motor_right_sign}")
check(res.cfg_left_sign == LEFT_SIGN and res.cfg_right_sign == RIGHT_SIGN,
      f"read configured signs from cfg: {res.cfg_left_sign}, {res.cfg_right_sign}")
check(res.left_agrees is LEFT_AGREES, f"left agreement {res.left_agrees} == {LEFT_AGREES}")
check(res.right_agrees is RIGHT_AGREES, f"right agreement {res.right_agrees} == {RIGHT_AGREES}")
# The whole point: a correct sign must be LEFT ALONE, a wrong one NEGATED.
check(res.left_sign == LEFT_SIGN,
      f"correct left sign preserved: {res.left_sign} == {LEFT_SIGN} (not flipped)")
check(res.right_sign == -RIGHT_SIGN,
      f"wrong right sign negated: {res.right_sign} == {-RIGHT_SIGN}")
check(not res.swapped, "wiring not reported swapped")
check(not res.crosstalk, "no spurious crosstalk")
for k, truth in TRUE.items():
    got = res.deadband.get(k)
    # The sweep resolves to the fine step, so landing within one fine step
    # above the true breakaway is a correct answer, not a miss.
    ok = got is not None and truth <= got < truth + Args.fine
    check(ok, f"{k}: measured {got}, true {truth} (fine step {Args.fine})")

print(f"\n{'ALL SIMULATION CHECKS PASSED' if fails == 0 else f'{fails} FAILURES'}")
sys.exit(1 if fails else 0)
