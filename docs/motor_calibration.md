# Motor Calibration — the airframe backdoor over USB CDC

How to calibrate and test the Wanderer's motors from a Windows PC, using the
**system backdoor** (architecture §3a) carried on the Pico 2's USB CDC port.
No Pi5, no radio, no UART adapter — one USB cable.

Companion to [cockpit_bench_test.md](cockpit_bench_test.md), which drives the
*cockpit* by hand. This document drives the *maintenance hatch*.

---

## 1. What the backdoor is, and what it deliberately is not

The backdoor is a privileged, out-of-band maintenance path. Its verb list is
**closed**: every verb belongs to one of the three categories §3a admits, and
`firmware/common/backdoor_handler.h` names the category next to each one. There
is no streaming motion verb and there never will be — normal motion flies
through the cockpit (Pi5 / UART), full stop.

| Verb | Category | Effect |
|---|---|---|
| `estop` | emergency override | Latch FAULT. Honored always, from any state, with no lease. |
| `safe` | emergency override | Disarm to SAFE, stop any wiggle. |
| `dev on` / `dev off` | the authority gate | Acquire / release the motion lease. |
| `wiggle <l> <r> <ms>` | dev diagnostic | Raw per-mille duty, both wheels, self-terminating. |
| `enc` / `enc reset` | dev diagnostic | Read / zero the raw encoder counts. |
| `cfg` | dev diagnostic | Report the compiled-in `ENC_*_SIGN`, `MOTOR_*_SIGN`, ticks/m, max speed. |
| `ver` | dev diagnostic | Firmware version and the bench rails. |
| `help` | dev diagnostic | List the above. |

Replies use the cockpit's line grammar (`=ok verb fields…`, `=err verb reason`),
so the same parser reads both interfaces. `*` lines are bench logs; `!` lines
are events.

## 2. The authority gate

`wiggle` is the only motion-capable verb, and it is refused unless the backdoor
holds the **motion lease**. `dev on` grants the lease only when *all* of these
hold:

- the FSM is in **SAFE** (not armed),
- **no fault** is latched,
- **no cockpit commander is live** — nothing valid has arrived on UART0 within
  `LIVENESS_TIMEOUT_MS` (750 ms).

The lease is revoked the instant any cockpit frame arrives, and by any fault.
Revocation stops an in-flight wiggle and emits `!wiggle_done lease_lost`, so a
truncated sweep step announces itself rather than quietly producing a wrong
deadband reading. While the lease is held, `arm` is refused: the aircraft does
not become flyable while ground crew has the hatch open.

Net effect: **the Base can never wiggle a wheel the Pilot believes it owns.**

## 3. Safety rails

Enforced in firmware, not in the tool — the operator cannot raise them from the
wire:

- duty clamped to ±`BACKDOOR_MAX_DUTY_PERMILLE` (600 = 60%)
- duration clamped to `BACKDOOR_MAX_WIGGLE_MS` (3000 ms)
- every wiggle **self-terminates**; nothing latches the wheels on
- USB disconnect calls `backdoor_abort()` → wheels stop, lease dropped
- out-of-range values are **clamped and reported**, so the `=ok` reply always
  states what was actually applied

Both ceilings are advertised in the `ver` banner (`max_duty=… max_ms=…`), and
`tools/backdoor.py` reads them from there rather than keeping its own copy — so
reflashing with different rails needs no change to the tool.

## 4. Build and flash

```sh
cmake --build firmware/build --target wanderer_airframe
```

Hold BOOTSEL while connecting the Pico 2, copy
`firmware/build/airframe/wanderer_airframe.uf2` to the mass-storage device.

The backdoor lives in the **flight firmware** — there is no separate bench
binary to flash and un-flash. On boot it prints:

```
*airframe fw 0.3 cockpit on pico2 uart0 @115200
*backdoor on usb cdc -- type `help`
```

## 5. Prepare the robot

1. Motor power **off** while handling the robot or changing wiring.
2. Raise and securely support the chassis so both wheels rotate freely.
   Never run these tests with the drive wheels touching the floor.
3. Keep hands, cables, tools and loose clothing clear of the wheels.
4. Disconnect the Pi5 (or stop its cockpit process) — a live commander will
   refuse the lease, by design.
5. Be ready to remove motor power immediately.

The MDD10A has no motor-supply status line to the Pico, so firmware cannot
prove motor power is present. `dev on` is the operator's assertion that the
bench is safe.

## 6. Automatic calibration

```sh
pip install pyserial
python tools/backdoor.py --calibrate
```

The port is autodetected by Raspberry Pi VID (`0x2E8A`); override with
`--port COM5`. The tool confirms the chassis is raised, acquires the lease,
and runs three phases:

**Phase 0 — wheel rotation direction (the operator answers).** Runs the left
wheel alone for 3 s, stops, and asks whether that rotation would move the
vehicle **forward, toward bearing 0** (see architecture 2a for the body
frame). Then the same for the right wheel.

This step cannot be automated and it cannot be skipped. No sensor on the robot
knows which way the *vehicle* would go: an encoder counting up says the wheel
is turning, not that the chassis would advance. An operator's eye is the only
instrument available, and every later number depends on the answer -- the
deadband sweep, the encoder-sign verdict and the eventual ticks-per-metre roll
all assume a positive command drives toward bearing 0.

A "no" **aborts the run before anything is measured** and names the constant to
change (`MOTOR_LEFT_SIGN` / `MOTOR_RIGHT_SIGN` in `config.h`), with the value
to write. Measuring a reversed wheel produces numbers that look entirely
plausible and mean nothing, so the tool refuses to produce them.

`--rotation-ms` changes the run length; `--skip-rotation-check` bypasses the
phase entirely and is only safe once polarity is known good.

**Phase 1 — encoder wiring and direction.** Drives each wheel alone at 400‰ and
watches which encoder responds and in which direction. It first reads `cfg` for
the configured `ENC_*_SIGN`, because `enc` counts are already sign-corrected --
without that, a rising count cannot be told apart from a sign that happens to be
`+1`, and the tool would cheerfully suggest inverting a working encoder. Detects
three distinct
faults that are easy to confuse on the bench: a swapped encoder pair (one wheel
moves the *other* counter), a dead channel, and chassis movement (both counters
respond to a single-wheel command, meaning the robot is not properly
restrained). Produces `ENC_LEFT_SIGN` / `ENC_RIGHT_SIGN`.

**Phase 2 — deadband sweep.** For each wheel and each direction, steps duty
upward from rest in coarse steps until the encoder moves, then refines
downward in fine steps. Every pulse starts from a **stopped** wheel, because
breakaway is a static-friction threshold — measuring from a rolling start reads
low.

Output ends with a config.h-ready block. Sign lines appear **only when a sign
must change** -- a correct one is reported as "leave it alone" rather than
printed as a define, so it cannot be pasted in the wrong sense:

```
Suggested config.h changes:
  /* Both ENC_*_SIGN are already correct -- do not change them. */
  #define MOTOR_DEADBAND_LEFT  120
  #define MOTOR_DEADBAND_RIGHT 180
```

Deadbands are reported for both directions and both wheels, with the L/R skew
called out separately for forward and reverse, since a rover can be nearly
symmetric one way and badly skewed the other.

**The numbers are UNLOADED.** Wheels raised, carrying nothing: on the floor,
under the chassis' weight, real breakaway is higher. Treat them as a lower
bound and a wiring verdict, not as final feed-forward constants.

Useful knobs: `--step` / `--fine` (sweep resolution), `--pulse-ms`,
`--min-ticks` (what counts as movement), `--verbose` (echo the wire traffic).
`--max` defaults to the ceiling the board reports, and both it and `--pulse-ms`
are capped to what that board will honour.

### Why the asymmetry number matters

If the left wheel breaks away at 120‰ and the right at 180‰, then any commanded
straight line below 180‰ turns the robot: one wheel spins, one sits still. This
is the usual cause of "it veers at slow speed", and it is invisible to the
open-loop mm/s mapping, which believes both wheels are obeying. It also
destabilizes the PID loop waiting in `config.h` — a velocity controller
integrating error against a stalled wheel lurches when the output finally
clears the threshold.

## 7. Manual console

```sh
python tools/backdoor.py
```

```
backdoor> ver
=ok ver fw=0.3 iface=backdoor max_duty=600 max_ms=3000
backdoor> dev on
=ok dev
backdoor> enc reset
=ok enc left=0 right=0
backdoor> wiggle 300 0 500
=ok wiggle l=300 r=0 ms=500
!wiggle_done timeout
backdoor> enc
=ok enc left=1042 right=0
backdoor> dev off
=ok dev
```

PuTTY or any serial terminal works equally well — the protocol terminates a
line on either CR or LF, so no line-ending configuration is needed. Ctrl-C in
the tool sends `estop`.

## 8. Testing without hardware

```sh
python tools/test_backdoor_sim.py      # calibration logic vs. a simulated rover
```

Fakes a serial port and a rover with known deadbands, then asserts the sweep
recovers exactly those numbers — the only way to check the sweep logic is
correct, since a real rover offers no ground truth to compare against.

Firmware-side, the gate and rails are covered on the host:

```sh
cmake -S firmware/common/tests -B build_common_tests
cmake --build build_common_tests
ctest --test-dir build_common_tests -C Debug
```

`test_backdoor` covers lease refusal (commander live, armed, faulted),
mid-wiggle revocation, clamping, self-termination, ESTOP-always-honored, and
that cockpit verbs such as `drive` and `arm` are *not* reachable through the
backdoor.

## 9. Floor calibration — ticks per metre

```sh
python tools/backdoor.py --calibrate-floor
```

**No arguments.** It asks for everything it needs. Bring a tape measure and a
clear straight metre of hard floor.

This turns encoder ticks into a physical length. Until it is done,
`DEFAULT_TICKS_PER_METER` is a placeholder `10000.0f` and every distance and
velocity the cockpit reports is scaled by a guess.

Four guided steps:

| Step | Rover | What you do |
|---|---|---|
| 1. Rotation direction | raised | Watch each wheel run, answer yes/no |
| 2. Ticks per turn | raised | Turn each wheel 5 times by hand |
| 3. Wheel diameter | raised | Measure across the wheel, type it in |
| 4. The roll | **on the floor** | Type `s`, let it drive, measure how far |

### Step 2 — ticks per turn

The tool zeroes the counter, tells you to turn that wheel five full turns by
hand, reads the counter back, and shows you both the raw tick count and the
ticks-per-revolution. You confirm the number looks right, or redo the wheel.

Five turns rather than one because your error in judging "back to the mark"
divides by the count. It also warns if the count is zero (encoder not read),
negative (turned the wrong way), or if the *other* wheel's counter moved.

Measuring at the wheel makes this independent of gearbox ratio and encoder
PPR — whatever sits between motor and rim, the composite is what gets counted.

### Step 3 — diameter

Measured across the wheel through the centre. Combined with step 2 this gives
a rough ticks-per-metre: `ticks_per_rev / (π · D)`. It only decides how far to
roll. The real number comes from your tape.

### Step 4 — the roll

The tool tells you exactly what is about to happen, then waits for you to type
**`s`** — not a bare Enter, since the next thing that happens is the rover
driving off.

It creeps forward at 250‰ in short pulses, printing distance and speed, stops
itself after about a metre, and asks you for the actual distance travelled.
Measure to the *same* reference point on the rover you started from.

`ticks/m = mean ticks ÷ your measured distance`. Overshoot past the internal
target does not matter — the tape is what counts.

If the first pulse produces no ticks, the duty is below the *loaded* deadband:
on the floor the wheels carry the chassis, so breakaway is higher than the
raised-bench figure in §6. Raise `--roll-duty`.

### Checks it performs

- left vs right ticks/rev more than 2% apart — recount
- left vs right ticks during the roll more than 5% apart — it did not run
  straight, so one distance describes neither wheel's path
- implied rolling diameter, inverted back from the result and compared against
  the wheel you measured

Paste the result into `config.h` and reflash.

### Testing without hardware

```sh
python tools/test_floor_sim.py
```

Walks the same prompts with a scripted operator against a simulated rover,
including the hand-turning step, and checks the arithmetic.

## 10. Still not calibrated

- **`DEFAULT_MAX_SPEED_MM_S`** (600) — needs a sustained run near full duty,
  which needs more floor than a USB cable reaches. Waits for the Pi5 to carry
  the tool, driven over ssh, untethered.
- **`TRACK_WIDTH_M`** (0.30f) — needs a measured *rotation* rather than a
  straight roll. Its own procedure, not yet automated.
