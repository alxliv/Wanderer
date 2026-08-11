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
and runs two phases:

**Phase 1 — encoder wiring and direction.** Drives each wheel alone at 400‰ and
watches which encoder responds and in which direction. Detects three distinct
faults that are easy to confuse on the bench: a swapped encoder pair (one wheel
moves the *other* counter), a dead channel, and chassis movement (both counters
respond to a single-wheel command, meaning the robot is not properly
restrained). Produces `ENC_LEFT_SIGN` / `ENC_RIGHT_SIGN`.

**Phase 2 — deadband sweep.** For each wheel and each direction, steps duty
upward from rest in coarse steps until the encoder moves, then refines
downward in fine steps. Every pulse starts from a **stopped** wheel, because
breakaway is a static-friction threshold — measuring from a rolling start reads
low.

Output ends with a config.h-ready block:

```
Suggested config.h additions:
  #define ENC_LEFT_SIGN      -1
  #define ENC_RIGHT_SIGN     +1
  #define MOTOR_DEADBAND_LEFT  120
  #define MOTOR_DEADBAND_RIGHT 180
```

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

## 9. What this does not calibrate

`DEFAULT_TICKS_PER_METER` (a placeholder 10000.0f) and `DEFAULT_MAX_SPEED_MM_S`
(600, marked "calibrate") both need a **known-distance roll on the floor**, not
a raised bench, so they are deliberately out of scope here. `TRACK_WIDTH_M`
likewise needs a measured rotation. Those come after the wheels are known to
turn correctly.
