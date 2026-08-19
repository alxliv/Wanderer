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

This converts encoder ticks into a physical length. Until it is done,
`DEFAULT_TICKS_PER_METER` is only a rough estimate, and every distance and
velocity the cockpit reports is scaled by that guess.

The procedure has four guided steps:

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

### Mandatory order: floor before wheel-response measurement

Speed measurement depends on a correct `DEFAULT_TICKS_PER_METER` because the backdoor
converts encoder tick deltas into `mm/s` using that constant. If the geometry
calibration is still only a rough estimate, the sweep will suggest gains
for the wrong speed scale.

Do the sequence in this order:

1. `--calibrate` for signs and deadbands
2. `--calibrate-floor` for a measured ticks-per-metre value
3. `--pid-tune` for open-loop wheel-response measurement
4. validate on the floor with a short straight run

## 10. Wheel-response measurement and PI starting values

This measures each wheel's open-loop duty-to-speed response and calculates
starting values for the proportional-integral (PI) helper. The flight motor
path maps target speed directly to duty and does not call that helper.

Keep the rover raised so the wheels spin freely and do not touch the floor.

#### Active motor path

The flow is:

- `Cockpit.drive()` sends a wheel-speed target over UART
- the Pico stores the left/right target in the tactical layer
- each 10 ms tick samples the encoders
- `DEFAULT_MAX_SPEED_MM_S` maps each target speed to motor duty
- `motors_set()` applies per-wheel deadband and gain

The `--pid-tune` backdoor sweep also uses raw duty. Its CSV records evidence
for controller design without allowing feedback to hide motor mismatch.

#### Before you tune PID

Do these first:

1. Run the raised-wheel calibration and fix any sign or wiring mistakes:

   ```sh
   python tools/backdoor.py --calibrate
   ```

2. Run the floor calibration to populate a correct `DEFAULT_TICKS_PER_METER`:

   ```sh
   python tools/backdoor.py --calibrate-floor
   ```

3. Confirm neither wheel is stalled by a bad deadband value.
4. Keep the chassis raised and the wheels free before each sweep.
5. Disable the cockpit/driver so the backdoor is the only motion source.

#### Tune each wheel individually

The new arguments are easy to confuse, so here is the exact meaning:

- `--pid-targets` is the list of motor command values, not a speed target. These are values in per-mille duty, from 0 to 1000. `100` means 10% duty, `200` means 20% duty, `300` means 30%, and so on. The tool sends each one for a short pulse, measures the resulting wheel speed from the encoder, and prints the result for each duty. It does not generate a plot in this script; the CSV file is the data you review or plot later if you want.
- `--pid-pulse-ms` is how long each duty pulse lasts. A longer pulse gives the wheel more time to accelerate and settle, so the measured speed is more stable.
- `--pid-settle` is the rest time before each pulse. It lets the wheel come fully to rest before the next command, because static friction is higher than rolling friction. Without a settle time, the measured speed is distorted by the wheel not being at rest.
- `--pid-wheel` chooses which wheel to sweep: `left`, `right`, or `both`.
- `--pid-out` writes the raw results to CSV so you can compare the response curves later.

Run the built-in sweep helper:

```sh
python tools/backdoor.py --pid-tune --pid-wheel left --pid-targets 100,200,300,400,500,600 --pid-pulse-ms 800 --pid-settle 0.6 --pid-out left_pid.csv
```

Then the right wheel:

```sh
python tools/backdoor.py --pid-tune --pid-wheel right --pid-targets 100,200,300,400,500,600 --pid-pulse-ms 800 --pid-settle 0.6 --pid-out right_pid.csv
```

For a full sweep of both wheels together:

```sh
python tools/backdoor.py --pid-tune --pid-wheel both --pid-targets 100,200,300,400,500,600 --pid-pulse-ms 800 --pid-settle 0.6 --pid-out pid_sweep.csv
```

##### What the tool measures

The tool runs a short pulse at each duty level, reads the encoder delta for that pulse, converts it to wheel speed in millimetres per second, and prints a suggested starting value.

The conversion mirrors the firmware:

```text
speed_mm_s = tick_delta * 1e9 / (ticks_per_meter * elapsed_us)
```

The CSV contains the raw sweep data:

```csv
wheel,target_duty_permille,measured_mm_s,delta_ticks,elapsed_us
left,100,42,14,800000
```

The important numbers are:

- `target_duty`: the command sent to that wheel, in permille
- `measured_mm_s`: the speed inferred from encoder counts
- `delta_ticks`: the encoder movement during that pulse

#### Compare the CSVs in Excel

The CSV is the record you use later to compare one wheel against another or one tuning pass against the next. There is no built-in plot in the tool; Excel is the normal way to inspect the curve.

1. Save the output files from each wheel:

   ```sh
   python tools/backdoor.py --pid-tune --pid-wheel left --pid-targets 100,200,300,400,500,600 --pid-pulse-ms 800 --pid-settle 0.6 --pid-out left_pid.csv
   python tools/backdoor.py --pid-tune --pid-wheel right --pid-targets 100,200,300,400,500,600 --pid-pulse-ms 800 --pid-settle 0.6 --pid-out right_pid.csv
   ```

2. Open Excel.
3. In Excel, choose **Data → From Text/CSV** and load `left_pid.csv`.
4. In the import dialog, keep the comma delimiter and accept the headers. The table should have the columns:

   - `wheel`
   - `target_duty_permille`
   - `measured_mm_s`
   - `delta_ticks`
   - `elapsed_us`

5. Repeat for `right_pid.csv`. You can place both sets in separate sheets or in the same sheet.
6. To compare the curves, make a chart with:

   - X-axis = `target_duty_permille`
   - Y-axis = `measured_mm_s`

7. In Excel, select the two columns for the left wheel, then choose **Insert → Scatter or Line Chart**. Repeat for the right wheel, then overlay the two series on the same chart.
8. If you want the cleanest comparison, keep the left and right series in the same chart and label them clearly.

A good response curve is monotonic: speed rises as duty rises, without large spikes or repeated oscillation. If one side stays much lower than the other at the same duty, the deadband or open-loop motor gains are still mismatched.

If you want a quick sanity check before saving the tune, compare these points:

- `100` vs `200` duty: does the measured speed rise smoothly?
- `300` vs `400` duty: is the curve consistent?
- `500` vs `600` duty: is there overshoot or a sudden flattening?

The CSV is the evidence. If the curve is jagged or one wheel is clearly different, rerun the sweep before you change the firmware constants.

#### Interpret the result

The printed values are only starting points. They are intentionally conservative, not a final tune.

##### Typical starting point

The tool suggests a small, stable loop:

```c
#define MOTOR_LEFT_PID_KP   0.5f
#define MOTOR_LEFT_PID_KI   0.1f
#define MOTOR_LEFT_PID_KD   0.0f
```

and similarly for the right wheel.

Do not assume both sides share the same values. The left and right drivetrains are not identical; gear mesh, wheel slip, friction and the motor/encoder assembly can differ enough that one side needs a different gain.

##### What each PID coefficient means

The isolated PI helper calculates:

```text
feedforward = target_mm_s * 1000 / DEFAULT_MAX_SPEED_MM_S
command = feedforward + Kp * speed_error_mm_s
                      + Ki * integral(speed_error_mm_s * seconds)
```

A zero target returns zero duty and clears the accumulated error. Output
saturation also stops the integral from growing farther into the limit.

- `Kp` is the immediate response in duty per mm/s of speed error. If the target is 300 mm/s and the wheel is at 200 mm/s, `Kp = 0.5` adds 50 duty.
- `Ki` corrects persistent error using elapsed time, so its effect is independent of the 100 Hz sample rate. Too much `Ki` makes the wheel hunt or oscillate.
- `Kd` reacts to how fast the error is changing. It is useful when the wheel suddenly speeds up or slows down and you want to damp the response. Example: if the wheel is accelerating too hard and overshooting, `Kd` pushes back. In this project, start with `Kd = 0.0f` unless the wheel clearly overshoots and oscillates even with a moderate `Kp` and `Ki`.

A good rule is: `Kp` gives the first push, `Ki` fixes persistent under-speed, and `Kd` damps the correction. In this rover, start with `Kd = 0.0f` and tune it only if needed.

#### Tuning rules

Use this order:

1. Leave `Kd = 0.0f` until the wheel is otherwise stable.
2. Tune `Kp` first.
3. Add a small `Ki` only if the wheel settles below target or drifts.
4. Keep `Ki` smaller than `Kp`; the loop is fast enough that a large integral term can overshoot or oscillate.
5. Keep `Kp` low enough that the wheel does not hunt around the target.

##### Good behavior

A tuned wheel should:

- reach the target speed promptly
- settle without large oscillation
- not overshoot so far that it swings around the target repeatedly
- respond similarly at 200, 400 and 600 permille duty

##### Signs the tune is too aggressive

- the wheel overshoots the target and then oscillates
- the encoder speed swings between fast and slow on the same command
- the robot starts to weave even at low speed
- one side is stable while the other continues hunting

##### Signs the tune is too weak

- the wheel never reaches the commanded speed
- it lags visibly every time at moderate or high duty
- the loop needs a long time to settle
- the chassis drifts to one side under straight-line commands

#### Update the firmware

Once you have a measured pair of values, put them in `firmware/airframe/src/config.h`:

```c
#define MOTOR_LEFT_PID_KP   0.55f
#define MOTOR_LEFT_PID_KI   0.08f
#define MOTOR_LEFT_PID_KD   0.00f
#define MOTOR_RIGHT_PID_KP  0.62f
#define MOTOR_RIGHT_PID_KI  0.10f
#define MOTOR_RIGHT_PID_KD  0.00f
```

The PI helper accepts separate values for the two wheels. These constants do
not affect the open-loop flight motor path.

#### PI activation procedure

Keep the PI helper isolated from the flight motor path until its repeated-step
host tests pass. Then validate it with the chassis raised before any floor run.

After updating the constants, rebuild and flash the firmware:

```sh
cmake -S firmware -B firmware/build
cmake --build firmware/build --target wanderer_airframe
```

With the chassis raised, run these checks:

1. Start with a low duty pulse, such as 100 or 200 permille.
2. Increase to 300, 400 and 600 permille.
3. Watch each wheel settle.
4. Compare the measured speed to the commanded target.
5. If one side lags, increase that side's `Kp` slightly; if it oscillates, decrease `Kp` or `Ki`.
6. Repeat until both wheel speeds settle without reversing or hunting.

#### Exact tuning workflow

Use this sequence when validating an enabled PI path:

1. Raise the rover and confirm both wheels are free.
2. Run `python tools/backdoor.py --calibrate` and resolve any sign or deadband issues.
3. Run `python tools/backdoor.py --calibrate-floor` and record the final `DEFAULT_TICKS_PER_METER`.
4. Run `python tools/backdoor.py --pid-tune --pid-wheel left ...` and save the CSV.
5. Run `python tools/backdoor.py --pid-tune --pid-wheel right ...` and save the CSV.
6. Enter the suggested gains into `firmware/airframe/src/config.h`.
7. Rebuild and flash the robot.
8. Run three or four duty steps on each wheel and check for overshoot or hunting.
9. Adjust `Kp` and `Ki` until both wheels settle cleanly.
10. Keep the final values as the calibration result for that rover.

What is a "duty step"? It is a commanded motor duty level. The value is not a speed. A duty step is simply a new command such as 100, 200, 300, 400 permille. In this system, 100 = 10% duty, 200 = 20% duty, and so on. The wheel then responds by spinning at some measured speed, and the encoder tells you what that actual speed was.

For example, a duty step from 100 to 200 permille means: command the wheel to 20% duty instead of 10% duty, wait for the encoder to react, then compare the measured speed to the commanded value. The exact command sequence is:

```sh
python tools/backdoor.py --pid-tune --pid-wheel left --pid-targets 100,200,300,400,500,600 --pid-pulse-ms 800 --pid-settle 0.6 --pid-out left_pid.csv
```

This is a 100, 200, 300, 400, 500, 600 permille sweep. Each value is one duty step. You do not need to invent a different sequence; the sweep helper already does the stepping for you.

If you want to do the validation by hand instead of using the helper, run 3 or 4 short pulses such as:

- 100 permille for one pulse
- 200 permille for one pulse
- 300 permille for one pulse
- 400 permille for one pulse

Keep each pulse short and compare the measured speed against the command. If the wheel shoots past the target and then swings back, that is overshoot. If the speed keeps bouncing above and below the target, that is hunting. If it rises too slowly and never reaches the target, that is under-response.

Why save the CSV? Because the CSV is the raw record of the bench test. It lets you compare command vs measured speed across several duty levels, and it shows whether the loop is behaving linearly or whether one wheel has a real offset. The CSV is not decorative; it is the evidence you use before accepting a tune. When you inspect the file, look for the pattern:

- command rises smoothly
- measured speed rises smoothly with it
- no repeated oscillation between high and low values
- left and right wheels have similar response shape

If the CSV shows this pattern, the response data is usable. If the curve is
jagged, repeat the sweep. `Kp` and `Ki` do not affect these raw-duty pulses.

Do not save CSV files and then ignore them. Save them once per wheel, compare the curves, and use the same file to decide whether a new coefficient set improved or worsened the response.

#### Keep the numbers honest

The default values in `config.h` are only seed values. The real values come from a bench sweep on the actual hardware. Do not keep a generic value if the measured wheel response says otherwise.

If a wheel is mechanically different, the tune should be different. The correct implementation is not one global PID for both wheels; it is one tuned loop per side.

### Testing without hardware

```sh
python tools/test_floor_sim.py
```

Walks the same prompts with a scripted operator against a simulated rover,
including the hand-turning step, and checks the arithmetic.

## 11. Still not calibrated

- **`DEFAULT_MAX_SPEED_MM_S`** (600) — needs a sustained run near full duty,
  which needs more floor than a USB cable reaches. Waits for the Pi5 to carry
  the tool, driven over ssh, untethered.
- **`TRACK_WIDTH_M`** (0.195f) — needs a measured *rotation* rather than a
  straight roll. Its own procedure, not yet automated.
