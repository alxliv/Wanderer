# Wanderer — IMU Integration and Closed-Loop Heading

**Status:** Design spec. Not yet implemented.
**Scope:** Add a Pololu MinIMU-9 v6 to the airframe, own a heading estimate on
the Pico2, and close the loop on `proc turn` and on straight advance — replacing
the open-loop wheel-gain matching that currently makes a "forward" command veer.

Companions: [Wanderer_Command_Architecture.md](Wanderer_Command_Architecture.md)
(§2a body frame, §5 Tier 2), [motor_calibration.md](motor_calibration.md)
(bench procedure this extends), [../protocol/cockpit_protocol.md](../protocol/cockpit_protocol.md)
(normative wire).

---

## 0. What this fixes, and what it does not

Wanderer veers because two independent errors add up, and neither is currently
observed:

| Error | Present magnitude | Fixed by |
|---|---|---|
| Open-loop wheel gain mismatch | `MOTOR_RIGHT_GAIN_PERMILLE 841` — a *single* number fitted at one duty, on one floor, unloaded | Heading loop (integral term absorbs it continuously) |
| Deadband asymmetry (55‰ left vs 35‰ right) | Below ~65‰ one wheel spins and one does not | Deadband feed-forward (§9) — **not** the IMU |
| Wheel slip | Unbounded, surface-dependent | Heading loop (gyro does not care about slip) |
| Encoder-derived heading being wrong | Same root cause as slip | Gyro replaces it inside `proc turn` |

The IMU fixes three of those four. **The deadband asymmetry is not an IMU
problem** and a heading loop layered on top of it will fight a nonlinearity
instead of a disturbance — the integrator will wind up while a wheel sits
still, then lurch when it breaks away. §9 treats it as a prerequisite, not an
afterthought.

Equally, be clear about what the gyro does *not* give you: **position**.
Distance still comes from ticks. A gyro-corrected straight line is still dead
reckoning, still a *seed* for the Pilot to fuse with a map
(architecture §8), just a much better-conditioned one.

---

## 1. Part choice and why the magnetometer is not in the plan

MinIMU-9 v6 = **LSM6DSO** (3-axis gyro + accel) + **LIS3MDL** (magnetometer),
I²C, on-board regulator and level shifter, 2.5–5.5 V.

Only the LSM6DSO gyro's **Z axis** is load-bearing in this design.

- **Gyro noise:** 3.8 mdps/√Hz rate noise density ≈ 0.23 °/√hr angle random
  walk. At `TURN_RATE_RAD_S` = 50 °/s over a 2 s turn, random-walk error is
  well under 0.05° — irrelevant next to every other error in the vehicle.
  (**mdps** = millidegrees per second, the datasheet's unit throughout;
  3.8 mdps/√Hz = 0.0038 °/s per √Hz of bandwidth. Multiply by 60 to get
  angle random walk in °/√hr.)
- **Gyro bias** is the entire game. Raw zero-rate offset is ~±1 °/s, which
  integrates to **60°/min** if ignored. §5 is how that number gets to ~0.05 °/s
  (≈3 °/min), at which a 2 s turn accrues ~0.1°.
- **Accelerometer:** used only for the stillness detector (§5.3) and, later,
  a tilt reflex. Not used for heading.
- **Magnetometer: deliberately unused.** Two DC motors drawing amps within
  ~15 cm, plus indoor rebar and appliances, make LIS3MDL heading unreliable in
  exactly the situations where it would matter. Absolute heading is the Pilot's
  problem (map / scan matching, architecture §3), not the airframe's. Wire it,
  leave it powered, read nothing from it. Revisit only if a hard-iron
  calibration on the actual chassis shows a stable field.

**Which measuring range to use.** The gyro's range is selectable, and the
choice is a straight trade: a wider range reports coarser steps, a narrower
one risks running off the end of the scale. **Use ±500 °/s**, for three
reasons.

*Headroom.* The fastest turn the firmware ever commands is 50 °/s
(`TURN_RATE_RAD_S`), so ±500 leaves 10× margin. That margin matters because
going off-scale is unrecoverable: the reading pins at the maximum, the extra
rotation is never reported, and the heading stays wrong by however much was
missed. A bump, a shove, or dropping off a threshold all produce brief
rotation rates far above anything you deliberately command.

*The coarser steps cost nothing here.* At ±500 °/s the smallest change the
chip can report is 0.0175 °/s. The sensor's own electrical noise is around
0.02 °/s — about the same size as that step. So the steps are already as fine
as the noise they sit in, and a finer scale would only be measuring the noise
more precisely.

*Why not ±250 °/s.* It would halve the step to about 0.009 °/s, which by the
point above buys nothing real, while dropping the pinning threshold to
250 °/s — low enough that a knock could reach it.

---

## 2. Turn direction

Z points down and the frame is right-handed, in every layer — architecture
§2a. That makes **a positive turn a turn to the right**, and heading increases
as the robot turns right.

The IMU driver converts the chip's output to match:

```c
/* ---- IMU turn direction ----
 * Turning RIGHT must make this number positive. Z is down and the frame is
 * right-handed, system-wide -- see docs/Wanderer_Command_Architecture.md
 * section 2a. Applied in imu_sample(), the last thing before the rate leaves
 * the driver, so every reading above this line is already corrected.
 *
 * The LSM6DSO's own Z axis points UP when the board is mounted
 * components-up, so the expected value in that orientation is -1. Do not
 * trust that: turn the robot to the RIGHT by hand and confirm the number goes
 * positive. `python tools/imu_cal.py --check-sign` does exactly that.
 */
#define IMU_YAW_SIGN   (-1)
```

Do not work the sign out from the axis drawing in the datasheet. Turn the
robot to the right by hand and watch the number.

---

## 3. Wiring

> **The IMU goes on the airframe.** The heading loop is a Tier 2 firmware
> procedure (architecture §3), so the gyro sits inside a hard-real-time loop.
> Reading it across a Linux I²C bus would reintroduce the scheduling jitter the
> tactical layer exists to prevent — the same argument that puts the
> wheel-velocity loop on the Pico2. A pilot-side IMU can serve camera and
> Pan-Tilt attitude, but it cannot close `proc turn`. If both are wanted, that
> is an argument for a **second** IMU on the pilot bus.

### 3.1 Bus assignment

`I2C0` (GP4/GP5) is reserved for the VL53L0X ToF. **Put the IMU on `I2C1`,
not I2C0.** Reasons, in order of weight:

- The gyro is read every control tick and is *in the control loop*. A ToF
  ranging transaction or a stretched clock from a misbehaving INA226 must
  never be able to delay it.
- The planned INA226 pair also wants a bus; grouping the two slow,
  non-real-time sensors on I2C0 and the one real-time sensor alone on I2C1 is
  the partition that survives contact with a fault.

### 3.2 Pins

| Signal | Pin | Note |
|---|---|---|
| `IMU_SDA_PIN` | **GP6** | I2C1 SDA |
| `IMU_SCL_PIN` | **GP7** | I2C1 SCL |
| VDD | 3V3(OUT) | board regulator + level shifter accept it directly |
| GND | GND | |

Four wires. The carrier breaks out power and I²C only — no data-ready line
(§4.2).

**The radio's pins are reserved.** `firmware/rflink/main.cpp` holds GP2, 9, 14,
15, 21, 22 and 28 — `spi1` plus CE, CSN and `PIN_ROLE`. `wanderer_rflink` and
`wanderer_airframe` are never flashed together today, but architecture §3a puts
an RF backdoor *inside* the flight firmware, at which point those pins are
claimed for real. Treat them as taken.

The airframe holds GP0/1, 4/5, 8, 10–13, 16/17, 19/20. That leaves **GP3, GP6,
GP7, GP18, GP26, GP27** on the header. GP6/GP7 are the only free *adjacent*
I2C1 SDA/SCL pair. GP26/GP27 are the fallback I2C1 pair.

Bus at **400 kHz**. The MinIMU-9 v6 carries its own pull-ups (~10 kΩ) on the
level-shifted side, so add none. Keep the run under ~15 cm; a 100 nF ceramic
at the board's VDD pin.

I²C addresses: **LSM6DSO `0x6B`** (SA0 pulled high on this carrier),
**LIS3MDL `0x1E`**. No collision with the VL53L0X (`0x29`) or the planned
INA226s (`0x40`–`0x4F`) even if the buses are ever merged.

### 3.3 Mounting — this matters more than the wiring

- **Rigid.** Screwed or standoff-mounted to the chassis. Not foam tape:
  vibration aliases into the gyro output as an apparent bias, and adhesive
  creeps, which quietly rotates your calibration over weeks.
- **Z axis vertical**, within a couple of degrees. X/Y alignment to bearing 0
  barely matters for yaw (it is a rotation about Z regardless) but costs
  nothing to get right and matters for a future tilt reflex.
- **Position is irrelevant for yaw** — angular velocity of a rigid body is the
  same everywhere on it. Mount it wherever it is most rigid and least shaken.
  Do *not* place it on a cantilevered bracket just to centre it.
- **Away from the motors** — ≥10 cm if the chassis allows. Chiefly for the
  magnetometer you are not using yet, but also keeps the I²C run out of the
  high-current return path.

Keep `hardware/wiring.md` in sync with `config.h`, per the existing rule.

---

## 4. The `imu` module — driver

New files: `firmware/airframe/src/imu.h`, `imu.c`. C module with file-static
state, matching `encoders.h`'s shape.

```c
#ifndef WANDERER_IMU_H
#define WANDERER_IMU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    yaw_rate;   /* rad/s, right-turn-positive, bias-corrected */
    float    az;         /* m/s^2, for the stillness detector only */
    uint64_t t_us;       /* RP2350 timestamp of this sample */
    bool     fresh;      /* false if no new sample was available */
} imu_sample_t;

/*
 * Bring up I2C1 and the LSM6DSO. Returns 0 on success, negative if WHO_AM_I
 * does not read 0x6C or the bus does not respond. A negative return leaves
 * imu_healthy() false; the caller decides whether that is fatal.
 */
int  imu_init(void);

/*
 * Read gyro and accel over I2C. Applies IMU_YAW_SIGN and subtracts the current
 * bias estimate. fresh=false if the read failed. Call once per control tick.
 */
imu_sample_t imu_sample(void);

/* False once no fresh sample has arrived for IMU_STALE_MS. */
bool imu_healthy(void);

/* The raw count from the chip, uncorrected -- diagnostics and calibration. */
int16_t imu_raw_gyro_z(void);

#ifdef __cplusplus
}
#endif
#endif /* WANDERER_IMU_H */
```

### 4.1 Register configuration

WHO_AM_I (`0x0F`) must read **`0x6C`**. Anything else and you are talking to
the wrong part — stop.

| Register | Value | Meaning |
|---|---|---|
| `CTRL3_C` (0x12) | `0x01` then wait, then `0x44` | software reset; then BDU=1, IF_INC=1 |
| `CTRL2_G` (0x11) | `0x44` | gyro ODR 104 Hz, FS ±500 °/s |
| `CTRL1_XL` (0x10) | `0x40` | accel ODR 104 Hz, FS ±2 g |
| `CTRL4_C` (0x13) | `0x02` | enable the gyro's low-pass filter |
| `CTRL6_C` (0x15) | filter bandwidth | confirm the `FTYPE` bits against the datasheet at 104 Hz |

Gyro Z is `OUTZ_L_G`/`OUTZ_H_G` (`0x26`/`0x27`). **BDU=1 is not optional** —
without it a byte pair can straddle an update and produce a plausible,
catastrophic wrong value. At ±500 °/s each count is worth **0.0175 °/s**.

### 4.2 Timing — polled, not interrupt-driven

**The MinIMU-9 v6 breaks out power and I²C only.** The LSM6DSO die has INT1 and
INT2 pads, but this carrier does not route them to the header, so there is no
data-ready line to wait on and no GPIO to spend. The gyro is read by polling.

- **Gyro ODR 104 Hz, read once per 100 Hz control tick.** Matching the two
  rates means one 6-byte I²C read per tick — about 250 µs at 400 kHz, 2.5% of
  the loop.
- **Integrate against the RP2350 timer, not a nominal period.**
  `dpsi = yaw_rate * (t_us - t_prev_us) * 1e-6f`. This is what makes polling
  safe: the sensor's clock and the Pico's are independent, so occasionally the
  same sample is read twice or one is skipped. Using the measured interval
  turns that into a zero-order-hold error on a slowly-varying signal — bounded,
  and it averages out across a maneuver. Assuming a fixed 1/104 s interval
  instead would fold the LSM6DSO's oscillator tolerance straight into scale
  factor, and that error *does* accumulate.
- **Match the filter to the sample rate.** With ODR at 104 Hz the gyro's
  low-pass filter should be set so its bandwidth is under ~50 Hz, otherwise
  noise above half the sample rate folds back into the reading.

### 4.3 Health

`IMU_STALE_MS` = 50 ms — five missed reads. Losing the gyro mid-turn is a real
failure mode; a loose Dupont wire is all it takes. It must abort the procedure
loudly rather than integrate zero and stop wherever.

Without a data-ready line, "stale" means the I²C read itself failed or the
sample never changes. Check both: a bus error, and `STATUS_REG` (`0x1E`) bit
`GDA` never asserting.

---

## 5. Calibration

Three separate numbers. Do not conflate them.

### 5.1 Bias — the one that matters

Zero-rate offset, ~±1 °/s raw, drifts with temperature and time. Measured at
rest, subtracted from every sample.

**Boot calibration.** On `imu_init()`, if the vehicle is stationary, average
**512 samples (~5 s)** of raw gyro Z. Report both mean and standard
deviation — the σ is your noise floor and the number that tells you whether
the mount is rigid. Expect σ ≈ 0.02 °/s on a still bench; σ above ~0.2 °/s
means vibration or a bad mount, and you should fix that before proceeding.

If the vehicle is *not* still at boot, skip it and rely on §5.3. Never
calibrate bias on a moving robot; a wrong bias is worse than a stale one
because it looks fine.

### 5.2 Bias is not a constant

Motors heat the chassis. A bias calibrated cold is wrong by the time you have
been driving for ten minutes. This is why §5.3 exists and why "calibrate once
at the factory" does not work for this part.

### 5.3 Runtime re-zero — the trick that makes this part good enough

Continuously re-estimate bias whenever the robot is provably still:

```
still  :=  left_delta == 0  AND  right_delta == 0
           AND |yaw_rate_raw - bias| < STILL_GYRO_THRESH   (0.5 deg/s)
           AND |az - 9.81|            < STILL_ACCEL_THRESH (0.3 m/s^2)
           held for STILL_TICKS       (50 ticks = 0.5 s at CONTROL_HZ)

while still:  bias += IMU_BIAS_ALPHA * (yaw_rate_raw - bias)   /* alpha = 0.004 */
```

Notes that are easy to get wrong:

- **Encoder stillness alone is not enough.** A robot pushed across a slick
  floor with wheels locked reads zero ticks while rotating. The gyro and
  accelerometer gates close that hole cheaply.
- **Never update bias while a Tier 2 procedure is active**, even if the
  stillness test passes momentarily at the start of a turn. Gate on
  procedure-idle explicitly.
- α = 0.004 per sample at 104 Hz gives a ~2.4 s time constant — fast enough to
  track thermal drift across a stop, slow enough that half a second of
  spurious stillness cannot corrupt it.
- The residual after this lands around **0.05 °/s**, i.e. ~3 °/min of free
  drift. Over a 2 s turn that is 0.1°; over a 30 s straight run, 1.5°. Both
  are far below the errors you have today.

### 5.4 Scale factor — and the `TRACK_WIDTH_M` payoff

Sensitivity error is ~1–3%. Calibrate by rotating the robot through a large,
exactly known angle and comparing.

`tools/imu_cal.py --scale`, over the backdoor:

1. Mark the floor. Rotate the chassis **by hand** through **10 full turns**
   (3600°), slowly and continuously, returning to the mark.
2. Read integrated yaw. `IMU_SCALE = 3600.0 / measured_degrees`.
3. Ten turns rather than one for the same reason `--calibrate-floor` uses five
   hand-turns: your error in judging "back to the mark" divides by the count.

**Then get `TRACK_WIDTH_M` for free.** It is listed in
motor_calibration.md §10 as still uncalibrated and needing "its own procedure,
not yet automated" — the gyro *is* that procedure. Drive an in-place rotation,
integrate yaw, and read the encoders:

```
track = (d_left - d_right) / dpsi          where d = ticks / DEFAULT_TICKS_PER_METER
```

(Left minus right, because a positive `dpsi` is a turn to the right, which
drives the left wheel forward and the right wheel back — §2.)

Run it over several revolutions and fit the slope rather than using one
sample. This is the single best argument for doing the IMU work now: it closes
out a calibration constant that has no other automated path.

Caveat: this measures *effective* track under rotation slip, which is what the
kinematics actually want — it will legitimately differ from a tape measure
between the wheels. That is a feature. If it differs by more than ~15%,
suspect a sign error rather than physics.

### 5.5 Where the numbers live

`IMU_YAW_SIGN` and `IMU_SCALE` are compiled-in `config.h` constants, reported
by the backdoor's `cfg` verb alongside `ENC_*_SIGN`/`MOTOR_*_SIGN`. Bias is
**runtime state, not a config constant** — it is re-estimated every session and
must never be pasted into a header. The firmware owns all of them; the Pilot
asks and never carries a copy.

---

## 6. The heading estimator

New: `firmware/airframe/src/heading.h`, `heading.c` — a C module with
file-static state, in the `tactical.c` idiom.

```c
void  heading_init(void);
void  heading_update(const imu_sample_t *s, const encoder_sample_t *e,
                     bool procedure_active);
float heading_get(void);        /* rad, right-turn-positive, wrapped (-pi,pi] */
float heading_rate(void);       /* rad/s, bias-corrected */
float heading_bias(void);       /* rad/s, current estimate -- diagnostics */
void  heading_zero(void);       /* make the current heading the new zero */
bool  heading_valid(void);      /* false until first bias estimate lands */
```

The whole estimator is:

```c
if (s->fresh) {
    float dt = (s->t_us - prev_t_us) * 1e-6f;
    if (dt > 0.0f && dt < 0.1f)          /* reject the first sample and any gap */
        psi = wrap_pi(psi + s->yaw_rate * dt);
    prev_t_us = s->t_us;
}
update_bias_if_still(s, e, procedure_active);
```

That is the entire design and it should stay that way. Resist adding an EKF.
There is one state, one sensor, and one error term, and the error term is
handled by an observability condition (stillness) rather than by a covariance.
A filter here would add tuning knobs without adding information.

`wrap_pi` should live once, in a shared header, and get a host unit test —
angle wrapping is where this class of code actually breaks.

---

## 7. Control laws

### 7.1 Straight advance — `proc move`

Architecture §5 already lists "advance a relative distance and stop" and "hold
a relative heading while moving" as Tier 2. This is both, in one procedure.

On entry: latch `psi_ref = heading_get()`, record start ticks. Each control
tick at `CONTROL_HZ`:

```
e     = wrap_pi(psi_ref - heading_get());
i    += e * dt;                                    /* clamp: |i| <= I_MAX */
omega = KP_HEADING * e + KI_HEADING * i - KD_HEADING * heading_rate();
omega = clamp(omega, -OMEGA_MAX, OMEGA_MAX);
```

then straight into the differential mix — `v_left = v + omega*track/2`,
`v_right = v - omega*track/2` (right-turn-positive, §2) — so this rides on top
of the control surface already there rather than reaching for the motors.

Starting gains, to be tuned on the floor:

| Constant | Start | Reasoning |
|---|---|---|
| `KP_HEADING` | 2.0 s⁻¹ | 5° error → 0.17 rad/s correction; visible but not twitchy |
| `KI_HEADING` | 0.5 s⁻² | see below — this term is the point |
| `KD_HEADING` | 0.1 s | gyro rate is a clean derivative; damps without noise amplification |
| `I_MAX` | 0.3 rad/s equivalent | anti-windup, sized to the largest gain mismatch you expect |
| `OMEGA_MAX` | 0.5 rad/s | keeps correction authority well inside the wheel-speed limit |

**The integral term is not optional here, and it is the interesting one.**
The veer is a *persistent* disturbance — a wheel gain mismatch produces a
constant yaw rate — and proportional heading feedback against a constant rate
disturbance settles at a constant heading *offset*, not zero. The integrator
is what drives it to zero.

Better: **the integrator's steady-state value is the online, loaded,
per-surface replacement for `MOTOR_RIGHT_GAIN_PERMILLE`.** Log it. If 841 is a
good number, it should settle near zero. If it settles consistently far from
zero, 841 is wrong for the loaded case and you have just measured the
correction — for free, every run, on whatever floor you are on. That is a much
better instrument than the interpolation that produced 841.

### 7.2 `proc turn` — swap the heading source

The procedure structure already exists and does not change. The one edit is
that it stops deriving heading from `(d_right - d_left)/track` and instead
reads `heading_get()`. Everything else — `TURN_RATE_RAD_S`,
`TURN_OVERSHOOT_RAD`, `TURN_LINEAR_LIMIT_M_S`, the timeout, the
`!proc name=turn outcome=...` event — carries over unchanged.

Two additions:

- **Abort on stale IMU.** If `imu_healthy()` goes false mid-turn, abort with
  `!proc name=turn outcome=ABORTED reason=imu_stale`. Do not silently fall
  back to encoder heading: the whole point is that encoder heading is the
  thing you do not trust, and a silent downgrade means the failure shows up as
  a mysteriously bad turn weeks later.
- **Refuse to start** if `heading_valid()` is false, with a new
  `=err proc imu_not_ready`.

`TURN_OVERSHOOT_RAD` will need re-tuning after this change — it currently
compensates for encoder-heading error as well as for stopping latency, and
only the latter remains. Expect the right value to get smaller.

### 7.3 What `drive` does — nothing

Tier 1 `drive` stays open-loop in yaw. It is the Pilot's continuous hands-on
surface; a heading hold underneath it would be the airframe second-guessing a
commander that is actively steering. Heading hold belongs to Tier 2
procedures, where the Pilot has explicitly engaged and forgotten.

---

## 8. Protocol additions

All additive, all in the existing grammar; §2 parser robustness means older
clients skip them cleanly.

**New cockpit verbs**

| Verb | Args | `=ok` fields | `=err` reasons |
|---|---|---|---|
| `get_heading` | — | `psi=<rad> rate=<rad/s> bias=<rad/s> valid=<0\|1>` | — |
| `zero_heading` | — | — | `imu_not_ready` |
| `proc move` | `<distance_m> <linear_m_s>` | `name=move lin=<m/s> timeout=<s>` | `not_armed`, `bad_args`, `imu_not_ready`, `busy` |

`proc move` slots into the reserved `proc <name> [arg ...]` extension point of
§10 — no protocol revision needed.

**New error reason:** `imu_not_ready` — heading estimator has no valid bias yet,
or the IMU failed to initialize.

**New `!proc` reason:** `imu_stale`.

**New backdoor verbs** (category: *dev diagnostic* — the closed-membership rule
of architecture §3a admits "read a register", and neither verb commands motion):

| Verb | Effect |
|---|---|
| `imu` | `=ok imu raw=<count> rate=<deg/s> bias=<deg/s> psi=<deg> ok=<0\|1>` |
| `imu cal` | Re-run the §5.1 static bias average; report mean and σ |

`cfg` grows `imu_sign=` and `imu_scale=` alongside the existing constants, so
`tools/imu_cal.py` reads them from the board rather than keeping a copy —
same discipline as `tools/backdoor.py` reading `max_duty` from `ver`.

---

## 9. Prerequisite: deadband feed-forward

motor_calibration.md §6 already names this as "the usual cause of *it veers at
slow speed*", and the measured numbers make it concrete: **left breaks away at
55‰, right at 35‰, and both only turn reliably at 65‰.**

Now price the turn procedure against that. `TURN_RATE_RAD_S` = 50 °/s =
0.873 rad/s, so each wheel runs at `omega * track/2` = 0.873 × 0.0975 ≈
**0.085 m/s**. Against `DEFAULT_MAX_SPEED_MM_S` = 600, that is ≈ **142‰** —
and the 55/35 figures are *unloaded*; loaded breakaway is materially higher.
Turns are therefore running close to the floor of what the drivetrain can
actually deliver, which is very likely part of why they are inconsistent
today.

Fix it in `motors_set()`, before `MOTOR_*_SIGN`:

```
duty = (u == 0) ? 0
                : sign(u) * (DEADBAND + (1000 - DEADBAND) * |u| / 1000)
```

so a small commanded value produces a small *motion* rather than nothing. This
is worth landing **before** the heading loop, not after: an integrator working
against a dead zone winds up while the wheel sits still and then lurches. Do
it first and the loop tunes easily; do it after and you will spend an evening
blaming the gains.

While you are there, consider raising `TURN_RATE_RAD_S` — with heading closed
on the gyro there is much less reason to creep.

---

## 10. Milestones

Each ends in a working, committed, tested state, in the project's usual
design → implement → test → commit order.

**M0 — Wire and bring up.** Mount, wire I2C1, add `IMU_*` pins to `config.h`,
`imu_init()` with a WHO_AM_I check, `imu` backdoor verb printing the raw count.
*Done when:* `=ok imu raw=…` responds and the raw number swings the expected
way when you spin the robot by hand.

**M1 — Driver.** Polled reads on the control tick, timestamped from the RP2350
timer, `IMU_YAW_SIGN` verified by hand-spin, `imu_healthy()`. *Done when:* rate
in °/s is right within eyeball accuracy and holding the robot still reads near
zero.

**M2 — Estimator.** `heading.c`, boot bias cal, runtime re-zero, `get_heading`
and `zero_heading` on the cockpit, plus the matching model in
`pilot/cockpit/sim.py` and new golden vectors. *Done when:* the drift test in
§11 passes and the sim and firmware agree on the vectors.

**M3 — Calibration tooling.** `tools/imu_cal.py` with `--check-sign`,
`--scale`, `--track`. Land `IMU_SCALE` and a real `TRACK_WIDTH_M`.
*Done when:* `TRACK_WIDTH_M` stops being a guess.

**M4 — Deadband feed-forward** (§9). Re-run `tools/backdoor.py --calibrate` to
confirm the effective deadband is gone. *Done when:* a 100‰ command moves both
wheels.

**M5 — `proc turn` on the gyro.** Swap the heading source, add `imu_stale`
abort, re-tune `TURN_OVERSHOOT_RAD`. *Done when:* the repeatability test in §11
passes. **This retires the Linux-side odometry-polling placeholder in
`pilot/helm/` — delete it in the same commit**, so the bench tool cannot
outlive its replacement.

**M6 — `proc move` with heading hold.** The PID, anti-windup, integrator
logging. *Done when:* the straight-line test in §11 passes.

M4 is ordered before M5/M6 deliberately, per §9.

---

## 11. Test plan

**Host tests** (`firmware/common/tests`, ctest — remember `-C Debug` on
Windows). No hardware:

- `wrap_pi` across ±3π and at exactly ±π.
- Estimator fed a synthetic constant-rate stream integrates to the analytic
  angle within 0.1%.
- Bias convergence: inject a 1 °/s bias, hold the stillness condition, assert
  convergence to within 0.05 °/s and the expected time constant.
- Bias is **not** updated while `procedure_active` is true, nor while encoder
  deltas are nonzero.
- Stale detection fires at `IMU_STALE_MS` and aborts an active turn.
- Golden vectors in `protocol/cockpit_vectors.txt` for `get_heading`,
  `zero_heading`, `proc move`, `=err … imu_not_ready` — consumed by both the
  C++ and Python suites, as ever.

**Simulator** (`pilot/cockpit/sim.py`): model the gyro *with slip* — that is
the entire point. Give simulated wheels a slip factor so encoder-derived
heading and gyro heading diverge, and assert `proc turn` now tracks the gyro.
A slip-free sim would pass with the bug still in.

**Bench** (chassis raised, per motor_calibration.md §5):

| Test | Pass criterion |
|---|---|
| Bias noise | σ ≤ 0.05 °/s over 5 s still. Above 0.2 °/s → fix the mount |
| Static drift | Heading drifts < 0.5° over 60 s stationary, motors **powered and idling** — the idle case is where EMI shows up |
| Sign | Rotate the robot to the **right** by hand → `psi` increases (§2) |
| Scale | 10 hand turns integrate to 3600° ± 1% after `IMU_SCALE` |

**Floor:**

| Test | Pass criterion |
|---|---|
| Turn repeatability | `proc turn` 90°, ×10, alternating direction: final heading within **±2°**, no cumulative bias across the ten |
| Turn accuracy | Marked floor angle vs. commanded, ±3° |
| **Straight line** | `proc move 3.0 0.25`: lateral deviation < **5 cm over 3 m** (≈1°). Compare against the same run with the loop disabled — record both; the delta is the deliverable |
| Slip rejection | Straight run with one wheel briefly lifted or a hand-brake drag. Heading recovers; encoder-only would not |
| Thermal | 10 minutes of driving, then a static drift retest. Confirms §5.3 is doing its job |

The straight-line before/after pair is the number worth putting in the commit
message. It is the one that answers "did this work".

---

## 12. Open questions

1. **Does `proc move` replace helm's `forward`?** Helm's straight moves should
   probably route to `proc move` once M6 lands, the same way `turn` routes to
   `proc turn`. Worth settling when M6 is designed rather than discovered.
2. **Should the integrator's settled value be exposed as telemetry?** It is a
   live estimate of drivetrain asymmetry and would make a good transponder
   field — and a good early warning for a failing gearbox. Cheap to add, no
   control path consumes it, so it stays inside the telemetry ground rules.
3. **LIS3MDL** — wired, powered, unread. Revisit only with data from a
   hard-iron calibration on the actual chassis, not on principle.
