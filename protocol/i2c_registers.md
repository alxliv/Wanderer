# Wanderer — Tactical-host ↔ Pico I²C Register Map

**Boundary:** the tactical host — **Raspberry Pi Zero 2 W** (I²C **master**) ↔ Pico 2
(I²C **peripheral**). This is the single source of truth for the reflexive-tier interface,
and it is **board-agnostic** — the master could be any host.
The C header [`i2c_registers.h`](i2c_registers.h) mirrors this document exactly and is
included directly by the firmware; the host's Python side mirrors the same constants.

- **Protocol version:** 1
- **I²C address:** `0x42` (7-bit)
- **Bus speed:** 100 kHz default (safe), 400 kHz target once validated
- **Endianness:** **little-endian** for all multi-byte fields (matches RP2350 and the host)
- **Floats:** IEEE-754 `float32`, little-endian (native to both sides)

## Access model (register-pointer / memory model)

Standard I²C device convention:

- **Write:** master sends `[reg_addr][data0][data1]…`. The register pointer auto-increments
  with each data byte.
- **Read:** master sends `[reg_addr]`, issues a **repeated-START**, then reads `[data0][data1]…`;
  the pointer auto-increments. A read without first setting the pointer continues from the
  current pointer.

### Atomicity rules (important)
- **Telemetry reads are snapshotted.** When the master sets the register pointer to begin a
  read, the Pico latches a coherent copy of the telemetry block, so multi-byte values
  (e.g. 32-bit encoder counts) never tear mid-read. Read a related group in one transaction.
- **Commands apply on STOP.** Written command/control bytes take effect atomically when the
  write transaction completes (I²C STOP), so the left/right pair is applied together.

### Watchdog
- The watchdog timer is **refreshed on any write to the CONTROL/COMMAND region** (`0x10–0x1F`).
- If no refresh occurs within `WATCHDOG_TIMEOUT` (units of 10 ms), the Pico forces
  `CONTROL_MODE → IDLE`, clears `MOTOR_ENABLE`, stops the motors, and latches
  `STATUS.WATCHDOG_TRIPPED` plus `FAULT.WATCHDOG`.
- Recovery is explicit: the master must write safe commands, issue `CLEAR_FAULTS`, then
  re-assert `MOTOR_ENABLE` in a later command. Watchdog recovery is a latched safety stop,
  not an automatic pause/resume.
- A `WATCHDOG_TIMEOUT` of 0 disables the watchdog (not recommended).

---

## Register map

### INFO — read-only (`0x00–0x0F`)
| Addr | Name | Type | Notes |
|------|------|------|-------|
| 0x00 | `PROTOCOL_VERSION` | u8 | = 1 |
| 0x01 | `FW_VERSION_MAJOR` | u8 | firmware major |
| 0x02 | `FW_VERSION_MINOR` | u8 | firmware minor |
| 0x03 | `DEVICE_ID` | u8 | magic = `0x57` ('W'), sanity check |
| 0x04–0x0F | reserved | — | reads 0 |

### CONTROL / COMMAND — read/write (`0x10–0x1F`)
| Addr | Name | Type | Notes |
|------|------|------|-------|
| 0x10 | `CONTROL_MODE` | u8 | 0=IDLE/STOP, 1=VELOCITY (PID), 2=DIRECT_PWM (open-loop) |
| 0x11 | `CONTROL_FLAGS` | u8 | bitfield, see below |
| 0x12 | `CMD_LEFT`  | i16 | VELOCITY: mm/s · DIRECT_PWM: duty −1000…+1000 (sign=direction) |
| 0x14 | `CMD_RIGHT` | i16 | as above |
| 0x16 | `WATCHDOG_TIMEOUT` | u8 | ×10 ms (e.g. 50 = 500 ms); 0 = disabled |
| 0x17–0x1F | reserved | — | |

**`CONTROL_FLAGS` bits:**
| Bit | Name | Kind | Meaning |
|-----|------|------|---------|
| 0 | `MOTOR_ENABLE`  | level  | 1 = motor output allowed; 0 = stop (zero PWM) |
| 1 | `CLEAR_FAULTS`  | action | write 1 to clear FAULT bits; self-clearing |
| 2 | `RESET_ODOMETRY`| action | write 1 to zero encoder accumulators; self-clearing |
| 3–7 | reserved | — | |

### TELEMETRY — read-only (`0x20–0x3F`)
| Addr | Name | Type | Notes |
|------|------|------|-------|
| 0x20 | `STATUS` | u8 | bitfield, see below |
| 0x21 | `FAULT`  | u8 | bitfield, see below |
| 0x22 | `MEAS_LEFT`  | i16 | measured left wheel velocity, mm/s |
| 0x24 | `MEAS_RIGHT` | i16 | measured right wheel velocity, mm/s |
| 0x26 | `ENC_LEFT`  | i32 | cumulative left encoder ticks (signed) |
| 0x2A | `ENC_RIGHT` | i32 | cumulative right encoder ticks (signed) |
| 0x2E | `TOF_FRONT_MM` | u16 | front distance, mm (`0xFFFF` = invalid/out of range) |
| 0x30 | `TOF_STATUS` | u8 | 0 = valid; nonzero = sensor status/error code |
| 0x32 | `LOOP_HZ` | u16 | measured control-loop frequency (health) |
| 0x34 | `VBAT_MOTOR_MV` | u16 | motor-pack voltage, mV — **reserved/future** (needs sensing) |
| 0x36 | `VBAT_LOGIC_MV` | u16 | logic-pack voltage, mV — **reserved/future** |
| 0x38–0x3F | reserved | — | |

**`STATUS` bits:**
| Bit | Name | Meaning |
|-----|------|---------|
| 0 | `MOTORS_ENABLED` | mirror of effective enable state |
| 1 | `WATCHDOG_TRIPPED` | watchdog fired; motors forced to IDLE |
| 2 | `OBSTACLE_STOP` | obstacle reflex active (ToF < threshold) |
| 3 | `LEFT_AT_TARGET` | left wheel within velocity tolerance |
| 4 | `RIGHT_AT_TARGET` | right wheel within velocity tolerance |
| 5 | `ANY_FAULT` | mirror: `FAULT != 0` |
| 6–7 | `MODE` | current mode (0=idle,1=velocity,2=pwm) |

**`FAULT` bits:**
| Bit | Name | Meaning |
|-----|------|---------|
| 0 | `STALL_LEFT` | left wheel commanded but not moving |
| 1 | `STALL_RIGHT` | right wheel commanded but not moving |
| 2 | `TOF_ERROR` | ToF sensor read/init error |
| 3 | `ENC_LEFT_ERROR` | left encoder fault |
| 4 | `ENC_RIGHT_ERROR` | right encoder fault |
| 5 | `OVERCURRENT` | reserved/future |
| 6 | `LOW_VOLTAGE` | reserved/future |
| 7 | `WATCHDOG` | command watchdog timed out; latched until `CLEAR_FAULTS` |

### CONFIG — read/write (`0x40–0x5F`)
Online-tunable for R&D. Applied on write-transaction STOP. Defaults baked into firmware.
| Addr | Name | Type | Notes |
|------|------|------|-------|
| 0x40 | `TICKS_PER_METER` | f32 | x4 encoder-edge calibration (all valid A/B transitions per meter) |
| 0x44 | `PID_KP` | f32 | velocity PID, applied to both wheels |
| 0x48 | `PID_KI` | f32 | |
| 0x4C | `PID_KD` | f32 | |
| 0x50 | `MAX_PWM` | u16 | output clamp, 0…1000 (per-mille) |
| 0x52 | reserved | — | |
| 0x54 | `OBSTACLE_STOP_MM` | u16 | ToF reflex threshold, mm; 0 = disabled |
| 0x56–0x5F | reserved | — | |

---

## Typical interaction sequences

**Startup handshake (master):**
1. Read `PROTOCOL_VERSION`, `DEVICE_ID` → verify `1` and `0x57`.
2. (Optional) Write CONFIG: `TICKS_PER_METER`, PID gains, `MAX_PWM`, `OBSTACLE_STOP_MM`.
3. Write `WATCHDOG_TIMEOUT` (e.g. 50 = 500 ms).
4. Write `CONTROL_FLAGS = MOTOR_ENABLE`, set `CONTROL_MODE = VELOCITY`.

**Control loop (master, e.g. 20–50 Hz):**
1. Write `CMD_LEFT`, `CMD_RIGHT` (mm/s) — this also refreshes the watchdog.
2. Read telemetry block `0x20…0x33` in one transaction (status, velocities, encoders, ToF).

**Watchdog recovery after `FAULT.WATCHDOG`:**
1. Write `CMD_LEFT = 0`, `CMD_RIGHT = 0`, and `CONTROL_FLAGS` with `MOTOR_ENABLE = 0`.
2. Write `CONTROL_FLAGS = CLEAR_FAULTS`.
3. After telemetry shows `FAULT.WATCHDOG = 0`, write the desired `CONTROL_MODE` and
   re-assert `MOTOR_ENABLE`.

**Bring-up / open-loop test:**
- Set `CONTROL_MODE = DIRECT_PWM`, write `CMD_LEFT/RIGHT` as ±duty to validate wiring
  and direction before tuning PID.

## Versioning
Bump `PROTOCOL_VERSION` on any breaking layout change. The master checks it during the
startup handshake and refuses to drive on mismatch. Reserved regions allow additive,
backward-compatible growth without a version bump.
