# Wanderer — Wiring & Pinouts

> Pico GPIO numbers are assigned and reflected here. Keep this file aligned
> with the firmware; `firmware/airframe/src/config.h` wins any disagreement.

## Pico 2 (RP2350) GPIO summary

The `wanderer_airframe` flight firmware uses **16 header GPIOs**, all assigned
in [`firmware/airframe/src/config.h`](../firmware/airframe/src/config.h) — the
source of truth for application pin assignments. Keep this table and the
detailed connection tables below synchronized with it.

| GPIO | Physical pin | Direction / function | Connected device and use | Status / source |
|------|--------------|----------------------|--------------------------|-----------------|
| GP0 | 1 | UART0 TX, output | **Cockpit** protocol lines to pilot SBC RX (header pin 10), 115200 8N1 | Active; `PICO2_UART_TX_PIN` |
| GP1 | 2 | UART0 RX, input | **Cockpit** commands from pilot SBC TX (header pin 8) | Active; `PICO2_UART_RX_PIN` |
| GP4 | 6 | I²C0 SDA, bidirectional | VL53L0X front ToF data; Pico is I²C master | Reserved; `TOF_SDA_PIN` |
| GP5 | 7 | I²C0 SCL, output | VL53L0X front ToF clock | Reserved; `TOF_SCL_PIN` |
| GP6 | 9 | I²C1 SDA, bidirectional | MinIMU-9 v6 data; Pico is I²C master | Reserved; `IMU_SDA_PIN` |
| GP7 | 10 | I²C1 SCL, output | MinIMU-9 v6 clock | Reserved; `IMU_SCL_PIN` |
| GP8 | 11 | Digital output | VL53L0X `XSHUT` reset/enable | Reserved/optional; `TOF_XSHUT_PIN` |
| GP10 | 14 | PIO digital input | Left MD520 encoder C1 / channel A | Active; `ENC_LEFT_PIN_BASE` |
| GP11 | 15 | PIO digital input | Left MD520 encoder C2 / channel B | Active; `ENC_LEFT_PIN_BASE + 1` |
| GP12 | 16 | PIO digital input | Right MD520 encoder C1 / channel A | Active; `ENC_RIGHT_PIN_BASE` |
| GP13 | 17 | PIO digital input | Right MD520 encoder C2 / channel B | Active; `ENC_RIGHT_PIN_BASE + 1` |
| GP16 | 21 | PWM output | MDD10A `PWM1`, left motor speed at 20 kHz | Active; `M1_PWM_PIN` |
| GP17 | 22 | Digital output | MDD10A `DIR1`, left motor direction | Active; `M1_DIR_PIN` |
| GP18 | 24 | Digital input, IRQ | MinIMU-9 v6 LSM6DSO `INT1`, gyro data-ready | Reserved; `IMU_INT1_PIN` |
| GP19 | 25 | PWM output | MDD10A `PWM2`, right motor speed at 20 kHz | Active; `M2_PWM_PIN` |
| GP20 | 26 | Digital output | MDD10A `DIR2`, right motor direction | Active; `M2_DIR_PIN` |
| Board LED GPIO | Not on header | Digital output | Blinked by `wanderer_motor_test` only, not by the flight firmware | `PICO_DEFAULT_LED_PIN` from board definition |

“Reserved” means the assignment exists in `config.h`, but that subsystem is not
yet implemented in the current firmware.

**Treat the `wanderer_rflink` pins as taken, not free.** That target claims
**GP2** (link-good LED), **GP9** (nRF24 CSN), **GP14** (spi1 SCK), **GP15**
(spi1 MOSI), **GP21** (nRF24 CE), **GP22** (`PIN_ROLE`) and **GP28** (spi1
MISO) — see `firmware/rflink/main.cpp`. The two firmwares are never flashed at
the same time *today*, but architecture §3a plans an RF backdoor inside the
flight firmware, at which point those pins are claimed for real. Assigning any
of them to a new peripheral now buys a rework later.

After that exclusion the genuinely free header GPIOs are **GP3, GP26 and
GP27**. GP26/GP27 are an I²C1 SDA/SCL pair and are the natural fallback if the
IMU has to move off GP6/GP7.

PWM: RP2350 has 8 PWM slices / 16 channels; PWM1 (GP16) and PWM2 (GP19)
use separate channels. The firmware uses 20 kHz, the MDD10A maximum.
Encoders are decoded by two PIO state machines for accurate, CPU-light
quadrature counting. Each encoder's A/B must be a **consecutive GPIO pair**
(base pin + 1). Firmware counts every valid A/B transition (x4 decoding), so
`TICKS_PER_METER` calibration must use that same edge count. Forward wheel
motion should produce positive ticks; use `ENC_LEFT_SIGN` and `ENC_RIGHT_SIGN`
in `firmware/airframe/src/config.h` to correct polarity without rewiring A/B.

## Pico2 UART ↔ RPI5 UART — the cockpit link

The flight interface: three wires, crossed. Both boards are 3.3 V logic, so
they connect directly with no level shifter. Project convention calls the pilot
SBC "RPI5" whatever board is fitted; **the header pins below are the same on a
Pi 5 and a Zero 2 W**, only the Linux-side configuration differs.

| Pilot 40-pin header | | Airframe Pico 2 |
|---|---|---|
| Pin 8 — GPIO14, **TXD** | → | **GP1**, physical pin 2 (UART0 RX) |
| Pin 10 — GPIO15, **RXD** | ← | **GP0**, physical pin 1 (UART0 TX) |
| Pin 6 — GND | — | any Pico GND (e.g. physical pin 3) |

- TX→RX **crossover** on both signal wires. TX-to-TX is the classic silent
  failure: both sides talk, nobody listens.
- The GND wire is **not optional**, even with separately powered boards. A UART
  is a voltage referenced to ground; this is the shared reference (consistent
  with the starred-ground topology at the end of this file).
- Connect **nothing** to either board's supply pins over this harness — each
  board keeps its own power. The Pico's micro-USB carries a second, independent
  bench-log path (USB CDC `printf`), separate from the cockpit.

Enabling the header UART and — critically — disabling the Linux serial console
on it, plus the hand-typed bring-up session, are covered in
[`docs/cockpit_bench_test.md`](../docs/cockpit_bench_test.md). Pilot software
always opens `/dev/serial0` so it survives a board swap.

## Cytron MDD10A Rev 2.0 connections

Disconnect both battery packs before changing any wiring. The MDD10A uses
sign-magnitude control: one PWM and one direction input per motor. It accepts
3.3 V Pico logic directly and does not need a separate logic-supply connection.
See the installed-board [photo](../docs/cytron_mdd10A.jpeg) and
[Cytron product documentation](https://my.cytron.io/p-10amp-5v-30v-dc-motor-driver-2-channels).

| MDD10A terminal/pin | Connects to | Notes |
|---------------------|-------------|-------|
| POWER + | Motor pack 3S (+) | 5–30 V motor supply; **no reverse-polarity protection** |
| POWER - | Common ground | motor-pack negative and system ground |
| M1A / M1B | Left motor leads | swap the pair if left forward is reversed |
| M2A / M2B | Right motor leads | swap the pair if right forward is reversed |
| DIR1 | Pico GP17 (physical pin 22) | left direction |
| PWM1 | Pico GP16 (physical pin 21) | left speed, 20 kHz PWM |
| DIR2 | Pico GP20 (physical pin 26) | right direction |
| PWM2 | Pico GP19 (physical pin 25) | right speed, 20 kHz PWM |
| GND | Common ground | control-signal reference; required |

Control truth table in sign-magnitude mode:

| PWM | DIR | Output A | Output B |
|-----|-----|----------|----------|
| Low | X | Low | Low |
| High | Low | High | Low |
| High | High | Low | High |

The firmware defines a positive command as `DIR=LOW` and a negative command as
`DIR=HIGH`. A zero command sets PWM to zero. Motor lead orientation determines
which physical wheel direction is forward.

### Power-up check

1. Raise and secure the chassis so both driven wheels can rotate freely.
2. Verify continuity between Pico GND, MDD10A GND/POWER -, logic-pack GND, and
   motor-pack negative.
3. Verify motor-pack polarity at the disconnected MDD10A power leads. Reversed
   polarity can damage the board.
4. Power the regulated 5 V logic rail first; the Pico must hold both PWM inputs low.
5. Apply the motor pack to `POWER +` / `POWER -`; the motors must remain stopped.
6. Optionally use the M1A/M1B and M2A/M2B test buttons with the chassis raised
   to confirm each motor and its lead orientation.
7. Flash and run the motor hardware test described in
   [`firmware/README.md`](../firmware/README.md#motor-hardware-test-wanderer_motor_test).

Because the motors face opposite sides of the chassis, one output lead pair may
need to be swapped so that "BOTH forward" turns both wheels in the robot's
forward direction.

### Motor-pack fuse and switch

Use one **7.5 A ATO/ATC automotive blade fuse, rated 32 V DC**, in an inline
holder on the protected 3S pack's positive output. The 550 RPM MD520 is
specified at 3 A stall current, so two stalled motors demand approximately 6 A.
The 7.5 A fuse permits brief starting current while protecting the motor-power
wiring against a short circuit.

Wire the motor-power path in this order:

```text
3S cells → 3S BMS/protection board

BMS P+ → 7.5 A inline fuse → main switch/e-stop → MDD10A POWER +
BMS P- ─────────────────────────────────────────→ MDD10A POWER -
                                                    │
                                                    └→ common logic-ground bus
```

- Install the fuse holder as close as practical to `BMS P+`, preferably within
  10 cm. It protects the wire between the pack and the motor driver.
- Connect to the BMS protected output (`P+`/`P-`, or the equivalent labels on
  that board), not directly to raw cell terminals in a way that bypasses protection.
- Do not place the fuse in the negative/ground wire.
- Use 16 AWG or 18 AWG stranded copper for both power conductors and an inline
  fuse holder whose leads are at least the same gauge.
- The switch/e-stop must be DC-rated for at least 15 A at 12 V DC.
- The 3S BMS must support at least **7.5 A continuous** and the motor starting
  surge. If its specified continuous limit is lower, the BMS is the limiting
  component and must be replaced; fitting a larger fuse does not solve that.
- A 7.5 A fuse protects wiring from gross faults. It is not precise motor
  overload protection; firmware stall detection remains necessary.

## MD520 motor + encoder (per motor)

| Motor | Wire (typical) | Connects to | Notes |
|-------|----------------|-------------|-------|
| Left | Motor +/- | MDD10A M1A / M1B | 12 V via driver |
| Left | Encoder VCC | Pico 3V3 | NOT 5 V — RP2350 is not 5 V-tolerant |
| Left | Encoder GND | Common ground | — |
| Left | Encoder C1 (A) | Pico GP10 (physical pin 14) | quadrature channel A |
| Left | Encoder C2 (B) | Pico GP11 (physical pin 15) | quadrature channel B |
| Right | Motor +/- | MDD10A M2A / M2B | 12 V via driver |
| Right | Encoder VCC | Pico 3V3 | NOT 5 V — RP2350 is not 5 V-tolerant |
| Right | Encoder GND | Common ground | — |
| Right | Encoder C1 (A) | Pico GP12 (physical pin 16) | quadrature channel A |
| Right | Encoder C2 (B) | Pico GP13 (physical pin 17) | quadrature channel B |

> Confirm MD520 wire colors/pinout from the datasheet before wiring (verify in Phase 1).

## VL53L0X ToF (front)

| Pin | Connects to |
|-----|-------------|
| VIN | 3.3 V (Pico 3V3) |
| GND | Common ground |
| SDA / SCL | Pico I²C0 |
| XSHUT | Pico GPIO (optional, O5) |

Single sensor → default address 0x29, no collision handling needed yet.

## MinIMU-9 v6 IMU — on the airframe, not the pilot

Design spec: [`docs/imu_integration.md`](../docs/imu_integration.md).

| Pin | Connects to | Notes |
|-----|-------------|-------|
| VDD | Pico 3V3 (OUT) | on-board regulator + level shifter accept 2.5–5.5 V |
| GND | Common ground | — |
| SDA | Pico GP6 (physical pin 9) | I²C1 |
| SCL | Pico GP7 (physical pin 10) | I²C1 |
| INT1 | Pico GP18 (physical pin 24) | LSM6DSO gyro data-ready |

Addresses: LSM6DSO `0x6B` (SA0 pulled high on this carrier), LIS3MDL `0x1E`.
No collision with the VL53L0X (`0x29`) or the planned INA226s (`0x40`–`0x4F`)
even if the two buses were ever merged.

**Why I²C1 and not I²C0.** The gyro is read at 208 Hz and sits inside the
real-time control loop. A ToF ranging transaction or a stretched clock from a
misbehaving INA226 must never be able to delay it, so the slow non-real-time
sensors keep I²C0 and the one real-time sensor has I²C1 to itself.

**Why the airframe and not the pilot.** The heading loop is a Tier 2 firmware
procedure (architecture §3), so the gyro is inside a hard-real-time loop.
Reading it across a Linux I²C bus would reintroduce the scheduling jitter the
tactical layer exists to prevent — the same argument that puts the
wheel-velocity loop on the Pico2. If the pilot needs attitude for the camera or
Pan-Tilt, that calls for a *second* IMU on the pilot bus.

The board carries its own pull-ups (~10 kΩ), so add none. Keep the run under
about 15 cm and fit a 100 nF ceramic at the board's VDD pin. Mount it rigidly —
screwed or on standoffs, not foam tape, which both transmits vibration into the
gyro output as an apparent bias and creeps over time — with its Z axis vertical
and at least ~10 cm from the motors.

## Pilot SBC — I²C devices

| Device | Address | Bus |
|--------|---------|-----|
| Pan-Tilt HAT | 0x15 | pilot I²C |

The Pico is **not** on this bus — it reaches the pilot over the Pico2 UART ↔
RPI5 UART link.

Camera Module 3 connects via the **Zero-specific narrow FFC cable** (22-pin 0.5 mm → 15-pin);
a Pi 5 uses its own 22-pin cable instead.
The Pan-Tilt HAT physically overhangs the small Zero board but mounts on the 40-pin header.

## 5 V power distribution

| 5 V rail branch | Connects to | Notes |
|-----------------|-------------|-------|
| Pilot SBC | 5V + GND power input (O2) | GPIO 5V/GND recommended; USB power input still open |
| Pico 2 | VSYS + GND | separate branch from the rail; do not route through the pilot SBC |
| Pan-Tilt servos | 5 V + GND | dominant transient load; keep bulk cap nearby |

## Grounding rule (repeat — it matters)

```
motor pack (–) ─┐
logic pack (–) ─┤
MDD10A POWER - ─┼── ONE common ground node
Pico GND ───────┤
pilot SBC GND ──┘
```

The RPI5 UART ↔ Pico2 UART GND wire (header pin 6 → Pico GND) is part of this
same single ground node, not a second path around it.
