# Wanderer — Wiring & Pinouts

> Draft. Concrete Pico GPIO numbers are assigned in Phase 2 and reflected here.
> The counts/roles and Pico pin numbers below are assigned; keep them aligned with firmware.

## Pico 2 (RP2350) GPIO summary

The complete design reserves **15 header GPIOs**: 13 assigned in
[`pico/src/config.h`](../pico/src/config.h), plus GP0/GP1 assigned to UART0 by
[`pico/CMakeLists.txt`](../pico/CMakeLists.txt). The firmware also toggles the
board-defined onboard LED GPIO, which is not an external wiring connection.

`pico/src/config.h` is the source of truth for application pin assignments.
Keep this table and the detailed connection tables below synchronized with it.

| GPIO | Physical pin | Direction / function | Connected device and use | Status / source |
|------|--------------|----------------------|--------------------------|-----------------|
| GP0 | 1 | UART0 TX, output | Debug `printf` data to USB-serial adapter RX, 115200 baud | Active; `pico/CMakeLists.txt` |
| GP1 | 2 | UART0 RX, input | Debug console receive from USB-serial adapter TX | Active; `pico/CMakeLists.txt` |
| GP4 | 6 | I²C0 SDA, bidirectional | VL53L0X front ToF data; Pico is I²C master | Reserved; `TOF_SDA_PIN` |
| GP5 | 7 | I²C0 SCL, output | VL53L0X front ToF clock | Reserved; `TOF_SCL_PIN` |
| GP6 | 9 | I²C1 SDA, bidirectional | Zero 2 W ↔ Pico command/telemetry data; Pico peripheral at `0x42` | Active; `PI_SDA_PIN` |
| GP7 | 10 | I²C1 SCL, input | Clock from Zero 2 W I²C master | Active; `PI_SCL_PIN` |
| GP8 | 11 | Digital output | VL53L0X `XSHUT` reset/enable | Reserved/optional; `TOF_XSHUT_PIN` |
| GP10 | 14 | PIO digital input | Left MD520 encoder C1 / channel A | Reserved; `ENC_A_PIN_BASE` |
| GP11 | 15 | PIO digital input | Left MD520 encoder C2 / channel B | Reserved; `ENC_A_PIN_BASE + 1` |
| GP12 | 16 | PIO digital input | Right MD520 encoder C1 / channel A | Reserved; `ENC_B_PIN_BASE` |
| GP13 | 17 | PIO digital input | Right MD520 encoder C2 / channel B | Reserved; `ENC_B_PIN_BASE + 1` |
| GP16 | 21 | PWM output | MDD10A `PWM1`, left motor speed at 20 kHz | Active; `M1_PWM_PIN` |
| GP17 | 22 | Digital output | MDD10A `DIR1`, left motor direction | Active; `M1_DIR_PIN` |
| GP19 | 25 | PWM output | MDD10A `PWM2`, right motor speed at 20 kHz | Active; `M2_PWM_PIN` |
| GP20 | 26 | Digital output | MDD10A `DIR2`, right motor direction | Active; `M2_DIR_PIN` |
| Board LED GPIO | Not on header | Digital output | One-second firmware heartbeat | Active; `PICO_DEFAULT_LED_PIN` from board definition |

“Reserved” means the assignment exists in `config.h`, but that subsystem is not
yet implemented in the current firmware. All other header GPIOs are presently
unassigned. GP18 and GP21, formerly used by the L298N interface, are free.

PWM: RP2350 has 8 PWM slices / 16 channels; PWM1 (GP16) and PWM2 (GP19)
use separate channels. The firmware uses 20 kHz, the MDD10A maximum.
Encoders: decoded by PIO state machines for accurate, CPU-light quadrature counting;
each encoder's A/B must be a **consecutive GPIO pair** (base pin + 1).
External **4.7 kΩ pull-ups** recommended on the Zero 2 W I²C bus (internal pull-ups are weak).

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
   [`pico/README.md`](../pico/README.md#motor-tests).

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

## Raspberry Pi Zero 2 W — I²C devices

| Device | Address | Bus |
|--------|---------|-----|
| Pico 2 (peripheral) | 0x42 | dedicated bus? (O3) |
| MinIMU-9 v6 — LSM6DSO | 0x6B | Zero I²C |
| MinIMU-9 v6 — LIS3MDL | 0x1E | Zero I²C |
| Pan-Tilt HAT | 0x15 | Zero I²C |

No address conflicts. Pico may get its own I²C bus for isolation (O3).
Camera Module 3 connects via the **Zero-specific narrow FFC cable** (22-pin 0.5 mm → 15-pin).
The Pan-Tilt HAT physically overhangs the small Zero board but mounts on the 40-pin header.

## 5 V power distribution

| 5 V rail branch | Connects to | Notes |
|-----------------|-------------|-------|
| Zero 2 W | 5V + GND power input (O2) | GPIO 5V/GND recommended; micro-USB still open |
| Pico 2 | VSYS + GND | separate branch from the rail; do not route through Zero |
| Pan-Tilt servos | 5 V + GND | dominant transient load; keep bulk cap nearby |

## Grounding rule (repeat — it matters)

```
motor pack (–) ─┐
logic pack (–) ─┤
MDD10A POWER - ─┼── ONE common ground node
Pico GND ───────┤
Zero 2 W GND ───┘
```
