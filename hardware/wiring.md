# Wanderer — Wiring & Pinouts

> Draft. Concrete Pico GPIO numbers are assigned in Phase 2 and reflected here.
> The counts/roles below are settled; exact pin numbers are TBD.

## Pico 2 (RP2350) GPIO budget

~14 of 26 usable GPIO. Comfortable headroom.

| Function | Signals | Pins | Notes |
|----------|---------|------|-------|
| Motor A (L298N) | ENA (PWM), IN1, IN2 | 3 | ENA on a PWM-capable pin |
| Motor B (L298N) | ENB (PWM), IN3, IN4 | 3 | ENB on a PWM-capable pin |
| Encoder A | A, B channels | 2 | quadrature via PIO; 3.3 V logic |
| Encoder B | A, B channels | 2 | quadrature via PIO; 3.3 V logic |
| I²C0 (master → ToF) | SDA, SCL | 2 | VL53L0X front sensor |
| ToF XSHUT (O5) | XSHUT | 1 | optional but recommended |
| I²C1 (peripheral ← Pi 5) | SDA, SCL | 2 | Pico = I²C slave |

PWM: RP2350 has 8 PWM slices / 16 channels — ENA & ENB trivially covered.
Encoders: decoded by PIO state machines for accurate, CPU-light quadrature counting.

## L298N connections

| L298N pin | Connects to | Notes |
|-----------|-------------|-------|
| +12V (VS) | Motor pack 3S (+) | motor supply |
| GND | Common ground | shared with everything |
| +5V | (leave per jumper) | onboard reg NOT used for Pi 5 |
| ENA / ENB | Pico PWM pins | speed (PWM) |
| IN1 / IN2 | Pico GPIO | Motor A direction |
| IN3 / IN4 | Pico GPIO | Motor B direction |
| OUT1 / OUT2 | Motor A | — |
| OUT3 / OUT4 | Motor B | — |

Pico GPIO is 3.3 V; L298N inputs accept 3.3 V as logic HIGH.

## MD520 motor + encoder (per motor)

| Wire (typical) | Connects to | Notes |
|----------------|-------------|-------|
| Motor +/- | L298N OUT pair | 12 V via driver |
| Encoder VCC | **3.3 V** (Pico 3V3) | NOT 5 V — RP2350 not 5 V-tolerant |
| Encoder GND | Common ground | — |
| Encoder C1 (A) | Pico GPIO | quadrature ch A |
| Encoder C2 (B) | Pico GPIO | quadrature ch B |

> Confirm MD520 wire colors/pinout from the datasheet before wiring (verify in Phase 1).

## VL53L0X ToF (front)

| Pin | Connects to |
|-----|-------------|
| VIN | 3.3 V (Pico 3V3) |
| GND | Common ground |
| SDA / SCL | Pico I²C0 |
| XSHUT | Pico GPIO (optional, O5) |

Single sensor → default address 0x29, no collision handling needed yet.

## Raspberry Pi 5 — I²C devices

| Device | Address | Bus |
|--------|---------|-----|
| Pico 2 (peripheral) | TBD (e.g. 0x42) | dedicated bus? (O3) |
| MinIMU-9 v6 — LSM6DSO | 0x6B | Pi 5 I²C |
| MinIMU-9 v6 — LIS3MDL | 0x1E | Pi 5 I²C |
| Pan-Tilt HAT | 0x15 | Pi 5 I²C |

No address conflicts. Pico may get its own Pi 5 I²C bus for isolation (O3).

## Grounding rule (repeat — it matters)

```
motor pack (–) ─┐
logic pack (–) ─┤
L298N GND ──────┼── ONE common ground node
Pico GND ───────┤
Pi 5 GND ───────┘
```
