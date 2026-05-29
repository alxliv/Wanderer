# Wanderer — Wiring & Pinouts

> Draft. Concrete Pico GPIO numbers are assigned in Phase 2 and reflected here.
> The counts/roles below are settled; exact pin numbers are TBD.

## Pico 2 (RP2350) GPIO budget

~15 of 26 usable GPIO. Comfortable headroom.
**Source of truth for pins:** [`pico/src/config.h`](../pico/src/config.h) — keep this table in sync with it.

| Function | Signal | GPIO | Notes |
|----------|--------|------|-------|
| Motor A (left) | ENA (PWM) | GP16 | speed |
| | IN1 | GP17 | direction |
| | IN2 | GP18 | direction |
| Motor B (right) | ENB (PWM) | GP19 | speed |
| | IN3 | GP20 | direction |
| | IN4 | GP21 | direction |
| Encoder A (left) | A / B | GP10 / GP11 | consecutive pair for PIO; 3.3 V logic |
| Encoder B (right) | A / B | GP12 / GP13 | consecutive pair for PIO; 3.3 V logic |
| I²C0 (master → ToF) | SDA / SCL | GP4 / GP5 | VL53L0X front sensor |
| ToF XSHUT (O5) | XSHUT | GP8 | optional reset/boot control |
| I²C1 (peripheral ← Pi 5) | SDA / SCL | GP6 / GP7 | Pico = I²C slave @ 0x42 |
| UART0 debug | TX / RX | GP0 / GP1 | `printf` console @ 115200 |

PWM: RP2350 has 8 PWM slices / 16 channels — ENA (GP16) & ENB (GP19) trivially covered.
Encoders: decoded by PIO state machines for accurate, CPU-light quadrature counting;
each encoder's A/B must be a **consecutive GPIO pair** (base pin + 1).
External **4.7 kΩ pull-ups** recommended on the Pi 5 I²C bus (internal pull-ups are weak).

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
