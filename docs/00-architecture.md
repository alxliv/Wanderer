# Wanderer — Architecture

## Overview

Wanderer uses a **three-tier control hierarchy**, modeled loosely on a nervous system.
Each tier has a distinct timescale, responsibility, and physical processor.

| Tier          | Processor                         | Role             | Timescale     |
|---------------|-----------------------------------|------------------|---------------|
| **Strategic** | Windows PC (base station)         | mission / intent | seconds–human |
| **Tactical**  | Raspberry Pi 5 + Hailo 8L         | perception / nav | ~10–100 ms    |
| **Reflexive** | Raspberry Pi Pico 2 (RP2350)      | motors / reflexes| ~1–10 ms      |

The guiding principle: **lower tiers keep the robot safe and stable without the upper tiers**.
If the Pi 5 crashes or the link drops, the Pico must still stop the robot safely
(command-timeout watchdog + obstacle reflex). If the base station disconnects, the Pi 5
holds a safe state.

## Data flow

```
                 user
                  │
          HTTP / WebSocket
                  │
        ┌─────────▼─────────┐
        │  Base station      │   Windows PC
        │  FastAPI + Web UI  │   Strategic
        └─────────┬─────────┘
                  │  raw TCP over Wi-Fi (shared network)
                  │  (NRF24 fallback for outdoor range — future)
        ┌─────────▼─────────┐
        │  Raspberry Pi 5    │   Tactical
        │  Python (+ C/C++)  │   - camera + Hailo 8L inference
        │  Hailo 8L, Cam 3   │   - state estimation (wheel odom + IMU)
        │  MinIMU-9, Pan-Tilt│   - motion behaviors, obstacle avoidance
        └─────────┬─────────┘
                  │  I²C  (Pi 5 = master, Pico = peripheral)
        ┌─────────▼─────────┐
        │  Raspberry Pi Pico 2│  Reflexive
        │  C/C++ (Pico SDK)   │  - PWM motor control via L298N
        │                     │  - quadrature encoders (PIO)
        │                     │  - PID velocity control
        │                     │  - VL53L0X ToF (I²C master)
        │                     │  - safety reflexes + watchdog
        └──┬──────────┬──────┘
           │          │
       L298N        VL53L0X
       │   │         (front)
     Motor Motor
      A     B
   (+encoders)
```

## Tier responsibilities

### Strategic — Base station (PC)
- Web UI in the browser for the operator.
- Mission / waypoint commanding; manual teleoperation override.
- Live telemetry display (pose, velocity, battery, sensors).
- Data logging and playback for R&D.
- Talks to the Pi 5 over **raw TCP** (Wi-Fi, shared network).

### Tactical — Raspberry Pi 5
- Camera pipeline + **Hailo 8L** inference (object / obstacle recognition).
- **State estimation**: fuses wheel odometry (from Pico) with the **MinIMU-9** IMU.
- Local motion behaviors: drive-to-target, obstacle avoidance, exploration.
- Controls the **Pan-Tilt HAT** (camera aim).
- Master on the I²C link to the Pico; translates tactical intent into wheel-velocity targets.
- TCP server/endpoint for the base station.

### Reflexive — Pico 2 (RP2350)
- Drives both motors via the **L298N** (PWM on ENA/ENB, direction on IN1–IN4).
- Reads **quadrature encoders** (via PIO state machines) → wheel odometry & velocity.
- **Closed-loop PID** velocity control per wheel.
- Reads the front **VL53L0X** ToF sensor (Pico is I²C master here).
- **Safety reflexes**: stop on imminent collision; **watchdog** stops motors if no
  command arrives from the Pi 5 within a timeout.
- I²C **peripheral** to the Pi 5 (register-style command/telemetry interface).

## I²C bus topology

The Pico participates in **two** I²C buses:

- **I²C0 (master)** → VL53L0X ToF sensor(s). The Pico initiates these transactions.
- **I²C1 (peripheral)** → Raspberry Pi 5. The Pi 5 is master; the Pico responds.

The RP2350 has two independent I²C controllers, so these roles do not conflict.

On the **Pi 5 side**, the I²C devices are the Pico (peripheral), the MinIMU-9
(LSM6DSO @ 0x6B, LIS3MDL @ 0x1E), and the Pan-Tilt HAT (@ 0x15) — no address
collisions. The Pico will likely be placed on its own Pi 5 I²C bus for isolation
(decision deferred; see decision log).

## Communication contracts

Two protocol boundaries, both defined in `protocol/` as the single source of truth:

1. **PC ↔ Pi 5** — raw TCP message framing (commands + telemetry).
2. **Pi 5 ↔ Pico** — I²C register map (commands + telemetry).

Keeping these in `protocol/` lets all three codebases agree on the same definitions.
