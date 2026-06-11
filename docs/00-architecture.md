# Wanderer — Architecture

## Overview

Wanderer uses a **three-tier control hierarchy**, modeled loosely on a nervous system.
Each tier has a distinct timescale, responsibility, and physical processor.

| Tier          | Processor                         | Role                      | Timescale     |
|---------------|-----------------------------------|---------------------------|---------------|
| **Strategic** | Windows 11 PC + NVIDIA GPU        | mission / intent + **vision** | seconds–human |
| **Tactical**  | Raspberry Pi Zero 2 W             | relay / state estimation  | ~10–100 ms    |
| **Reflexive** | Raspberry Pi Pico 2 (RP2350)      | motors / reflexes         | ~1–10 ms      |

**Perception is off-board.** Heavy vision (people ID, semantic scene understanding, coarse
visual localization) runs on the **PC GPU** from **periodic JPEG stills** the robot sends up —
not a live video stream. This "remote-brain" design trades on-board autonomy for far lower
power/weight/bulk on the robot.

The guiding principle: **lower tiers keep the robot safe and stable without the upper tiers**.
The time-critical jobs are deliberately kept local and link-independent:
- **Obstacle stopping** → VL53L0X ToF + Pico reflex (not the camera).
- **Continuous pose** → wheel odometry + IMU dead-reckoning (not visual SLAM); the camera
  contributes only *occasional* landmark/semantic correction.

So if the link drops, the Pico still stops the robot (command-timeout watchdog + obstacle
reflex) and the Zero holds a safe state; if the Zero/PC stalls, nothing time-critical is lost.

## Data flow

```
                 user
                  │
          HTTP / WebSocket
                  │
        ┌──────────────────────┐
        │  Base station         │   Windows 11 PC + NVIDIA GPU
        │  FastAPI + Web UI      │   Strategic + Perception
        │  GPU vision inference  │   - object/people ID on stills
        └─────────┬────────────┘
                  │  raw TCP over Wi-Fi (shared network); periodic JPEG stills up
                  │  (ESP-01S backup low-rate radio / NRF24 — future)
        ┌─────────▼─────────┐
        │  Raspberry Pi Zero 2 W │ Tactical (thin relay)
        │  Python                │  - capture periodic stills (Cam 3) → PC
        │  Cam 3, MinIMU-9       │  - state estimation (wheel odom + IMU)
        │  Pan-Tilt              │  - forward motion intent → Pico
        └─────────┬─────────┘
                  │  I²C  (Zero 2 W = master, Pico = peripheral)
        ┌─────────▼─────────┐
        │  Raspberry Pi Pico 2│  Reflexive
        │  C/C++ (Pico SDK)   │  - PWM/DIR motor control via MDD10A
        │                     │  - quadrature encoders (PIO)
        │                     │  - PID velocity control
        │                     │  - VL53L0X ToF (I²C master)
        │                     │  - safety reflexes + watchdog
        └──┬──────────┬──────┘
           │          │
      MDD10A        VL53L0X
       │   │         (front)
     Motor Motor
      1     2
   (+encoders)
```

## Tier responsibilities

### Strategic + Perception — Base station (Windows 11 PC, NVIDIA GPU)
- Web UI in the browser for the operator.
- Mission / waypoint commanding; manual teleoperation override.
- **GPU vision inference** on the periodic stills the robot uploads (people ID, object/
  scene understanding, coarse visual localization). Stack TBD (CUDA — PyTorch / ONNX
  Runtime / TensorRT); decision deferred.
- Live telemetry display (pose, velocity, battery, sensors) + image view.
- Data logging and playback for R&D.
- Talks to the Zero 2 W over **raw TCP** (Wi-Fi, shared network).

### Tactical — Raspberry Pi Zero 2 W (thin relay)
- Camera pipeline: capture **periodic JPEG stills** (hardware-encoded) and upload to the PC.
- **State estimation**: fuses wheel odometry (from Pico) with the **MinIMU-9** IMU
  (continuous dead-reckoning pose; corrected occasionally by PC vision).
- Controls the **Pan-Tilt HAT** (camera aim).
- Master on the I²C link to the Pico; translates tactical intent into wheel-velocity targets.
- TCP server/endpoint for the base station.
- *Not* doing on-board heavy vision or visual SLAM — that's the PC's job.

### Reflexive — Pico 2 (RP2350)
- Drives both motors via the **Cytron MDD10A Rev 2.0** using sign-magnitude
  control (PWM1/DIR1 and PWM2/DIR2).
- Reads **quadrature encoders** (via PIO state machines) → wheel odometry & velocity.
- **Closed-loop PID** velocity control per wheel.
- Reads the front **VL53L0X** ToF sensor (Pico is I²C master here).
- **Safety reflexes**: stop on imminent collision; **watchdog** stops motors if no
  command arrives from the tactical layer within a timeout.
- I²C **peripheral** to the Zero 2 W (register-style command/telemetry interface).

## I²C bus topology

The Pico participates in **two** I²C buses:

- **I²C0 (master)** → VL53L0X ToF sensor(s). The Pico initiates these transactions.
- **I²C1 (peripheral)** → Raspberry Pi Zero 2 W. The Zero is master; the Pico responds.

The RP2350 has two independent I²C controllers, so these roles do not conflict.

On the **Zero 2 W side**, the I²C devices are the Pico (peripheral), the MinIMU-9
(LSM6DSO @ 0x6B, LIS3MDL @ 0x1E), and the Pan-Tilt HAT (@ 0x15) — no address
collisions. The Pico will likely be placed on its own I²C bus for isolation
(decision deferred; see decision log).

## Communication contracts

Two protocol boundaries, both defined in `protocol/` as the single source of truth:

1. **PC ↔ Zero 2 W** — raw TCP message framing (commands + telemetry + periodic stills).
2. **Zero 2 W ↔ Pico** — I²C register map (commands + telemetry).

Keeping these in `protocol/` lets all three codebases agree on the same definitions.
**The Pico-side contract (`i2c_registers.*`) is unchanged by the board swap** — the
tactical processor is interchangeable behind it.
