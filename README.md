# Wanderer

A small model-size R&D platform (hardware + software) for a **semi-autonomous exploration vehicle**.

Wanderer is a differential-drive robot (two driven wheels + front caster) built around a
three-tier control architecture:

- **Strategic + Perception** — Base station (Windows 11 PC, NVIDIA GPU): high-level control,
  mission planning, telemetry, teleop, **and GPU vision inference** on stills sent up by the robot.
- **Tactical** — Raspberry Pi Zero 2 W + Pi Camera 3 + Pan-Tilt: thin relay — captures periodic
  stills, state estimation (odometry + IMU), forwards commands to the reflexive layer.
- **Reflexive** — Raspberry Pi Pico 2 (RP2350): motor control, encoders, distance sensing, safety reflexes.

```
Browser ⇄ (HTTP/WebSocket) ⇄ FastAPI + GPU vision (PC) ⇄ (raw TCP/Wi-Fi) ⇄ Zero 2 W ⇄ (I²C) ⇄ Pico 2 ⇄ motors/sensors
 user                         base station                                  tactical          reflexive
                                                          ▲ periodic JPEG stills ┘
```

> **Perception runs off-board on the PC GPU** (the robot sends still images "now and then", not video).
> This is a "remote-brain" design: safety-critical jobs stay local — obstacle stopping on the Pico/ToF,
> continuous pose from odometry+IMU — so a dropped link still leaves the robot safe.

## Repository layout

| Path           | Contents                                                        |
|----------------|-----------------------------------------------------------------|
| `pico/`        | C/C++ firmware for the Pico 2 (reflexive layer, Pico SDK)       |
| `tactical/`    | Python for the Zero 2 W (tactical relay: stills, odometry, I²C) |
| `basestation/` | Python FastAPI + web UI + GPU perception (strategic layer)      |
| `protocol/`    | Shared message / I²C register / TCP protocol definitions        |
| `hardware/`    | Wiring diagrams, pinouts, BOM, power budget                     |
| `docs/`        | Architecture, decision log, plan of work                        |

## Documentation

- [Architecture](docs/00-architecture.md)
- [Decision log](docs/01-decisions.md)
- [Plan of work](docs/02-plan.md)
- [Power budget](hardware/power-budget.md)
- [Wiring & pinouts](hardware/wiring.md)
