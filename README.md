# Wanderer

A small model-size R&D platform (hardware + software) for a **semi-autonomous exploration vehicle**.

Wanderer is a differential-drive robot (two driven wheels + front caster) built around a
three-tier control architecture:

- **Strategic** — Base station (Windows PC): high-level control, mission planning, telemetry, teleop.
- **Tactical** — Raspberry Pi 5 + Hailo 8L AI Kit + Pi Camera 3: perception, state estimation, behaviors.
- **Reflexive** — Raspberry Pi Pico 2 (RP2350): motor control, encoders, distance sensing, safety reflexes.

```
Browser  ⇄ (HTTP/WebSocket) ⇄  FastAPI (PC)  ⇄ (raw TCP/Wi-Fi) ⇄  Pi 5  ⇄ (I²C) ⇄  Pico 2  ⇄  motors/sensors
 user                          base station                      tactical          reflexive
```

## Repository layout

| Path           | Contents                                                        |
|----------------|-----------------------------------------------------------------|
| `pico/`        | C/C++ firmware for the Pico 2 (reflexive layer, Pico SDK)       |
| `rpi5/`        | Python (+ some C/C++) for the Pi 5 (tactical layer)             |
| `basestation/` | Python FastAPI server + web UI (strategic layer)               |
| `protocol/`    | Shared message / I²C register / TCP protocol definitions        |
| `hardware/`    | Wiring diagrams, pinouts, BOM, power budget                     |
| `docs/`        | Architecture, decision log, plan of work                        |

## Documentation

- [Architecture](docs/00-architecture.md)
- [Decision log](docs/01-decisions.md)
- [Plan of work](docs/02-plan.md)
- [Power budget](hardware/power-budget.md)
- [Wiring & pinouts](hardware/wiring.md)
