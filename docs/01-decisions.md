# Wanderer — Decision Log

A running record of design decisions. Append new entries; don't rewrite history.
Status: ✅ decided · 🔵 recommended/default · ⏳ open

| # | Topic | Decision | Status |
|---|-------|----------|--------|
| D1 | Drive type | Differential drive: 2 driven wheels + front caster | ✅ |
| D2 | Software stack | Custom lightweight stack (not ROS 2) | ✅ |
| D3 | Base station | Python + FastAPI backend, web UI (HTML/JS) in browser, on Windows PC | ✅ |
| D4 | PC ↔ Pi 5 link | Raw TCP over Wi-Fi (shared network); NRF24 considered for outdoor range later | ✅ |
| D5 | Pi 5 language | Mostly Python, some C/C++ allowed | ✅ |
| D6 | Pico 2 language | C/C++ only (Pico SDK) | ✅ |
| D7 | Motor driver | L298N dual H-bridge | ✅ |
| D8 | Distance sensor | VL53L0X ToF — **start with one, front-facing** | ✅ |
| D9 | IMU | Pololu MinIMU-9 v6 (LSM6DSO + LIS3MDL), **mounted on the Pi 5** | ✅ |
| D10 | Camera | Pi Camera Module 3 + Pan-Tilt HAT (controlled by Pi 5) | ✅ |
| D11 | Camera video stream | **Deferred** to a later phase | ✅ |
| D12 | Motors | 2× MD520 (JGB37-520), 12 V, 550 RPM, with quadrature Hall encoders | ✅ |
| D13 | Motor power | 3S Li-ion (3× 18650) ≈ 11.1 V nom / 12.6 V full | ✅ |
| D14 | Logic power | 2S Li-ion (2× 18650) ≈ 7.4 V → buck converter → 5 V rail | ✅ |
| D15 | Battery protection | 18650 protection boards present on both packs | ✅ |
| D16 | Common ground | All grounds tied together (both packs, L298N, Pico, Pi 5) | ✅ |
| D17 | Encoder logic voltage | Encoders powered at **3.3 V** (RP2350 GPIO is not 5 V-tolerant) | ✅ |
| D18 | Pico encoder decode | Quadrature decoded via **PIO** state machines | ✅ |
| D19 | Pico I²C topology | I²C0 = master (ToF), I²C1 = peripheral (to Pi 5) | ✅ |
| D20 | Pico power | Powered from the Pi 5's 5 V rail | 🔵 |
| D21 | I²C protocol style | Register-map style command/telemetry interface | 🔵 |
| D22 | Build order | Start with Phase 2 (Pico firmware) after scaffolding | ✅ |

## Open / to confirm

- **O1 — Buck converter spec.** Need ≥ 5 A (ideally 6 A) at 5 V to cover Pi 5 + Pan-Tilt
  servos. Do we have one, and what is its rating? Otherwise spec in BOM.
- **O2 — Pi 5 power entry.** Feed 5 V into the GPIO 5V/GND pins (bypasses USB-C PD,
  may show low-power warning, quietable via config) **or** via a USB-C cable from the buck?
- **O3 — Pico Pi-5 I²C bus.** Put the Pico on its own dedicated Pi 5 I²C bus for isolation,
  or share the bus with the IMU + Pan-Tilt HAT? (No address conflict either way.)
- **O4 — Pi 5 logic-pack runtime.** 2S (no parallel) gives limited runtime under
  Pi 5 + Hailo + servo load. Acceptable to start; future upgrade path is 2S2P.
- **O5 — ToF XSHUT.** Wire the VL53L0X XSHUT to a Pico GPIO for clean reset/boot control?
  (Recommended even for a single sensor.)

## Notes on tradeoffs accepted

- **L298N voltage drop (~2–3 V)**: motors effectively see ~9–10 V → slightly lower top
  speed. Accepted for prototype.
- **L298N current limit (~2 A/ch continuous)**: rely on Pico stall/timeout protection;
  avoid prolonged stalls. Accepted for prototype.
- **L298N onboard 5 V regulator** is too weak (~0.5 A) for the Pi 5 — **not used** for it.
