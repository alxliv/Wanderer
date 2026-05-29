# Wanderer — Decision Log

A running record of design decisions. Append new entries; don't rewrite history.
Status: ✅ decided · 🔵 recommended/default · ⏳ open

| # | Topic | Decision | Status |
|---|-------|----------|--------|
| D1 | Drive type | Differential drive: 2 **front** driven wheels + **rear** caster | ✅ |
| D2 | Software stack | Custom lightweight stack (not ROS 2) | ✅ |
| D3 | Base station | Python + FastAPI backend, web UI (HTML/JS) in browser, on Windows PC | ✅ |
| D4 | PC ↔ tactical host link | Raw TCP over Wi-Fi (shared network); NRF24 considered for outdoor range later | ✅ |
| D5 | Tactical host language | Mostly Python on the Zero 2 W, some C/C++ allowed | ✅ |
| D6 | Pico 2 language | C/C++ only (Pico SDK) | ✅ |
| D7 | Motor driver | L298N dual H-bridge | ✅ |
| D8 | Distance sensor | VL53L0X ToF — **start with one, front-facing** | ✅ |
| D9 | IMU | Pololu MinIMU-9 v6 (LSM6DSO + LIS3MDL), mounted on the tactical SBC (now Zero 2 W — see D23) | ✅ |
| D10 | Camera | Pi Camera Module 3 + Pan-Tilt HAT (now on Zero 2 W via Zero FFC cable — see D23) | ✅ |
| D11 | Camera video stream | **Deferred** to a later phase | ✅ |
| D12 | Motors | 2× MD520 (JGB37-520), 12 V, 550 RPM, with quadrature Hall encoders | ✅ |
| D13 | Motor power | 3S Li-ion (3× 18650) ≈ 11.1 V nom / 12.6 V full | ✅ |
| D14 | Logic power | 2S Li-ion (2× 18650) ≈ 7.4 V → buck converter → 5 V rail | ✅ |
| D15 | Battery protection | 18650 protection boards present on both packs | ✅ |
| D16 | Common ground | All grounds tied together (both packs, motor driver, Pico, Zero 2 W) | ✅ |
| D17 | Encoder logic voltage | Encoders powered at **3.3 V** (RP2350 GPIO is not 5 V-tolerant) | ✅ |
| D18 | Pico encoder decode | Quadrature decoded via **PIO** state machines | ✅ |
| D19 | Pico I²C topology | I²C0 = master (ToF), I²C1 = peripheral (to Zero 2 W) | ✅ |
| D20 | Pico power | Powered from the tactical SBC's 5 V rail (now Zero 2 W) | 🔵 |
| D21 | I²C protocol style | Register-map style command/telemetry interface | 🔵 |
| D22 | Build order | Start with Phase 2 (Pico firmware) after scaffolding | ✅ |
| D23 | **Tactical board** | **Raspberry Pi Zero 2 W** selected for lower power/weight/bulk; built-in 2.4 GHz Wi-Fi; ARM64 quad-core | ✅ |
| D24 | **Perception location** | **Off-board on the PC GPU** (Windows 11 + NVIDIA). Robot has no on-board heavy vision | ✅ |
| D25 | **Vision data path** | Robot sends **periodic JPEG stills** (not video) over TCP; sufficient for people ID / semantics / coarse localization | ✅ |
| D26 | **Hailo 8L** | **Shelved** — no PCIe/M.2 on the Zero; PC GPU does inference instead | ✅ |
| D27 | Autonomy model | "Remote-brain": safety-critical jobs stay local (ToF reflex; odom+IMU pose). No camera-based visual SLAM | ✅ |
| D28 | Pose estimation | Wheel odometry + IMU dead-reckoning, with occasional camera/landmark correction from the PC | ✅ |
| D29 | ESP-01S | Reserved as a **future backup low-rate command/telemetry radio** (not the primary link) | 🔵 |
| D30 | PC inference stack | CUDA on NVIDIA GPU; framework TBD (PyTorch / ONNX Runtime / TensorRT) | ⏳ |
| D31 | 5 V regulator | **Pololu S13V30F5 (#4082)** buck-boost, 5 V, ~3 A @ 2S, 2.8–22 V in; + 470–1000 µF bulk cap. Holds 5 V across full 2S discharge (resolves O1) | ✅ |

## Open / to confirm

- ~~**O1 — Buck converter spec.**~~ ✅ **RESOLVED (D31):** Pololu S13V30F5 (#4082) buck-boost,
  5 V ~3 A, + bulk cap. See [hardware/bom.md](../hardware/bom.md).
- **O2 — Zero 2 W power entry.** 5 V into the GPIO 5V/GND pins (recommended) or via the
  micro-USB power port from the buck?
- **O3 — Pico I²C bus.** Put the Pico on its own dedicated Zero I²C bus for isolation,
  or share the bus with the IMU + Pan-Tilt HAT? (No address conflict either way.)
- ~~**O4 — Logic-pack runtime.**~~ ✅ **Largely resolved** by the Zero's low draw; 2S2P
  remains an easy upgrade if testing shows the servo duty cycle needs more capacity.
- **O5 — ToF XSHUT.** Wire the VL53L0X XSHUT to a Pico GPIO for clean reset/boot control?
  (Recommended even for a single sensor.)
- **O6 — Still cadence/resolution.** Target frame rate and JPEG resolution for the uploads
  (affects perceived responsiveness and 2.4 GHz Wi-Fi headroom). Defer to Phase 3.

## Notes on tradeoffs accepted

- **L298N voltage drop (~2–3 V)**: motors effectively see ~9–10 V → slightly lower top
  speed. Accepted for prototype.
- **L298N current limit (~2 A/ch continuous)**: rely on Pico stall/timeout protection;
  avoid prolonged stalls. Accepted for prototype.
- **L298N onboard 5 V regulator** is too weak (~0.5 A) for the logic rail — **not used**;
  the dedicated buck feeds the Zero 2 W and servos.
