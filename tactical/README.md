# Wanderer — Tactical layer (Raspberry Pi Zero 2 W)

Python for the tactical tier. The Zero 2 W is a **thin relay**, not the brain:

- captures **periodic JPEG stills** from the Pi Camera 3 and uploads them to the PC
- **I²C master** to the Pico (commands + telemetry, per [`protocol/i2c_registers.md`](../protocol/i2c_registers.md))
- reads the **MinIMU-9** and fuses with wheel odometry for dead-reckoning pose
- drives the **Pan-Tilt HAT** (camera aim)
- **TCP** server/endpoint to the base station ([`protocol/tcp_messages.md`](../protocol/tcp_messages.md), TBD)

Heavy vision (people ID, scene understanding, localization) runs **off-board on the PC GPU** —
see [docs/00-architecture.md](../docs/00-architecture.md). Safety-critical behavior stays on the
Pico (ToF reflex + watchdog), independent of this layer and the link.

## Status
⬜ Phase 3 — not started. Pico-side contract is ready; this consumes it.

## Target
- Raspberry Pi OS (64-bit) on the Zero 2 W
- Camera Module 3 via the **Zero-specific narrow FFC cable**
- Python 3 (`python3-libcamera`/`picamera2`, `smbus2` for I²C, plain sockets for TCP)
