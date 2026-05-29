# Wanderer — Plan of Work

Phased plan. Each phase is detailed and executed in turn. ✅ done · 🚧 in progress · ⬜ not started

## Phase 0 — Project foundation 🚧
- [x] Agree three-tier architecture & responsibilities
- [x] Choose software stack (custom lightweight)
- [x] Settle key hardware decisions (driver, sensors, power, IMU, camera)
- [x] Initialize repo + directory structure
- [x] Write architecture, decision log, plan, power & wiring docs
- [ ] Resolve open items O1–O5 (see decision log)

## Phase 1 — Hardware definition & wiring ⬜
- [ ] Finalize BOM (incl. buck converter, wiring, connectors, mounting)
- [ ] Power architecture: pack configs, buck, distribution, fusing, common ground
- [ ] Wiring & pinout maps: Pico↔L298N, Pico↔encoders, Pico↔ToF, Pico↔Pi 5 (I²C),
      Pi 5↔camera/Hailo/IMU/Pan-Tilt
- [ ] Bench-test each subsystem in isolation (motors, encoders, ToF, IMU, link)

## Phase 2 — Reflexive layer (Pico 2) ⬜  ← next focus
- [ ] Project skeleton (Pico SDK, CMake), blink/UART sanity
- [ ] Motor PWM + direction via L298N
- [ ] Quadrature encoder reading via PIO → ticks, distance, velocity
- [ ] Per-wheel closed-loop PID velocity control
- [ ] VL53L0X ToF reading (I²C0 master)
- [ ] Safety reflexes: collision auto-stop + command-timeout watchdog
- [ ] I²C peripheral interface (I²C1) — register map for commands/telemetry

## Phase 3 — Tactical layer (Raspberry Pi 5) ⬜
- [ ] I²C master driver + command/telemetry protocol (mirror of Phase 2)
- [ ] Camera pipeline + Hailo 8L inference
- [ ] State estimation (wheel odometry + MinIMU-9 fusion)
- [ ] Pan-Tilt control
- [ ] Local motion behaviors (drive-to-target, obstacle avoidance)
- [ ] TCP server + protocol to base station

## Phase 4 — Strategic layer (Base station, Windows) ⬜
- [ ] FastAPI server + TCP client to Pi 5
- [ ] Web UI: telemetry dashboard (pose, speed, battery, sensors)
- [ ] Manual teleoperation / override
- [ ] Mission / waypoint commanding
- [ ] Data logging & playback

## Phase 5 — Integration & autonomy ⬜
- [ ] End-to-end command + telemetry paths
- [ ] Mapping / exploration behavior (frontier exploration or light SLAM)
- [ ] Camera video streaming (deferred from D11)
- [ ] Field testing, tuning, iteration
- [ ] (Future) NRF24 outdoor link
