# Wanderer — Plan of Work

Phased plan. Each phase is detailed and executed in turn. ✅ done · 🚧 in progress · ⬜ not started

## Phase 0 — Project foundation 🚧
- [x] Agree three-tier architecture & responsibilities
- [x] Choose software stack (custom lightweight)
- [x] Settle key hardware decisions (driver, sensors, power, IMU, camera)
- [x] Initialize repo + directory structure
- [x] Write architecture, decision log, plan, power & wiring docs
- [ ] Resolve remaining open items O2, O3, O5, O6 (see decision log)

## Phase 1 — Hardware definition & wiring ⬜
- [ ] Finalize BOM (incl. wiring, connectors, mounting, fuses/e-stop)
- [ ] Power architecture: pack configs, distribution, fusing, common ground
- [x] Wiring & pinout map: Pico↔Cytron MDD10A
- [ ] Wiring & pinout maps: Pico↔encoders, Pico↔ToF, Pico↔Zero 2 W (I²C),
      Zero 2 W↔camera/IMU/Pan-Tilt
- [ ] Bench-test each subsystem in isolation (motors, encoders, ToF, IMU, link)

## Phase 2 — Reflexive layer (Pico 2) 🚧  ← current focus
- [x] I²C register map defined (`protocol/i2c_registers.{md,h}`)
- [x] Project skeleton (Pico SDK, CMake), UART console + LED heartbeat
- [x] I²C peripheral interface (I²C1) — full register space, pointer model, watchdog
- [x] Motor PWM + direction via Cytron MDD10A
- [ ] Quadrature encoder reading via PIO → ticks, distance, velocity
- [ ] Per-wheel closed-loop PID velocity control
- [ ] VL53L0X ToF reading (I²C0 master)
- [ ] Safety reflexes: command-timeout watchdog (done) + collision auto-stop (with ToF)

## Phase 3 — Tactical layer (Raspberry Pi Zero 2 W, thin relay) ⬜
- [ ] PC ↔ Zero TCP protocol definition (`protocol/tcp_messages.*`)
- [ ] I²C master driver + command/telemetry protocol (mirror of Phase 2)
- [ ] Camera pipeline: capture periodic JPEG stills, upload to PC
- [ ] State estimation (wheel odometry + MinIMU-9 dead-reckoning fusion)
- [ ] Pan-Tilt control
- [ ] Command relay: translate tactical intent → wheel-velocity targets for the Pico
- [ ] TCP server + protocol to base station

## Phase 4 — Strategic + Perception (Base station, Windows 11 + NVIDIA GPU) ⬜
- [ ] FastAPI server + TCP client to the Zero
- [ ] GPU perception service: inference on uploaded stills (people/object/scene)
- [ ] Pick inference stack (D30: PyTorch / ONNX Runtime / TensorRT)
- [ ] Web UI: telemetry dashboard (pose, speed, battery, sensors) + image/detection view
- [ ] Manual teleoperation / override
- [ ] Mission / waypoint commanding
- [ ] Data logging & playback

## Phase 5 — Integration & autonomy ⬜
- [ ] End-to-end command + telemetry paths
- [ ] Exploration behavior (frontier exploration; coarse vision-assisted)
- [ ] Field testing, tuning, iteration
- [ ] (Future) ESP-01S / NRF24 backup low-rate radio link
