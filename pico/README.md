# Wanderer — Pico 2 firmware (reflexive layer)

C/C++ firmware for the Raspberry Pi Pico 2 (RP2350). Drives the motors, reads
encoders and the ToF sensor, runs the PID velocity loops and safety reflexes,
and exposes the [I²C register interface](../protocol/i2c_registers.md) to the tactical host (Zero 2 W).

## Status — Phase 2
- ✅ I²C peripheral (slave @ `0x42`) exposing the full register space
- ✅ Register-pointer access model with read snapshotting + command-apply-on-STOP
- ✅ 100 Hz control-loop scaffold
- ✅ Command watchdog (stops on host silence) + `CONTROL_FLAGS` handling
- ✅ L298N open-loop motor PWM in `DIRECT_PWM` mode
- ⬜ Quadrature encoders (PIO) → velocity & odometry
- ⬜ Per-wheel PID
- ⬜ VL53L0X ToF + obstacle reflex

## Prerequisites
- **Raspberry Pi Pico SDK ≥ 2.0.0** (required for RP2350 / `pico2`)
- ARM GNU toolchain (`arm-none-eabi-gcc`), CMake ≥ 3.13, Ninja (or Make)
- `PICO_SDK_PATH` set to your SDK checkout (or pass `-DPICO_SDK_FETCH_FROM_GIT=ON`)

## Build

### Generic
```sh
# from pico/
cmake -B build -G Ninja -DPICO_SDK_PATH=/path/to/pico-sdk
cmake --build build
```

### This machine (Windows / PowerShell, run from the repo root)
The ARM toolchain isn't on PATH, and the SDK's picotool source build fails here
(it grabs an incompatible host clang++) — so prepend the toolchain and point at the
installed picotool.

One-time configure:
```powershell
$tc = "$env:USERPROFILE\.pico-sdk\toolchain\14_2_Rel1\bin"
$env:PATH = "$tc;$env:PATH"
cmake -S pico -B pico/build -G Ninja `
  -DPICO_SDK_PATH="$env:PICO_SDK_PATH" `
  -DPICO_TOOLCHAIN_PATH="$tc" `
  -Dpicotool_DIR="$env:USERPROFILE/.pico-sdk/picotool/2.1.1/picotool"
```

Build (repeat after code changes):
```powershell
$tc = "$env:USERPROFILE\.pico-sdk\toolchain\14_2_Rel1\bin"; $env:PATH = "$tc;$env:PATH"
cmake --build pico/build
```

Output: `pico/build/wanderer_pico.uf2`.

## Motor tests

The automated unit test validates signed direction mapping, command saturation,
and the configurable PWM limit without requiring a Pico:

```powershell
cmake -S pico/tests -B pico/tests/build
cmake --build pico/tests/build
ctest --test-dir pico/tests/build -C Debug --output-on-failure
```

`pico/build/wanderer_motor_test.uf2` is a separate physical bring-up image.
Before flashing it, raise and secure the chassis so both wheels turn freely.
After a 5-second delay it runs this one-shot sequence at 30% PWM:

1. Left forward, then reverse
2. Right forward, then reverse
3. Both forward, then reverse

Each movement lasts one second with a one-second stopped interval. The firmware
then leaves both outputs stopped. This test confirms wiring and visible motor
operation; the unit test alone cannot prove that a physical motor turns.

After the hardware test, flash `wanderer_pico.uf2` again for normal operation.
Normal firmware only drives the motors when `MOTOR_ENABLE` is set and
`CONTROL_MODE` is `DIRECT_PWM`; `IDLE`, `VELOCITY` (until PID is implemented),
and watchdog fault states all coast both motors.

## Flash
Hold **BOOTSEL**, plug in the Pico 2, release — it mounts as a USB drive — then
copy `wanderer_pico.uf2` onto it. (Or use `picotool load`.)

## Debug console
`printf` output is on **UART0** (GP0 = TX, GP1 = RX) at 115200 baud — connect a
USB-serial adapter. USB-CDC stdio is disabled so it doesn't interfere with the
BOOTSEL workflow; switch it in `CMakeLists.txt` if preferred.

## Layout
| File | Purpose |
|------|---------|
| `src/main.c` | boot, defaults, control-loop scaffold, watchdog |
| `src/i2c_peripheral.{c,h}` | I²C slave + register image + typed accessors |
| `src/motors.{c,h}` | L298N GPIO/PWM driver |
| `src/motor_output.{c,h}` | testable command clamp and direction mapping |
| `src/config.h` | pin map + tunable defaults (keep in sync with `hardware/wiring.md`) |
| `tests/` | host unit test and one-shot Pico motor hardware test |
| `../protocol/i2c_registers.h` | shared register definitions (source of truth) |
```
