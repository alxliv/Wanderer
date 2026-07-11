# Wanderer — Pico firmware

One pico-sdk CMake tree building every Pico target in the system (all Pico 2 /
RP2350 for now):

| Target | Flashes onto | What it is |
|---|---|---|
| `wanderer_motor_test` | rover Pico2 | One-shot motor/encoder hardware bring-up test (see below) |
| `wanderer_rflink` | either Pico2 | Transitional dual-role RF firmware from RF-Comms: ROLE pin (GP22) high = base dongle, low = vehicle side |

Planned as the command architecture is implemented: a `wanderer_airframe`
target (UART cockpit, procedures, reflexes) and a `wanderer_dongle` target,
both dissolving `rflink`. The old I2C-cockpit main is parked in
`airframe/legacy/`.

## Layout

| Path | Purpose |
|---|---|
| `common/` | Shared C++ headers: `protocol.h` (RF binary frames), `tactical.h` (TacticalCore FSM) |
| `airframe/src/` | Rover drivers: MDD10A motors, PIO quadrature encoders, pin map (`config.h`) |
| `airframe/tests/` | Host unit tests + the motor hardware test source |
| `airframe/legacy/` | Superseded I2C-cockpit firmware, kept as reference |
| `rflink/` | Imported working RF firmware (`main.cpp`) |
| `lib/RF24/` | nRF24L01 driver — git submodule pinned at v1.6.1 (`git submodule update --init`) |

## Build

### Generic
```sh
# from firmware/
cmake -B build -G Ninja -DPICO_SDK_PATH=/path/to/pico-sdk
cmake --build build
```

### This machine (Windows / PowerShell, run from the repo root)

One-time configure:
```powershell
$tc = "$env:USERPROFILE\.pico-sdk\toolchain\14_2_Rel1\bin"
$env:PATH = "$tc;$env:PATH"
cmake -S firmware -B firmware/build -G Ninja `
  -DPICO_SDK_PATH="$env:PICO_SDK_PATH" `
  -DPICO_TOOLCHAIN_PATH="$tc" `
  -Dpioasm_DIR="$env:USERPROFILE/.pico-sdk/tools/2.2.0/pioasm" `
  -Dpicotool_DIR="$env:USERPROFILE/.pico-sdk/picotool/2.1.1/picotool"
```

Build (repeat after code changes):
```powershell
$tc = "$env:USERPROFILE\.pico-sdk\toolchain\14_2_Rel1\bin"; $env:PATH = "$tc;$env:PATH"
cmake --build firmware/build
```

Outputs: `firmware/build/airframe/wanderer_motor_test.uf2`,
`firmware/build/rflink/wanderer_rflink.uf2`.

## Flash

Hold **BOOTSEL**, plug in the Pico 2, release — it mounts as a USB drive — then
copy the `.uf2` onto it. (Or use `picotool load`.)

## Host unit tests

Validate the MDD10A sign-magnitude direction mapping, PWM limits, and encoder
tick-to-velocity conversion without a Pico:

```powershell
cmake -S firmware/airframe/tests -B firmware/airframe/tests/build
cmake --build firmware/airframe/tests/build
ctest --test-dir firmware/airframe/tests/build -C Debug --output-on-failure
```

`-C Debug` matches the configuration built by the multi-config Visual Studio
generator used on this machine by default.

## Motor hardware test (`wanderer_motor_test`)

A standalone, manually armed physical bring-up test. **Raise and secure the
chassis so both wheels turn freely before flashing.**

After flashing, open the Pico's USB CDC serial port (no UART adapter needed —
the firmware waits for the port to open), enable motor power, and type `S` to
arm. A final 5-second delay allows an abort by cutting motor power. The MDD10A
provides no power-good signal, so typing `S` *is* the operator's confirmation
that motor power is on.

Each `S` resets encoder counts and runs once at 40% PWM:

1. Left forward, then reverse
2. Right forward, then reverse
3. Both forward, then reverse

Each movement lasts one second with a one-second pause. Serial output prints
cumulative left/right ticks and per-100 ms deltas: the moving wheel should
change while the stopped wheel stays near zero; forward should be positive. If
one wheel's signs are reversed, flip its `ENC_*_SIGN` in `airframe/src/config.h`
(keep `hardware/wiring.md` in sync).
