# Wanderer — Pico firmware

One pico-sdk CMake tree building every Pico target in the system (all Pico 2 /
RP2350 for now):

| Target | Flashes onto | What it is |
|---|---|---|
| `wanderer_airframe` | rover Pico2 | **The flight firmware**: UART cockpit, tactical FSM, motors, encoders (see below) |
| `wanderer_motor_test` | rover Pico2 | One-shot motor/encoder hardware bring-up test (see below) |
| `wanderer_rflink` | either Pico2 | Transitional dual-role RF firmware from RF-Comms: ROLE pin (GP22) high = base dongle, low = vehicle side |

Still planned as the command architecture is implemented: a `wanderer_dongle`
target, which together with `wanderer_airframe` dissolves `rflink`. The old
I2C-cockpit main is parked in `airframe/legacy/` and is not built.

## Layout

| Path | Purpose |
|---|---|
| `common/` | Shared C++: `cockpit_codec.h/.cpp` (cockpit line protocol), `cockpit_handler.h/.cpp` (command dispatch), `tactical.h/.cpp` (tactical FSM), `protocol.h` (RF binary frames) |
| `common/tests/` | Host unit tests for the codec, handler, and FSM |
| `airframe/src/` | Flight firmware `main.cpp` + rover drivers: MDD10A motors, PIO quadrature encoders, pin map (`config.h`) |
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

Outputs: `firmware/build/airframe/wanderer_airframe.uf2`,
`firmware/build/airframe/wanderer_motor_test.uf2`,
`firmware/build/rflink/wanderer_rflink.uf2`.

## Flash

Hold **BOOTSEL**, plug in the Pico 2, release — it mounts as a USB drive — then
copy the `.uf2` onto it. (Or use `picotool load`.)

## Host unit tests

Two independent host test trees, both building without the Pico SDK.

Airframe drivers — MDD10A sign-magnitude direction mapping, PWM limits, and
encoder tick-to-velocity conversion:

```powershell
cmake -S firmware/airframe/tests -B firmware/airframe/tests/build
cmake --build firmware/airframe/tests/build
ctest --test-dir firmware/airframe/tests/build -C Debug --output-on-failure
```

Shared cockpit + tactical code — line codec (checked against the golden wire
vectors in `protocol/cockpit_vectors.txt`), command handler, and FSM:

```powershell
cmake -S firmware/common/tests -B build_common_tests
cmake --build build_common_tests
ctest --test-dir build_common_tests -C Debug --output-on-failure
```

`-C Debug` matches the configuration built by the multi-config Visual Studio
generator used on this machine by default.

## Airframe flight firmware (`wanderer_airframe`)

The rover's tactical layer. Two separate serial paths, which are easy to
confuse:

| Path | Physical port | Carries |
|---|---|---|
| Cockpit | GP0 (TX) / GP1 (RX) — UART0, 3.3 V TTL, 115200 8N1 | Only cockpit protocol lines (`protocol/cockpit_protocol.md`) |
| Bench log | the Pico's own micro-USB | USB CDC `printf` — boot line and debug `*` lines |

`airframe/src/config.h` is the source of truth for pins and baud
(`COCKPIT_*`); `hardware/wiring.md` mirrors it. Motor control is open loop for
now — wheel target mm/s maps linearly to per-mille PWM.

On boot the USB CDC port prints:

```text
*airframe fw 0.3 cockpit on uart0 @115200
```

To bring the link up against the pilot SBC for the first time — three wires
plus the Linux-side serial console configuration, then a hand-typed session
that exercises the FSM and deadman — follow
[`docs/cockpit_bench_test.md`](../docs/cockpit_bench_test.md).

**Wheels off the ground** whenever this firmware is running with motor power
on: `drive` moves real motors.

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
