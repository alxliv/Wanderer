# Wanderer

An autonomous mobile research platform. The target architecture distributes control across
three parties — the guiding analogy: **Base is Air Traffic Control, the Pi5 is
the Pilot, the Pico2 is the aircraft itself.** The full design is in
[docs/Wanderer_Command_Architecture.md](docs/Wanderer_Command_Architecture.md)
— the project's source of truth.

| Party | Embodiment | Owns | Code |
|---|---|---|---|
| Base (ATC) | PC + Pico "dongle" (USB↔nRF24) | Goals, supervision, high-level direction | `base/`, `firmware/` (dongle side) |
| Pilot (strategic) | Raspberry Pi 5 | Judgment: plan, world model, localization | `pilot/` |
| Airframe (tactical) | Pico 2 (RP2350) on the rover | Execution: real-time loops, procedures, reflexes | `firmware/airframe/` |

These are architectural responsibilities, not all of which are implemented
yet. Currently, the Pilot provides a tested cockpit API backed by a simulator;
the real UART link and strategic autonomy are still to come. The airframe has
motor and encoder drivers plus a hardware bring-up target, but not yet the full
UART cockpit, procedures, or reflexes. The imported `rflink` target remains the
working transitional RF implementation.

## Repository layout

```
docs/       Architecture documents and photos (start here)
hardware/   BOM, wiring, pinouts, power budget
protocol/   Protocol status plus the normative Base↔dongle text specification
firmware/   All Pico firmware — one pico-sdk CMake tree, multiple targets
  common/     Shared C++: RF binary protocol frames, TacticalCore FSM
  airframe/   Rover Pico2: motor/encoder drivers and bring-up test; legacy/ = old I2C main
  rflink/     Transitional dual-role RF firmware imported from RF-Comms (proven)
  lib/RF24/   nRF24L01 driver (git submodule, pinned)
pilot/      Python cockpit API and simulator; real Pi5 strategic layer is planned
base/       Base-station PC software — interactive Python serial CLI
tools/      Placeholder for future development utilities (currently empty)
```

After cloning, fetch the RF24 submodule:

```sh
git submodule update --init
```

## Provenance

The working RF link and binary protocol were developed in the separate
[RF-Comms](https://github.com/alxliv/RF-Comms) repo (`Pico2_V2_RF`) and imported
here: `firmware/common/protocol.h`, `firmware/common/tactical.h`,
`firmware/rflink/main.cpp`, `protocol/base_text_protocol.md`, and
`base/wanderer_client.py`. RF-Comms remains the standalone radio testbed; this
repo is where the code evolves to fit the command architecture.
