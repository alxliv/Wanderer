# Wanderer — Protocol Definitions

Single source of truth for the two communication boundaries. All three codebases
(`pico/`, `rpi5/`, `basestation/`) must agree with what is defined here.

## Boundaries

1. **PC ↔ Pi 5** — raw TCP message framing (commands + telemetry over Wi-Fi).
   - To be defined in Phase 3/4.
2. **Pi 5 ↔ Pico** — I²C register map (Pi 5 = master, Pico = peripheral).
   - To be defined in Phase 2.

## Design intent

- **Register-map style** for I²C: the Pi 5 reads/writes byte-addressable registers
  exposed by the Pico (command registers in, telemetry registers out).
- Versioned: include a protocol/firmware version register so mismatches are detectable.
- Endianness and field layout documented explicitly here once drafted.

> Files (e.g. `i2c_registers.md`, `i2c_registers.h`, `tcp_messages.md`) land here as
> each boundary is designed. Where practical, generate language-specific headers from a
> single definition to avoid drift.
