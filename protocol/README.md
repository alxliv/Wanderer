# Wanderer — Protocol Definitions

Single source of truth for the two communication boundaries. All three codebases
(`pico/`, `tactical/`, `basestation/`) must agree with what is defined here.

## Boundaries

1. **PC ↔ Zero 2 W** — raw TCP message framing (commands + telemetry + periodic stills over Wi-Fi).
   - To be defined in Phase 3/4 (`tcp_messages.*`).
2. **Zero 2 W ↔ Pico** — I²C register map (Zero = master, Pico = peripheral). ✅ defined
   ([`i2c_registers.md`](i2c_registers.md), [`i2c_registers.h`](i2c_registers.h)).

## Design intent

- **Register-map style** for I²C: the master (Zero 2 W) reads/writes byte-addressable
  registers exposed by the Pico (command registers in, telemetry registers out).
- Versioned: include a protocol/firmware version register so mismatches are detectable.
- Endianness and field layout documented explicitly here once drafted.

> Files (e.g. `i2c_registers.md`, `i2c_registers.h`, `tcp_messages.md`) land here as
> each boundary is designed. Where practical, generate language-specific headers from a
> single definition to avoid drift.
