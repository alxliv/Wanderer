# Protocol specifications

Normative specs for every link in the system. Implementations (C++ in
`firmware/common/`, Python in `base/` and `pilot/`) are hand-written against
these documents — when they disagree, the spec here wins; fix the code.

| Link | Spec | Status |
|---|---|---|
| PC ↔ base dongle (USB CDC, text) | [base_text_protocol.md](base_text_protocol.md) | Imported from RF-Comms; working |
| Base ↔ Wanderer (nRF24, binary) | `firmware/common/protocol.h` is currently self-documenting | To be reshaped: relay framing, transponder, backdoor (arch §3a) |
| Pi5 ↔ Pico2 cockpit protocol (UART) | [cockipt protocol](cockpit_protocol.md) | To be designed against arch §5 (three tiers) |`

The imported RF protocol still has the Base commanding motion directly
(`CMD_MOVE`) — the pre-architecture model. Under the command architecture the
Base flies only through the Pilot; the RF command set will shrink to
transponder + backdoor + relay.
