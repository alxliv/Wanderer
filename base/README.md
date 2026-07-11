# Base — ATC station (PC)

Ground-station software. Talks to the base-dongle Pico over USB CDC using the
line-based text protocol ([protocol/base_text_protocol.md](../protocol/base_text_protocol.md));
the dongle translates to the binary nRF24 protocol over the air.

**Stack:** Python CLI/TUI, growing into a package as the command set grows.

## Contents

- `wanderer_client.py` — interactive serial console imported from RF-Comms.
  Sends typed verbs to the dongle, renders sigil-prefixed replies (`=` ack,
  `>` data, `#` telemetry redrawn in place, `!` event, `*` log).

  ```sh
  pip install pyserial
  python wanderer_client.py COM5
  ```
