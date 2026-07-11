# Pilot — strategic layer (Raspberry Pi 5)

Owns all *judgment*: the plan, the world model, position-in-house, emergency
planning. Reaches the airframe through the UART cockpit; is reached by the Base
over nRF24 relay (field) or WiFi/SSH (development). See
[docs/Wanderer_Command_Architecture.md](../docs/Wanderer_Command_Architecture.md).

**Stack:** Python package (planned), with C++ modules or native daemons later
only where profiling demands it. Nothing here yet — the cockpit protocol
(`protocol/`) comes first.
