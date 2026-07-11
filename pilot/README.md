# Pilot — strategic layer (Raspberry Pi 5)

Owns all *judgment*: the plan, the world model, position-in-house, emergency
planning. Reaches the airframe through the UART cockpit; is reached by the Base
over nRF24 relay (field) or WiFi/SSH (development). See
[docs/Wanderer_Command_Architecture.md](../docs/Wanderer_Command_Architecture.md).

**Stack:** Python, with C++ modules or native daemons later only where
profiling demands it.

## Cockpit API (`cockpit/`)

The Pilot's interface to the airframe. **The API is the contract; the wire
protocol is an implementation detail** behind `CockpitLink` — pilot code never
sees the UART.

```python
from cockpit import Cockpit, StateChanged, TacticalState
from cockpit.sim import SimulatedCockpitLink   # real UART link comes later

with Cockpit(SimulatedCockpitLink()) as cp:
    cp.on_event(lambda e: print("airframe says:", e))
    cp.arm()
    cp.drive(0.3, 0.0)        # m/s, rad/s — standing order until superseded
    odo = cp.odometry()       # the Pilot PULLS; nothing streams
    cp.disarm()
```

Design rules (from the command architecture + the cockpit discussion):

- **Blocking request/response.** Every command is answered at once or raises
  (`CockpitTimeout` = channel trouble, `CockpitNack` = airframe refused).
  Single command in flight; no pipelining.
- **Events via callbacks.** Airframe-initiated news (`StateChanged`,
  `FaultRaised`; later reflex fires and procedure completions) arrives on a
  dispatcher thread through `on_event()`.
- **No telemetry through the cockpit API.** The 100 Hz broadcast goes to the
  Base via the transponder. The Pilot queries `state()` / `odometry()` at its
  own cadence.
- **The deadman is honest.** Liveness = any valid command; while driving, the
  velocity stream is the heartbeat, and `ping()` covers quiet phases. There is
  deliberately no hidden auto-heartbeat thread: if the Pilot hangs, the
  airframe must fall back ("zombie" mode) and ramp to a stop — and only a
  fresh `drive()` resumes motion.
- Semantics mirror the proven `TacticalCore` FSM
  (`firmware/common/tactical.h`): SAFE / ACTIVE / FALLBACK / FAULT.

Layout: `api.py` (Cockpit class), `link.py` (transport abstraction + op
vocabulary), `events.py`, `errors.py`, `sim.py` (simulated airframe — an
executable statement of what the firmware must do).

## Tests

```sh
# from pilot/
python -m unittest discover -s tests -v
```

The tests run the API against the simulator and pin the milestone-one
contract: arm/drive/query, nacks when unarmed, fallback on commander silence
(queries feed liveness but don't resume motion), e-stop latching, stop vs
disarm.
