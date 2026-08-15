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

`drive()` returns a `DriveApplied`. A command needing more wheel speed than
the drivetrain has is **not** refused — the airframe scales both wheels by one
factor, which preserves the commanded turn radius and gives up speed instead,
then reports what it did:

```python
applied = cp.drive(0.5, 1.0)          # wants right wheel at 0.65 m/s
if applied.limited:                   # ... but the wheel limit is 0.6
    print(applied.linear_m_s,         # 0.462 — same arc,
          applied.angular_rad_s)      # 0.923 — traversed slower
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

For a ground-load calibration of the open-loop right-wheel gain, use the
cockpit UART tool rather than the raised-wheel USB backdoor calibration:

```sh
# from pilot/ on the Pi; reads gains from the running Pico
python3 -m helm.motor_gain_cal --speed 0.25
```

It performs confirmed, short open-loop runs, measures gyro heading change and
encoder travel, and prints a replacement `MOTOR_RIGHT_GAIN_PERMILLE`; review,
rebuild, and flash that value deliberately.

Before using that calibration, verify the IMU and encoder sign chain with the
read-only cockpit monitor:

```sh
python3 -m helm.imu_monitor --zero
```

## Health service (`health/`)

The Pilot host runs `wanderer-health.service`, a systemd unit that answers one
question without a laptop, an SSH session, or a screen: **is the Pilot board
alive and fit to drive?**

A shell monitor (`wanderer-health.sh`) polls every few seconds and checks the
two things that actually strand the vehicle mid-run:

- **Link** — Wi-Fi radio present and unblocked, associated, holding an IPv4
  address and a default route, signal above the weak/fault dBm thresholds, and
  (optionally) the Base PC answering a ping.
- **Power** — `vcgencmd` undervoltage and throttling flags, both active and
  historical, when that tool is available.

The verdict is reported two ways:

- **A common-cathode RGB LED** driven through the Linux LED subsystem —
  green OK, yellow degraded, red fault, magenta undervoltage/throttling, blue
  starting. LED off while the board's POWER LED is on means Linux or the
  service itself is down. This is the readout that works when nothing else does.
- **`/run/wanderer/health.env`**, rewritten atomically on every check, plus a
  journal entry on each state change — so pilot code and `journalctl` can both
  consume the same state.

The script also runs one-shot (`--once --no-led`) and exits 0 / 1 / 2 for
OK / WARN / FAULT, which is how the mocked test suite exercises it on a
development PC with no Pi hardware.

Setup — the `config.txt` LED overlays, package prerequisites, install commands,
tunables in `/etc/default/wanderer-health`, signal bands, and the test harness —
is documented in **[health/README.md](health/README.md)**.

## Tests

```sh
# from pilot/
python -m unittest discover -s tests -v
```

The tests run the API against the simulator and pin the milestone-one
contract: arm/drive/query, nacks when unarmed, fallback on commander silence
(queries feed liveness but don't resume motion), e-stop latching, stop vs
disarm.
