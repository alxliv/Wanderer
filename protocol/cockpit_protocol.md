# Wanderer Cockpit — Pi5 ↔ Pico2 UART Protocol

The tactical interface: how the **Pilot** (Pi5) flies the **airframe** (Pico2)
over the local UART. This is the only flight interface in the system
(architecture §2, §6). It implements the cockpit contract of architecture §5:
a dependable request/response channel — every request answered at once — plus
asynchronous events for what the airframe must report on its own initiative.

```
  Pi5 (Pilot)  ──────────── UART, text lines ────────────  Pico2 (airframe)
  pilot/cockpit/                                           TacticalCore FSM,
  UART CockpitLink                                         motors, reflexes
```

The same UART also carries the Pico2's *radio-modem* hat: Base↔Pilot relay
traffic (§4 below). Cockpit and relay share the wire but never mix — one prefix
character splits them, and relay payloads are opaque to both this spec and the
Pico2.

This document is normative for the wire. The Python semantics reference is
`pilot/cockpit/` (API + simulator); the firmware reference is
`firmware/common/tactical.h`. Where they disagree with this spec, the spec
wins — fix the code (see §11 for the known deltas).

---

## 1. Design principles

- **Same text discipline as the base protocol.** One message per `\n`-terminated
  line, 7-bit ASCII, resynchronizable by construction, unknown-skip rules.
  One line-codec implementation serves both links.
- **One request, one reply.** Unlike the Base protocol — where `=` is only a
  syntactic ack and the real answer arrives an RF hop later as `>` — the cockpit
  responder *is* the executor. So the cockpit has no `>` class at all: every
  request is answered by exactly one `=` line that is **semantic** —
  accepted-and-applied, or refused-with-reason. Query results ride as fields on
  the `=ok` line.
- **Nothing streams through the cockpit** (arch §5). There is deliberately no
  `#` telemetry class on this link. The Pilot pulls state and odometry by query
  at its own cadence; the high-rate broadcast is the transponder's job (arch
  §3a), not the cockpit's.
- **The vocabulary is the tiers.** Tier 1 (control surface) plus housekeeping is
  specified now — exactly the op set of `pilot/cockpit/link.py`. Tier 2
  (procedures) and Tier 3 (reflex configuration) have their syntax *reserved*
  (§10) but not designed.
- **SI on the wire.** Meters per second, radians per second, plain decimal
  numbers. The wire is machine↔machine; there is no int16/mm-per-s legacy to
  carry over from the RF protocol.

---

## 2. Framing

- A **message is one line**, terminated by `\n`. A leading `\r` is tolerated.
- Encoding is 7-bit ASCII. Maximum line length **120 bytes** including the
  terminator; over-length input is discarded to the next newline and answered
  `=err ? line_too_long`.
- **Pilot → airframe** lines are **bare requests**: `verb [arg ...]` — or relay
  lines prefixed `^`.
- **Airframe → Pilot** lines are **sigil-prefixed**:

| Sigil | Class   | Meaning                                         | Solicited?          |
|:-----:|---------|-------------------------------------------------|---------------------|
| `=`   | reply   | The one answer to your request (`=ok` / `=err`) | reply, exactly one  |
| `!`   | event   | Airframe-initiated: state change, fault, (later: procedure outcome, reflex fire) | async |
| `*`   | log     | Free-form human text; machine clients ignore it | async               |
| `^`   | relay   | Radio-modem hat: payload from the Base, forwarded verbatim | async    |

**Parser robustness (both directions):** unknown sigil → skip the line; unknown
`key=` → ignore that field, keep the rest; unparseable line → skip to the next
newline; blank lines ignored. On airframe input, `*`-prefixed lines are also
ignored (comments in scripted input).

**Interleaving:** `!`, `*`, and `^` lines may appear at any time, including
between a request and its `=` reply. The reply to a request is the **next `=`
line**, whatever arrives around it. There is **no ordering guarantee** between
a reply and the events the same request caused: `arm` may produce
`=ok arm` then `!state ...`, or the reverse. Pilot code must treat events as an
independent channel (the existing `Cockpit` API already does).

---

## 3. Requests and replies

### Request format

`verb [arg ...]` — verbs are case-insensitive, arguments positional,
whitespace-separated. Numbers are plain decimal (`0.25`, `-0.1`, `3`); no
exponents required of the parser.

### Reply format

- `=ok <verb> [key=value ...]` — executed; query results as fields.
- `=err <verb> <reason> [detail ...]` — refused. `<reason>` is one
  machine-readable token from the registry (§7); anything after it is
  human-readable detail. If the verb itself could not be parsed, it is echoed
  as `?`.

The verb echo is a desync guard: with one request in flight (the API layer
guarantees this), a mismatched echo means a stale or lost line — discard and
retry. `=ok` / `=err` map onto the `CockpitLink` contract as return /
`CockpitNack(code=reason, message=detail)`; no reply within the timeout is
`CockpitTimeout`.

For example (bare lines are the Pilot; `=` lines are the airframe):

```
ping
=ok ping
get_state
=ok get_state state=ACTIVE
drive 0.25 0.0
=ok drive
drive 0.25 x
=err drive bad_args expected 2 numbers
mve 1 2
=err ? unknown_command mve
```

### Tier 1 + housekeeping vocabulary

One wire verb per `OP_*` constant in `pilot/cockpit/link.py`, same spelling:

| Verb          | Args                          | `=ok` fields                       | Possible `=err` reasons |
|---------------|-------------------------------|------------------------------------|-------------------------|
| `ping`        | —                             | —                                  | —                       |
| `arm`         | —                             | —                                  | `fault_latched`         |
| `disarm`      | —                             | —                                  | `fault_latched`         |
| `estop`       | —                             | —                                  | — (never refused)       |
| `clear_fault` | —                             | —                                  | `no_fault`, `fault_persists` |
| `drive`       | `<linear_m_s> <angular_rad_s>`| —                                  | `not_armed`, `bad_args` |
| `stop`        | —                             | —                                  | `not_armed`             |
| `get_state`   | —                             | `state=<NAME>` `fault=<NAME>` (only in FAULT) | —            |
| `get_odometry`| —                             | `lt=<int> rt=<int> vl=<m/s> vr=<m/s>` | —                    |
| `get_version` | —                             | `fw=<major>.<minor>`               | —                       |

Field conventions follow the Base protocol: FSM states and fault codes by
**name** (`SAFE`, `ACTIVE`, `FALLBACK`, `FAULT`; `ESTOP`, ...), booleans `0`/`1`,
new fields may be appended at any time without a protocol revision.

---

## 4. The relay lines (`^`) — the modem hat

The Pico2 wears two hats (arch §2) and this prefix is the entire boundary
between them on the UART:

- `^<payload>` **Pilot → airframe**: forward `<payload>` verbatim to the Base
  over RF.
- `^<payload>` **airframe → Pilot**: `<payload>` arrived verbatim from the Base
  over RF.

The payload is **opaque**: the Pico2 never parses it, this spec never defines
it. Its content is the ATC↔Pilot protocol (a future spec in `protocol/`). The
payload must itself be newline-free text fitting the line-length bound; how it
is fragmented across ≤32-byte RF frames is the relay framing's concern
(`protocol.h` reshape, arch §3a), invisible on the UART.

Relay lines are not requests: they get no `=` reply and do **not** refresh the
motion lease (§5) — a chatty Base must not mask a dead Pilot.

---

## 5. Liveness and the deadman

The motion lease of architecture §6, on this wire:

- **Every well-formed cockpit request refreshes the lease** — any known verb,
  even one answered `=err`. Malformed lines and `^` relay lines do not.
- If the airframe is `ACTIVE` and the lease lapses, it enters `FALLBACK`
  (bounded decel ramp to zero) and reports `!state from=ACTIVE to=FALLBACK`.
  Only a fresh `drive` resumes motion — the Pilot must re-assert intent, never
  silently re-accelerate.
- The lease window is a firmware constant, currently **750 ms**
  (`LIVENESS_TIMEOUT_MS`, `tactical.h`). The Pilot must request at a period
  well inside it: the drive stream does this naturally during motion; call
  `ping` when idle. Recommended worst-case request interval: **≤ 250 ms**.
  This is deliberately not automatic in the API — a hung Pilot must look hung.

`estop` is honored from any state at any time, latches `FAULT` with code
`ESTOP`, and is independent of (and senior to) the lease. The backdoor's ESTOP
(arch §3a) is a second, parallel path to the same latch; the cockpit one is not
special.

---

## 6. State semantics on the wire

The FSM is `TacticalCore` (`tactical.h`); the sim (`pilot/cockpit/sim.py`) is
its executable mirror. Normative outcomes per state:

| Request ↓ / State → | SAFE            | ACTIVE          | FALLBACK        | FAULT              |
|---------------------|-----------------|-----------------|-----------------|--------------------|
| `arm`               | → ACTIVE, ok    | ok (no-op)      | ok (no-op)¹     | `=err fault_latched` |
| `disarm`            | ok (no-op)      | → SAFE, ok      | → SAFE, ok      | `=err fault_latched` |
| `drive`             | `=err not_armed`| apply, ok       | → ACTIVE, apply, ok | `=err not_armed` |
| `stop`              | `=err not_armed`| zero velocity, ok | `=err not_armed`² | `=err not_armed` |
| `estop`             | → FAULT, ok     | → FAULT, ok     | → FAULT, ok     | ok (already latched) |
| `clear_fault`       | `=err no_fault` | `=err no_fault` | `=err no_fault` | → SAFE, ok / `=err fault_persists` |
| queries             | ok              | ok              | ok              | ok                 |

¹ `arm` never exits FALLBACK — that is `drive`'s exclusive privilege.
² In FALLBACK the airframe is already stopping itself; `stop` is refused rather
than silently converted, so the Pilot learns the true state. `disarm` is the
correct "stand down" from FALLBACK.

`clear_fault` succeeds only if the fault *condition* is gone (`fault_persists`
otherwise). For `ESTOP` the condition is definitionally gone; the reason exists
for hardware faults (overcurrent, tilt) that may still be present.

---

## 7. Error reason registry

| Reason           | Meaning                                            |
|------------------|----------------------------------------------------|
| `unknown_command`| Verb not recognized                                |
| `bad_args`       | Wrong count or unparseable argument                |
| `line_too_long`  | Input exceeded the line bound (§2)                 |
| `not_armed`      | Motion request outside ACTIVE (see §6 for FALLBACK)|
| `fault_latched`  | Refused while FAULT is latched                     |
| `no_fault`       | `clear_fault` with nothing latched                 |
| `fault_persists` | Fault condition still present; latch kept          |
| `busy`           | *Reserved* for Tier 2 procedure arbitration        |

New reasons may be added; a client treats an unknown reason as a generic NACK.

---

## 8. Events

| Event        | Form                            | Meaning                                  |
|--------------|---------------------------------|------------------------------------------|
| state change | `!state from=<NAME> to=<NAME>`  | Every FSM transition, whatever caused it (command, deadman, reflex, backdoor) |
| fault        | `!fault code=<NAME>`            | A fault latched; accompanies the transition to FAULT |

Fault code registry (grows with Tier 3): `ESTOP`. Wire names map to the
integer codes of `events.FaultRaised` in the link implementation.

Events fire for *every* cause, including transitions the Pilot itself
commanded — the event stream is a complete, self-sufficient record of the FSM.

---

## 9. Examples — request / event / reply chains

Bare lines are the Pilot; sigil lines are the airframe. Comments in
parentheses are not on the wire.

### Startup handshake and first motion

```
ping
=ok ping
get_version
=ok get_version fw=0.1
get_state
=ok get_state state=SAFE
arm
=ok arm
!state from=SAFE to=ACTIVE          (may also arrive before =ok arm — §2)
drive 0.30 0.0
=ok drive
drive 0.30 0.0                       (the stream doubles as the heartbeat)
=ok drive
drive 0.20 0.50                      (arc left)
=ok drive
stop
=ok stop                             (velocity zero, still ACTIVE)
disarm
=ok disarm
!state from=ACTIVE to=SAFE
```

### Refusal: motion before arming

```
get_state
=ok get_state state=SAFE
drive 0.30 0.0
=err drive not_armed state is SAFE
```

The Pilot's `Cockpit.drive()` raises `CockpitNack("not_armed", ...)`; nothing
moved and nothing changed state — no event fires.

### Deadman: the Pilot goes quiet, then resumes

```
drive 0.30 0.0
=ok drive
                                     (Pilot hangs; > 750 ms of silence)
!state from=ACTIVE to=FALLBACK       (airframe ramps itself to zero)
                                     (Pilot recovers)
stop
=err stop not_armed state is FALLBACK   (refusal tells the truth — §6²)
drive 0.30 0.0                       (fresh drive is the only resume)
=ok drive
!state from=FALLBACK to=ACTIVE
```

### E-stop, latch, and clearing

```
estop
=ok estop
!fault code=ESTOP
!state from=ACTIVE to=FAULT
arm
=err arm fault_latched
clear_fault
=ok clear_fault
!state from=FAULT to=SAFE
arm
=ok arm
!state from=SAFE to=ACTIVE
```

### Interleaving: relay and log traffic during a query

```
get_odometry
^GOAL waypoint 12.5 -3.0             (Base traffic arriving mid-request: opaque,
*battery check due                    not the reply, no lease refresh)
=ok get_odometry lt=15320 rt=15294 vl=0.298 vr=0.301
```

The reply is the next `=` line regardless of what interleaves. The `^` payload
here is illustrative only — its actual content belongs to the future ATC↔Pilot
spec, not this one.

---

## 10. Reserved — Tiers 2 and 3, debug surface

Syntax pre-reserved so today's parsers skip it cleanly; semantics deliberately
not designed yet (arch §5, §7):

- **Tier 2 procedures:** `proc <name> [arg ...]` to engage, `abort` to abort.
  Outcome event: `!proc name=<n> outcome=DONE|ABORTED|SUPERSEDED [reason=<r>]`.
  Arbitration between a procedure and the Tier 1 stream will define `busy`.
- **Tier 3 reflexes:** `cfg <reflex> [key=value ...]` to enable/threshold;
  fire event `!reflex name=<n> [action=<a>]`.
- **Debug/raw surface:** `raw ...` — the open-loop bench surface, gated behind
  a debug mode, refused otherwise. Shape TBD.
- **Client tag echo:** trailing `@<tag>` echoed on the reply, exactly as
  reserved in the Base protocol §10 — only if a client ever wants pipelined
  requests. The baseline (one in flight) does not need it.

---

## 11. Known deltas — code this spec obsoletes

Recorded per `protocol/README.md`'s rule (spec wins, fix the code):

1. **`tactical.h` conflates stop and disarm.** `cmd_stop` today disarms to
   SAFE (the RF-era `CMD_STOP`). The cockpit splits this into `stop`
   (zero velocity, stay ACTIVE) and `disarm` (→ SAFE). `TacticalCore` needs
   the two entry points.
2. **`sim.py` silently no-ops `arm` and `disarm` in FAULT.** The spec refuses
   both with `fault_latched` (§6): a Pilot must never believe it armed a
   faulted airframe. Fix the sim (and mirror in firmware).
3. **`tactical.h` `on_transition()` is an empty seam.** It must emit `!state`
   (and `!fault`) on the UART per §8.
4. **`sim.py` `clear_fault`** always succeeds; firmware carries the
   `condition_cleared` check. The sim should model `fault_persists` once a
   simulated persistent fault exists (Tier 3 work).

---

## 12. Link parameters

- UART, **115200 baud, 8N1, no flow control** as the baseline. Budget check:
  a 50 Hz drive stream plus replies is ≈ 2 KB/s against ≈ 11 KB/s capacity;
  headroom for relay traffic remains. The rate may be raised by agreement of
  both ends without touching this spec; the electrical choice of UART (arch:
  isolation) is invariant.
- On open, both ends discard input to the first newline (a partial line may be
  in flight). Recommended Pilot handshake: `ping`, then `get_version`.

---

## 13. Quick reference

```
Pilot -> airframe (bare verbs):
  ping | arm | disarm | estop | clear_fault
  drive <linear_m_s> <angular_rad_s> | stop
  get_state | get_odometry | get_version
  ^<payload>                      (relay to Base, opaque)

airframe -> Pilot (sigil + payload):
  =  reply   =ok <verb> [k=v ...]      =err <verb> <reason> [detail]
  !  event   !state from=.. to=..      !fault code=..
  *  log     *free text (machines ignore)
  ^  relay   ^<payload> (from Base, opaque)

States: SAFE | ACTIVE | FALLBACK | FAULT     Faults: ESTOP
Deadman: any valid request refreshes; 750 ms window; drive resumes FALLBACK.
No `#` on this link: nothing streams through the cockpit.
```