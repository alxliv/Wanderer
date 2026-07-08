# Wanderer — Command Architecture & Layer Responsibilities

**Status:** Architectural foundation. No concrete commands or syntax yet.
This document defines *who owns what* and *why*, so that the concrete tactical
command vocabulary can be designed against a settled set of principles.

---

## 1. The Three Parties

Wanderer is an autonomous entity, not a teleoperated machine. Control is
distributed across three parties, each with a distinct and non-overlapping role.

| Party | Role in the metaphor | Physical embodiment | Owns |
|---|---|---|---|
| **Base (PC)** | Air Traffic Control | Remote station | Goals, supervision, high-level direction. Never controls machinery directly. |
| **Strategic layer** | The Pilot | Pi5 (Linux SBC) | All *judgment*: the plan, the world model, position-in-house, emergency planning. |
| **Tactical layer** | The Airframe + Servos | Pico2 (RP2350) | All *execution*: real-time control loops, self-terminating procedures, reflexes. |

The guiding analogy throughout: **Base is ATC, Pi5 is the Pilot, Pico2 is the
aircraft itself** — its control surfaces, servos, instruments, and protective
systems.

---

## 2. The Communication Topology

```
                 ATC / Goals                    Cockpit / Execution
   ┌────────┐   (nRF24 or WiFi)   ┌────────┐    (UART, local)    ┌────────┐
   │  BASE  │ <─────────────────> │  PI5   │ <─────────────────> │ PICO2  │
   │  (ATC) │                     │(Pilot) │                     │(Airframe)│
   └────────┘                     └────────┘                     └────────┘
      ▲  ▲                                                          │   │
      │  │        nRF24 relay (Pico2 forwards verbatim)            │   │
      │  └──────────────────────────────────────────────────────────┘   │
      │                                                                  │
      │       SYSTEM BACKDOOR + TRANSPONDER (nRF24, direct, privileged)  │
      └──────────────────────────────────────────────────────────────────┘
```

**Primary rule: the Base controls Wanderer only through the Pilot (Pi5).** All
normal operation — every goal, every motion — flies through the cockpit. The Base
is never a flight interface to the tactical layer.

**Two narrow exceptions** route directly Base↔Pico2 and are *not* normal control
paths (see §3a):

- **Transponder (telemetry):** The airframe *broadcasts* its state to ATC for
  supervision, logging, test, and debug. This is broadcast, not a command channel
  — ATC watching the squawk does not make ATC the pilot.
- **System backdoor (privileged, out-of-band):** A maintenance hatch for
  emergency overrides, link/system config, and dev-time diagnostics. Explicitly
  *not* a control path.

The Base reaches the Pilot by one of two transports:

- **nRF24 relay (field / long-range):** The Pico2 acts purely as a *radio modem*.
  Base↔Pi5 traffic is forwarded verbatim and never interpreted.
- **WiFi / SSH (near / development):** The Pi5 is reached directly; the Pico2 is
  not in the comms path at all and its radio-modem function is simply unused.

The cockpit interface (Pi5 ↔ Pico2) is **invariant** to which transport ATC uses
to reach the Pilot. Same pilot, same cockpit, two ways for ATC to reach the cabin.

### The Pico2 wears two separate hats

These must be kept strictly distinct — conflating them invites bugs:

1. **Cockpit / tactical executor:** Pi5→Pico2 frames that the Pico2 *executes*.
2. **Radio modem:** Base↔Pi5 frames that the Pico2 *forwards verbatim and never
   interprets*.

Every frame arriving from the Pi5 over UART carries an implicit "for me / relay
to base" distinction. Frames from the Base over RF are relay-to-Pi5 by default,
*except* the narrow set of transponder/backdoor frames the Pico2 handles itself
(§3a).

---

## 3. The Dividing Line (the heart of the design)

The boundary between the Pilot and the airframe is **not complexity.**

> Complexity is a trap. If you sort tasks by how hard they are, you can always
> push one more "capable-enough" task downward — until the tactical layer is
> quietly holding a second copy of the world. (An autoland in zero visibility is
> enormously complex, yet it is still the servo system's job, not a separate
> brain's.)

The real line is **judgment vs. execution**, tested by one question:

> ### Does the task need to know *where it is in the world*, or *what is around it*?
>
> - **Yes** → it needs the map / semantic perception / an open choice → **Pilot (Pi5)**
> - **No** → it only holds a number true against the body's own senses → **Servo (Pico2)**

### Worked examples

| Task | Needs world model? | Owner | Why |
|---|---|---|---|
| "Go to the kitchen" | Yes — needs the map | **Pi5** | Requires knowing position in house |
| "Get around that chair" | Yes — needs perception | **Pi5** | Requires sensing surroundings |
| "Search for my slippers" | Yes — semantic + map | **Pi5** | Open "what do I do now" choice |
| Decide *to* turn / *to* fly an approach | Yes — it's a decision | **Pi5** | Judgment, standing decisions, modes |
| Turn 90° in place | No | **Pico2** | Relative geometry, proprioceptive |
| Advance 1.2 m and stop | No | **Pico2** | error = target − odometry, drive to zero |
| Hold a relative heading | No | **Pico2** | IMU/encoders only, no map |
| Back up and realign after a stall | No | **Pico2** | A stall is proprioceptive, not strategic |
| Hold a commanded wheel velocity | No | **Pico2** | Hard-real-time control loop |

### Why "autopilot" is the wrong word for the tactical layer

The word "autopilot" hides two different things:

- **The autopilot that *decides*** — intercept the localizer, sequence the
  approach, choose when to descend. That is *judgment*. It is a "thin pilot."
  **It lives on the Pi5**, because a mode is a standing decision, and decisions
  belong to the Pilot.
- **The servo loop that *holds*** — whatever moves the control surfaces hundreds
  of times a second to *make* "320 knots" true. It has no judgment. Error in,
  actuation out. **It lives on the Pico2.**

A maneuver like "advance 1.2 m" is the *second* thing. The Pi5 already decided
*to* advance and *where* 1.2 m points in the world; the Pico2 merely runs the
servo that makes the number true. It is muscle the brain commands — **not** a
brain of its own.

### Why execution *must* be on the Pico2 (not a preference)

The Pi5 runs Linux: non-real-time, subject to scheduling jitter. To close a
metric maneuver up there, it would have to read encoder ticks across the UART,
compute, and stream velocity back every cycle for the whole maneuver. A single
30 ms scheduler hiccup mid-maneuver leaves the wheels running on a stale command
with nothing minding the encoders. Putting a hard-real-time loop on a
soft-real-time brain is exactly the failure the tactical layer exists to prevent.

---

## 3a. The System Backdoor (and Transponder)

The Base has two direct paths to the Pico2 over nRF24. Neither is a control path,
and naming the command path a *backdoor* — rather than a channel — is deliberate:
it sets the access policy. A backdoor is out-of-band, privileged, and not a normal
operating route. **Nobody flies through the backdoor.**

### Transponder — telemetry broadcast (ATC squawk)

The airframe radiates its own state to the Base: position/odometry, voltages,
currents, FSM state, active-procedure status, reflex flags, faults. This is for
supervision, logging, testing, and debug. It is **broadcast, not a command
channel** — ATC receiving the squawk does not become the pilot. This sits
entirely inside the ATC metaphor and threatens none of the layering.

### System backdoor — privileged out-of-band commands (maintenance hatch)

A narrow, privileged path for operations that are deliberately *not* part of
normal flight. Membership is **closed and explicitly justified**: every operation
admitted to the backdoor must be one of exactly three categories.

| Category | Examples | Why it's allowed on the backdoor |
|---|---|---|
| **Emergency override** | ESTOP; safe/disarm | A hardware-level kill must be reachable from the Base regardless of who (or whether anyone) is flying. |
| **Link / system config** | Set PA level, channel, data rate; **reboot the Pi5** | Configures the machine and the link, not the mission. Reboot-the-Pilot is the defining case: if the Pi5 hangs, ATC reaches *through* the always-on airframe to power-cycle the cockpit. |
| **Dev-time diagnostic** | Raw bench wiggle of a motor; read a register | Lets functions be tested over RF *without* an active Pi5 — gated to a Pi5-absent / maintenance mode so it can never fight the Pilot. |

### Why this does not violate the dividing line (§3)

Every backdoor operation lives *below* the judgment line: it commands **the
machine and the link, not the mission.** None needs the map, none needs the world
model. The instant a proposed backdoor command would need to know where it is in
the world or what is around it, it is out of bounds — by exactly the same rule
that governs everything else. Ground crew can safe the aircraft, configure the
radio, and power-cycle the cockpit; they do not fly the plane.

### The one discipline the backdoor demands

A backdoor rots in a predictable way: an estop today, a config tomorrow, a
bench-wiggle for convenience, and in six months it is an undocumented second
flight interface fighting the Pi5. The defense is the closed-membership rule
above — **nothing reaches the backdoor merely because routing there was
convenient.** Normal motion flies through the cockpit (Pi5 / UART), full stop.

### Authority interlock

- The **Pi5 (UART) is the sole *motion* commander.** Only it streams the velocity
  surface and engages maneuvers.
- The backdoor is **not a motion-streaming commander.** Its motion-capable
  operation (raw bench wiggle) is gated to a **Pi5-absent / dev mode**, so the
  Base cannot wiggle the wheels while the Pilot believes it is flying.
- **ESTOP is the deliberate exception — honored from any source, always,**
  regardless of authority or mode. A hardware kill should never be gated.

---

## 4. State Ownership — keep the brains from splitting

A core hazard, and a restatement of a previously confirmed bug (conflicting dual
state sources) at *system* scale:

- **Pico2 owns EXECUTION state:** Am I mid-maneuver? What is my odometry? Did the
  procedure finish, abort, or get superseded?
- **Pi5 owns INTENT state:** The plan. The world pose. What comes next. Where we
  are ultimately headed.

> **Litmus test for any proposed Pico2 feature:** If you cannot build it without
> the Pico2 knowing *the plan it serves* (what comes next, where it's ultimately
> headed), then it is **not** a Pico2 feature.

The Pico2 may know it is "advancing 1.2 m." It must **never** know that this
advance is "step 3 of the route to the kitchen."

---

## 5. The Cockpit, in Three Tiers

The tactical interface (Pi5 ↔ Pico2) is structured as three kinds of interface
over the same `TacticalCore` FSM. They layer in incrementally without changing
the contract.

### Tier 1 — Control Surface ("the stick")

Streamed, idempotent, watchdog-leased. The Pilot's continuous hands-on path.

- A **body-velocity** surface (linear + angular). The Pico2 closes the
  wheel-velocity loop on the Hall encoders; the differential mix is internal.
  Pure angular = turn in place. (Skid steer — no strafe.)
- A raw open-loop surface for **bench / diagnostic** use only, gated behind a
  debug mode.

*This is the honest reading of "the pilot flies the plane": the Pilot is a
continuous controller, hands always on the yoke.*

### Tier 2 — Managed Procedures (servos the Pilot engages and forgets)

Each is a **closed procedure**: defined entry conditions, a control law needing
only the body's own senses plus *relative* geometry, explicit termination and
abort, and a reported outcome. The Pilot *engages and forgets*; the procedure
self-terminates and reports `DONE` / `ABORTED(reason)` / `SUPERSEDED` as an
asynchronous event up the supervisory channel.

Examples of this class:

- Advance a relative distance and stop.
- Turn a relative angle in place.
- Follow an arc of given relative geometry.
- Hold a relative heading while moving.
- Recovery procedures (back up, un-stall) — appropriate here *because* a stall is
  proprioceptive, not strategic.

This is where the "capable Pico2" lives — arbitrarily clever *inside the box*,
but never authoring the plan.

### Tier 3 — Reflexes ("the protections")

Not commanded — *configured* (enable / threshold) — and they *report when they
fire*. Always-on, autonomous, beneath every other tier. This is what makes
motion safe even with no one in the loop.

Examples:

- Obstacle / bump → decelerate or stop.
- Tilt / rollover → cut.
- Overcurrent / stall → cut and flag.
- Comms-loss deadman → FALLBACK.

Plus existing housekeeping: arm / disarm / e-stop / stop / brake; info / fault /
parameter queries; and the free ~100 Hz telemetry stream — which now also carries
**active-procedure status** and **reflex flags** alongside pose, currents,
voltage, and FSM state, so the Pilot always sees what the airframe is doing.

---

## 6. The Deadman — one *motion* lease, one link

Committing to "the Base controls Wanderer only through the Pilot" means the
tactical layer has exactly **one motion commander** — the Pi5 over UART. The
backdoor (§3a) can issue discrete, safe, low-rate operations, but it is *not* a
motion-streaming commander, so it does not create a competing motion lease. There
is no AUTO/MANUAL motion arbitration to resolve.

Consequence — the motion deadman is not mode-dependent:

- It watches **only the UART heartbeat** — the pilot's hands on the yoke.
- **RF loss is purely an ATC-comms event** with zero bearing on whether the rover
  keeps moving — exactly as a plane keeps flying through a radio dropout.
- If the **Pi5** hangs, the UART goes quiet and FALLBACK fires. (And if the Pi5
  stays hung, ATC can reboot it through the backdoor — §3a.)

One motion lease, one link, no modes. The backdoor's emergency overrides (ESTOP)
sit outside this entirely and are honored from any source at any time.

---

## 7. Composition — the boundary to confirm at implementation time

A remaining design question, recorded here for the implementation phase:

- A **named, parameterized procedure** — even when internally composite (e.g. a
  three-point turn, back-up-and-realign) — is an *automatic action* and may live
  on the Pico2 as a single engaged behavior.
- A **runtime-assembled sequence** ("do A, then B, then C," composed on the fly)
  is *the plan*. It is the Pilot flying procedures one at a time and must **not**
  be pushed down — otherwise the Pico2 becomes a place where "the plan" secretly
  lives, re-introducing the split-brain hazard of §4.

Proposed boundary (to confirm): named composites on the Pico2; assembled
sequences stay on the Pi5.

---

## 8. Hardware Notes Carried Into Implementation

- **UART, not I2C, for the cockpit.** A cockpit needs instruments that *push*
  alerts (master caution, faults, obstacle reflexes) without being polled. I2C is
  master-driven and cannot let the Pico2 raise asynchronous events; it is also
  more fragile across two boards. UART is full-duplex, supports async events,
  matches the existing `!`-sigil async-event pattern, and preserves power-domain
  isolation.
- **Partition the RP2350's two cores.** Keep the comms / radio-relay work (which
  can block on RF) physically unable to delay the real-time control loop:
  control + reflexes on one core, comms + routing on the other, with a lock-free
  queue between. The cockpit stays responsive even while the radio shuttles
  base↔Pi5 traffic.
- **Downlink payload now multiplexes two streams:** Pico2-generated telemetry,
  plus Pi5→Base relay traffic. Decide how they share the ACK-payload budget
  (fixed split vs. frame-type tag) before it becomes load-bearing.
- **Inner wheel-velocity loop is forced onto the Pico2** — the Pi5's non-real-time
  scheduling cannot reliably close it. The cockpit's primary control surface is
  therefore *velocity*, not raw PWM.
- **Odometry lives on the Pico2** (it has the encoders and the real-time loop) and
  streams in telemetry. Be clear-eyed that Pico-only dead reckoning *drifts* — it
  is a *seed* for the Pi5 to fuse with the map, not real localization.

---

## 9. One-Paragraph Summary

The Base is Air Traffic Control and controls Wanderer only through the Pilot. The
Pilot (Pi5) holds all judgment — the plan, the world model, position-in-house,
emergency planning — and reaches the airframe through a local cockpit interface
(UART). The airframe (Pico2) holds all execution — the real-time velocity loop,
self-terminating relative maneuvers, and always-on reflexes — and never knows the
plan it serves. The Base sees the airframe directly only through a *transponder*
(broadcast telemetry) and a privileged *system backdoor* (emergency overrides,
link/system config, dev diagnostics, reboot-the-Pilot) — neither of which is a
flight interface. The line between Pilot and airframe is not complexity but
judgment: *if a task needs to know where it is in the world or what is around it,
it is the Pilot's; if it only needs to hold a number true against the body's own
senses, it is the airframe's.* Concrete commands will be designed to populate the
three cockpit tiers — control surface, managed procedures, reflexes — against
exactly this division.
