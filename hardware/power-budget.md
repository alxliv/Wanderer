# Wanderer — Power Budget & Architecture

## Two-pack design

| Pack | Config | Voltage (nom / full / empty) | Feeds |
|------|--------|------------------------------|-------|
| **Motor pack** | 3S (3× 18650) | 11.1 V / 12.6 V / ~9.0 V | L298N → motors |
| **Logic pack**  | 2S (2× 18650) | 7.4 V / 8.4 V / ~6.0 V  | buck → 5 V → Zero 2 W, Pico, servos |

Both packs use 18650 protection boards.

```
 Motor pack 3S ──► L298N ──► Motor A / Motor B
 (11.1V)              ▲
                      │ control (PWM + dir) from Pico
                      │
 Logic pack 2S ──► S13V30F5 (5V) ──► 5 V rail ──┬──► Raspberry Pi Zero 2 W
 (7.4V)            buck-boost + bulk cap         ├──► Pico 2 VSYS
                                                 └──► Pan-Tilt servos  (dominant 5 V load)

 ALL GROUNDS COMMON:  motor pack GND = logic pack GND = L298N GND = Pico GND = Zero GND
```

## ⚠️ Common ground (mandatory)
The two packs are separate sources, but **every ground must be tied together**. The I²C
buses, L298N control lines, and encoder signals are all referenced to ground; without a
shared reference the system will misbehave. Single common ground node.

## 5 V rail load estimate

| Consumer | Typical | Peak |
|----------|---------|------|
| Raspberry Pi Zero 2 W | ~0.7 W | ~3 W (5 V @ ~0.6 A) |
| Pi Camera 3 | <1 W | ~1 W |
| Pan-Tilt servos (×2) | ~1–2 W moving | ~10 W stall (brief) — **dominant load** |
| Pico 2 + sensors | <1 W | ~1 W |

→ **5 V regulator: Pololu S13V30F5 (#4082)** — a **buck-boost** 5 V regulator, input 2.8–22 V,
~3 A continuous at the 2S input voltage (rated 2–4 A depending on input). Chosen over a plain
buck because the buck-boost **holds 5 V across the entire 2S discharge** (8.4 V → 6 V and below),
so no end-of-charge brownout; it also has soft-start (gentle Zero boot) plus over-current and
thermal protection. **Add a 470–1000 µF bulk capacitor** on the 5 V rail near the servos to
absorb their current spikes. (Resolves **O1**.)

The Zero 2 W and Pico 2 are both powered as separate branches from the regulated 5 V rail.
Do not route Pico power through the Zero; feed Pico **VSYS** from 5 V and tie Pico GND to the
common ground node.

> Switching to the Zero 2 W dropped the SBC's ~25 W worst-case to ~3 W, removing the
> 5–6 A buck requirement and largely retiring the runtime worry (O4).

## Zero 2 W power entry — OPEN (O2)
- **Option A — 5 V into the GPIO 5V/GND pins:** simplest; the Zero has no PD negotiation to
  worry about. Recommended. Keep wiring short and the buck well-regulated.
- **Option B — via the micro-USB power port** from the buck (USB-A→micro-USB), if preferred
  for the input protection.

## Motor rail notes
- L298N drops ~2–3 V → motors see ~9–10 V. Slightly reduced top speed; accepted.
- L298N ~2 A/channel continuous; MD520 stall current can exceed this. Pico enforces
  stall/timeout protection; avoid prolonged stalls.
- Encoder **logic** is powered separately at **3.3 V** (from the Pico), NOT from the
  motor rail — RP2350 GPIO is not 5 V-tolerant.

## Runtime — Resolved For Prototype (O4)
With the Zero 2 W the logic-rail draw is small (~1–2 W typical), so the 2S pack should give
reasonable runtime; the servos' duty cycle is the main variable. 2S2P remains an easy
upgrade if needed. Motor pack runtime depends on driving duty cycle.

## To finalize in Phase 1
- Exact cell capacity (mAh) and resulting runtime estimate.
- Fusing / inline protection per rail.
- Battery monitoring (voltage/current sense) approach for telemetry.
- Connectors and a main power switch / e-stop.
