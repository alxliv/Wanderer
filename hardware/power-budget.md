# Wanderer — Power Budget & Architecture

## Two-pack design

| Pack | Config | Voltage (nom / full / empty) | Feeds |
|------|--------|------------------------------|-------|
| **Motor pack** | 3S (3× 18650) | 11.1 V / 12.6 V / ~9.0 V | L298N → motors |
| **Logic pack**  | 2S (2× 18650) | 7.4 V / 8.4 V / ~6.0 V  | buck → 5 V → Pi 5, Pico, servos |

Both packs use 18650 protection boards.

```
 Motor pack 3S ──► L298N ──► Motor A / Motor B
 (11.1V)              ▲
                      │ control (PWM + dir) from Pico
                      │
 Logic pack 2S ──► BUCK (≥5–6 A) ──► 5 V rail ──┬──► Raspberry Pi 5
 (7.4V)                                          ├──► Pan-Tilt servos
                                                 └──► Pico (via Pi 5 5V)

 ALL GROUNDS COMMON:  motor pack GND = logic pack GND = L298N GND = Pico GND = Pi 5 GND
```

## ⚠️ Common ground (mandatory)
The two packs are separate sources, but **every ground must be tied together**. The I²C
buses, L298N control lines, and encoder signals are all referenced to ground; without a
shared reference the system will misbehave. Single common ground node.

## 5 V rail load estimate

| Consumer | Typical | Peak |
|----------|---------|------|
| Raspberry Pi 5 (8 GB) | ~3–5 W | up to ~25 W (5 V @ 5 A) |
| Hailo 8L (via PCIe) | ~2–3 W | ~5 W |
| Pi Camera 3 | <1 W | ~1 W |
| Pan-Tilt servos (×2) | ~1–2 W moving | ~10 W stall (brief) |
| Pico 2 + sensors | <1 W | ~1 W |

→ **Buck converter target: ≥ 5 A, ideally 6 A continuous at 5 V**, with headroom for
servo spikes. **OPEN (O1):** confirm available buck / rating, else add to BOM.

## Pi 5 power entry — OPEN (O2)
- **Option A — 5 V into GPIO 5V/GND pins:** simple, but bypasses USB-C PD negotiation;
  Pi 5 may show a low-power/under-voltage warning and limit USB current. Quietable via
  firmware config. Fine here (Hailo is on PCIe, not USB).
- **Option B — USB-C from buck:** cleaner negotiation/protection; needs a buck with proper
  5 V/5 A USB-C output or a USB-C breakout.

## Motor rail notes
- L298N drops ~2–3 V → motors see ~9–10 V. Slightly reduced top speed; accepted.
- L298N ~2 A/channel continuous; MD520 stall current can exceed this. Pico enforces
  stall/timeout protection; avoid prolonged stalls.
- Encoder **logic** is powered separately at **3.3 V** (from the Pico), NOT from the
  motor rail — RP2350 GPIO is not 5 V-tolerant.

## Runtime — OPEN (O4)
2S logic pack has no parallel capacity; under Pi 5 + Hailo + servo load, expect short
runtime. Future upgrade: 2S2P (4 cells). Motor pack runtime depends on driving duty cycle.

## To finalize in Phase 1
- Exact cell capacity (mAh) and resulting runtime estimate.
- Buck converter part + rating (O1).
- Fusing / inline protection per rail.
- Battery monitoring (voltage/current sense) approach for telemetry.
- Connectors and a main power switch / e-stop.
