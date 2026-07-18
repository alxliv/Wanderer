# Cockpit Bench Test — Pilot ↔ Airframe over UART

First live bring-up of the cockpit link (`protocol/cockpit_protocol.md`):
the Pilot SBC drives the airframe Pico2 over UART0, by hand, before any
pilot software exists. Passing this test proves the wire, the codec, the
handler, the tactical FSM, the deadman, and the event stream — end to end
on real hardware.

> **Board naming.** Project convention: the Pilot SBC is called "RPI5"
> regardless of the board actually fitted. The serial-port *configuration*
> however differs between boards, so this document says which physical board
> each instruction applies to. Current pilot hardware: **Raspberry Pi
> Zero 2 W** (Pi 5 differences in the last section).

There are two separate serial paths in this setup — don't confuse them:

| Path | Physical port | Protocol | Purpose |
|---|---|---|---|
| Cockpit | Pico2 GP0/GP1 (UART0, 3.3 V TTL) | cockpit text protocol | The flight interface under test |
| Bench log | Pico2's own micro-USB connector | USB CDC (`printf` stdio) | Boot line + debug logs only |

---

## 1. Wiring — three wires, crossed

Both boards are 3.3 V logic: direct connection, no level shifter.

| Pilot 40-pin header | | Airframe Pico2 |
|---|---|---|
| Pin 8 — GPIO14, **TXD** | → | **GP1** (UART0 RX) |
| Pin 10 — GPIO15, **RXD** | → | **GP0** (UART0 TX) |
| Pin 6 — GND | → | any Pico GND pin |

- TX→RX **crossover** on both signal wires. TX-to-TX is the classic silent
  failure: both sides talk, nobody listens.
- The GND wire is **not optional**, even with separately powered boards. A
  UART is a voltage referenced to ground; this wire is the shared reference
  (consistent with the project's starred-ground topology).
- Connect **nothing** to either board's supply pins over this harness — each
  board keeps its own power.

## 2. Pilot configuration (Raspberry Pi Zero 2 W)

Two things must be true: the header UART is **enabled**, and the Linux
**serial console is disabled** on it. If the console is left on, a `getty`
login prompt spews text into the cockpit and interprets `!state` lines as
login attempts.

```
sudo raspi-config
  → Interface Options → Serial Port
  → "login shell over serial?"        → No
  → "serial port hardware enabled?"   → Yes
sudo reboot
```

Equivalent by hand: add `enable_uart=1` to `/boot/firmware/config.txt` and
delete `console=serial0,115200` from `/boot/firmware/cmdline.txt`.

**Always use the device `/dev/serial0`** — a symlink that points at the
primary header UART on any Pi model. Pilot software must take the port as a
parameter defaulting to `/dev/serial0`, so it runs unmodified when the board
is swapped.

Zero 2 W wrinkle: its full PL011 UART (`ttyAMA0`) is claimed by Bluetooth,
so `serial0` points at the **mini-UART** (`ttyS0`), whose baud clock tracks
the core clock. `enable_uart=1` pins the core frequency so 115200 stays
accurate — that setting matters beyond merely "on". If bytes ever garble
under CPU load, add `dtoverlay=disable-bt` to `config.txt` (the pilot has no
Bluetooth use) to hand the PL011 to the header pins.

Sanity checks after reboot:

```
ls -l /dev/serial0     # symlink exists
groups                 # your user is in "dialout"
                       # if not: sudo usermod -aG dialout $USER, re-login
```

## 3. The test session

On the pilot:

```
sudo apt install picocom
picocom -b 115200 /dev/serial0
```

Power the airframe. **Wheels off the ground** — `drive` moves real motors.
Then type the session (this is `protocol/cockpit_protocol.md` §9, live):

```
ping                 →  =ok ping
get_version          →  =ok get_version fw=0.3
get_state            →  =ok get_state state=SAFE
arm                  →  !state from=SAFE to=ACTIVE
                        =ok arm
drive 0.2 0.0        →  =ok drive        (wheels spin)

   ...stop typing for ~1 second...

                     →  !state from=ACTIVE to=FALLBACK   (unprompted!)
drive 0.2 0.0        →  !state from=FALLBACK to=ACTIVE
                        =ok drive
stop                 →  =ok stop          (wheels stop, still ACTIVE)
estop                →  !fault code=ESTOP
                        !state from=ACTIVE to=FAULT
                        =ok estop
arm                  →  =err arm fault_latched
clear_fault          →  !state from=FAULT to=SAFE
                        =ok clear_fault
disarm               →  =ok disarm       (no-op ok from SAFE)
```

Reading the session:

- The unprompted `!state from=ACTIVE to=FALLBACK` while you sit idle is the
  most important line in the test: the **deadman** and the change-state
  callback working over the real wire, with no request in flight.
- Events may arrive **before** the `=ok` reply of the command that caused
  them (spec §2 permits either order; this implementation emits the event
  from inside the command).
- picocom does not echo your typing by default (`picocom --echo` if wanted).
  Line endings don't matter — the airframe ignores `\r`, so CR, LF, or CRLF
  all work. Exit with `Ctrl-A` then `Ctrl-X`.

Meanwhile the Pico's own micro-USB (bench-log path) shows the boot line
`*airframe fw 0.3 cockpit on uart0 @115200` on whatever host it's plugged
into — a second, independent window into the airframe.

## 4. Troubleshooting, in order

1. **Nothing comes back.** Swap the GP0/GP1 wires first — the TX/TX mistake.
   Then confirm the console is truly off:
   `cat /boot/firmware/cmdline.txt` must not contain `console=serial0`.
2. **Garbage characters.** Baud mismatch, or mini-UART clocking — see the
   `disable-bt` note in §2.
3. **Isolate which side is broken.** Unplug the Pico and short pilot pins
   8↔10 together (loopback): picocom should echo every keystroke. If it
   does, the pilot side is fine — suspect the harness or the airframe.
4. **Login prompt or boot text appears.** The serial console is still
   enabled; redo §2.

## 5. Board swap: actual Raspberry Pi 5

Same three wires, same header pins 8/10/6. Configuration differs:

- Enable the header UART with `dtparam=uart0=on` in
  `/boot/firmware/config.txt` (instead of the Zero 2 W's mini-UART dance).
- `serial0` ends up pointing at `ttyAMA10` — irrelevant to software that
  only ever opens `/dev/serial0`, which is the point of the convention.
- No Bluetooth/mini-UART wrinkle: the Pi 5 header UART is a full UART.

## 6. What this test clears the way for

With the manual session passing, the next step is the pilot's
`UartCockpitLink` (pyserial, port defaulting to `/dev/serial0`): the
existing `Cockpit` API, the simulator-validated test suite, and the spec §6
matrix then run **unchanged** against the real airframe — the payoff of the
`CockpitLink` abstraction.
