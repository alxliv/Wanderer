# Wanderer — Localization Design Notes

**Status:** Decided (design phase) — implementation not started
**Scope:** Indoor localization for a small apartment. Accuracy target: **few centimeters** in position, ~1° in heading.
**Date:** 2026-08-16

---

## 1. Decision Summary

| # | Decision | Choice |
|---|----------|--------|
| D1 | Localization sensor | 2D lidar (LDROBOT **D300 kit, LD19/LD19P sensor**) |
| D2 | Rejected alternatives | UWB, sonar/IR, WiFi RSSI, camera + fiducial markers, overhead camera |
| D3 | Pilot SBC | Stays **RPi Zero 2W** for now; Pi 5 is the documented migration path |
| D4 | Scan processing | Raw scans stream **pilot → base over WiFi**; base runs the estimator and returns pose fixes |
| D5 | Command interface | Base sends **goals** (waypoints), not maneuvers; pilot decomposes goals into cockpit `turn`/`move` legs |
| D6 | Lost-comm behavior | Pilot owns pre-defined autonomous fallback procedures (aviation "lost-comm" model) |
| D7 | Lidar power | From the pilot's existing 5V buck rail — **no separate battery**; direct feed, not through the Pi |
| D8 | SLAM usage | Two-phase: one-time mapping ritual, then continuous localization against the fixed map |

---

## 2. Sensor Selection (D1, D2)

### Chosen: LDROBOT D300 kit (LD19 / LD19P sensor)

- 360° DTOF, ~10 Hz rotation, ~4500 points/sec (~450 pts/rev), 12 m range, UART @ 230400 baud (3.3V), 5V power.
- One-way broadcast protocol (no commands needed) — good fit for the pilot's "read, timestamp, forward" role.
- Small, belt-less, light — mechanically suited to Wanderer.
- Buy the kit variant that includes the USB-UART adapter board (desk bring-up before rover integration).

### Rejected alternatives, with reasons

- **UWB (DW1000/DW3000 trilateration):** real-world indoor accuracy 15–30 cm with multipath — cannot honestly meet a few-cm target. No heading. Requires anchors installed and surveyed **per space**, which fails the versatility/scaling criterion. Positioning-only sensor; lidar also provides obstacle geometry, free space, mapping.
- **D200 kit (LD14P):** ~6 Hz rotation and ~2300 pts/s — more motion smear per scan, weaker scan-matching constraint, 8 m range, worse ambient light tolerance. It is the option you buy when cost is the criterion; cost is not the criterion. Acceptable later as a second/bench unit.
- **Sonar/IR:** obstacle detection only; cone too wide and returns too ambiguous for localization.
- **WiFi RSSI:** meter-level at best.
- **Camera + fiducial markers (ArUco/AprilTags):** capable of the accuracy, but paper tags on walls ruled out as not feasible.
- **Overhead camera at base:** single-room only; may still be useful someday as a dev "motion capture" tool, not as the system.

### Known lidar failure geometries (accepted)

- Long featureless corridors: pose can slide along the corridor axis.
- Glass (balcony doors, windows): invisible to lidar.
- Low furniture / chair legs: sparser, noisier features than walls. Mount the lidar high enough to see over the rover body; a known static map mitigates.

---

## 3. Architecture (D3, D4, D5)

### Responsibility split — same judgment-vs-execution rule, one level up

| Layer | Owns | Localization role |
|-------|------|-------------------|
| **Base (ATC)** | World model: map, scan matching / particle filter, path planning | Consumes timestamped scans; produces pose fixes and **goals** ("go to (x, y)", "dock", "explore room 2") |
| **Pilot (RPI5)** | Judgment: goal → leg decomposition, pose fusion, fallback procedures | Reads lidar UART, timestamps scans **at the pilot**, streams over WiFi; fuses pose fixes with odometry+gyro; issues cockpit `turn`/`move` legs; stops on its own when confidence is lost |
| **Pico2 (airframe)** | Execution: closed-loop metric maneuvers | **Unchanged. Never sees a scan.** Timeout response stays dumb: stop, hold, report |

### Why goals, not maneuvers, from the base

ATC issues clearances, not stick inputs. If the base sent raw `turn 40 / move 1.0`, a WiFi drop mid-sequence leaves the rover executing a stale plan with nobody watching. With goals, the pilot:

- decomposes a waypoint into legs itself (atan2 to target → turn → drive → re-check) — trivial math, well within the Zero 2W;
- can halt safely on its own when pose fixes go stale ("moved 2 m since last fix → confidence gone → stop");
- degrades gracefully to single-leg operation during bring-up ("go to a point 1 m ahead" ≡ "move 1 m").

This also makes autonomy fallback (D6) free: the pilot feeds its own goal queue instead of the base's — no second command pathway needed.

### Timestamping rule (accuracy-critical)

WiFi latency is variable. Scans are timestamped **at the pilot**; the base's pose fixes echo the scan timestamp back. A fix is fused as "where I was *then*," with odometry bridging forward to now. Without this, few-cm accuracy dies in transit.

### Link budget notes

- Scan stream: ~150–180 kbps sustained — fine over WiFi, **2–4× beyond** realistic sustained nRF24 throughput (~40–80 kbps).
- **Scans never go over the RF backdoor.** Two rules block it: the backdoor must never grow into a flight interface (and ESTOP queued behind scan packets is unacceptable), and routing perception through the Pico2 violates judgment-vs-execution.
- If WiFi independence is ever required, the fix is not a fatter RF pipe — it is moving the estimator onboard (Pi 5 path below), shrinking cross-air traffic to pose + commands (~2 kbps).

### Pi 5 migration path (documented, not scheduled)

Swap pilot Zero 2W → Pi 5, pull the estimator onboard. Links then carry only poses and goals; the base becomes a pure monitor/mission issuer and WiFi becomes optional for flight. Same software, different placement. This path exists with lidar and would not have existed with UWB.

---

## 4. Lost-Comm Procedures (D6)

Aviation model: a pilot who loses ATC follows a pre-briefed, **deterministic** procedure that ATC can predict. The value is not only sensible behavior — it is that the base, on reconnect, knows where to look and what state to expect.

### Degradation ladder

| State | Condition | Behavior |
|-------|-----------|----------|
| `LINK_OK` | Base commands and pose fixes fresh | Normal ops |
| `BASE_SILENT` | No base commands; pose fixes still fresh enough | Finish current leg, then hold position (default) |
| `POSE_STALE` | No usable pose fixes | Map-free reactive behaviors using the raw scan stream still flowing through the pilot: nearest clear space, wall-follow, back away from < 20 cm. Includes "hide under a table" class behaviors (drive to nearest large clutter cluster, stop) — odometry + reactive, no pose required |
| `LOST_HOLDING` | Everything degraded, scans doubtful | Stop. Optionally chirp a diagnostic heartbeat (`state = LOST_HOLDING`) on the **backdoor** — legitimate use: diagnostics, not flight commands |

The base may also explicitly command `resume own navigation`, handing full control to the pilot's own goal queue and procedures.

### Placement rule

Fallback procedures live in the **pilot's strategic layer, not Pico2 firmware**. Choosing what to do about a lost base (hold vs. retreat vs. hide) is judgment. The Pico2's timeout response remains: stop wheels, hold, report.

### Action item

The four states above go into the pilot FSM skeleton **from the first version**, even if three of them initially just mean "stop." Cheap now, expensive to retrofit.

---

## 5. SLAM Workflow (D8)

Full SLAM (simultaneous mapping + localization with loop closure) is needed **once per apartment**, not continuously:

1. **Mapping phase (one-time ritual, per furniture rearrangement):** teleop Wanderer via helm, record scans + odometry, run an offline SLAM pass on the base/desktop → occupancy-grid map. No real-time pressure.
2. **Operating phase (continuous):** localization only — scan matching / particle filter (AMCL-style or plain ICP) against the fixed map. This is the part that runs live and carries the few-cm target.

The map is a **calibration artifact** — measured once, then relied upon; philosophically the same as the airframe owning `tpm` and `track`. No full ROS required: the estimator is implementable in Python on the base station, consistent with Python living base-side.

---

## 6. Power (D7)

- **D300 requirements:** 5V nominal, ~290–350 mA running, ~450–500 mA motor spin-up spikes. Does **not** tolerate 2S directly.
- **Decision:** feed from the pilot's existing 2S→5V buck rail. No separate battery — that pattern is reserved for isolating the safety-critical Pico2, which the lidar is not; it is a pilot-side peripheral and belongs on the pilot-side rail.
- **Combined 5V budget:** Zero 2W (~150–400 mA typ., 600–700 mA WiFi bursts) + D300 ≈ **1.0–1.2 A worst case**. Requires a 3 A-class buck; a 1 A module is undersized and must be upgraded.
- **Wiring rules:**
  1. **Never power the lidar through the Pi.** Feed the D300's 5V pin directly from the buck output. Data via GPIO UART pins, or via the kit's USB adapter with its 5V line cut (data only). The Zero's micro-USB/onboard power path is a known brownout point; motor spike + WiFi burst = mystery resets.
  2. **Decouple at the D300 connector:** ~100 µF electrolytic + 100 nF ceramic (same medicine as the nRF24).
- **Open checks:** confirm the actual buck module's honest continuous rating; re-verify 2S pack runtime with ~2 W continuous extra draw (lidar runs whenever Wanderer is awake).

---

## 7. Interface Note — UART Conflict

`/dev/serial0` on the pilot is already the cockpit link to the Pico2. The lidar therefore connects via the kit's **USB-UART adapter** (5V line cut per §6), sidestepping the conflict and keeping lidar current off the GPIO rail. Lidar UART is 3.3V @ 230400 baud if GPIO connection is ever revisited.

---

## 8. Priority Order (unchanged near-term)

1. Bench-test `turn` sign convention, tune `OVERSHOOT_COMP_DEG`, verify command timeout on real UART (current milestone).
2. Order D300 kit; desk bring-up with vendor viewer via USB adapter.
3. Lidar → pilot → WiFi → base data path; **live scan visualization against odometry-predicted pose** (exercises the whole pipeline before any SLAM math exists, and builds intuition for drift).
4. Mapping ritual + offline SLAM pass → apartment map.
5. Live localization (estimator on base) → pose fixes → goal interface on pilot.
6. Lost-comm FSM states in pilot skeleton (stub behaviors acceptable initially).
