# Wanderer — Bill of Materials

Working BOM. **Status:** ✅ have · 🛒 to buy · 🔮 future. Quantities are per robot.
Decision references point at [docs/01-decisions.md](../docs/01-decisions.md).

## Drivetrain & chassis
| Qty | Item | Notes | Status | Ref |
|----:|------|-------|:------:|-----|
| 1 | Perforated aluminum chassis (2-tier) | from the prototype build | ✅ | — |
| 2 | MD520 (JGB37-520) gearmotor, 12 V, 550 RPM, quadrature Hall encoder | [yahboom.net/study/MD520](https://www.yahboom.net/study/MD520); **2 front driven wheels** | ✅ | D1, D12 |
| 1 | Rear caster wheel | passive, trailing | ✅ | D1 |

## Electronics — compute & control
| Qty | Item | Notes | Status | Ref |
|----:|------|-------|:------:|-----|
| 1 | Raspberry Pi Pico 2 (RP2350) | reflexive layer | ✅ | D6 |
| 1 | Raspberry Pi Zero 2 W | tactical relay (ARM64, 2.4 GHz Wi-Fi) | 🛒 | D23 |
| 1 | L298N dual H-bridge motor driver | ~9–10 V to motors after drop | ✅ | D7 |

## Sensing
| Qty | Item | Notes | Status | Ref |
|----:|------|-------|:------:|-----|
| 1 | VL53L0X ToF distance sensor | front-facing; [pololu #2490](https://www.pololu.com/product/2490); I²C 0x29 | ✅ | D8 |
| 1 | Pololu MinIMU-9 v6 IMU | LSM6DSO @ 0x6B + LIS3MDL @ 0x1E; [pololu #2862](https://www.pololu.com/product/2862); on the Zero | ✅ | D9 |

## Camera
| Qty | Item | Notes | Status | Ref |
|----:|------|-------|:------:|-----|
| 1 | Raspberry Pi Camera Module 3 | periodic stills | ✅ | D10 |
| 1 | Pan-Tilt HAT | camera aim; I²C 0x15 | ✅ | D10 |
| 1 | Zero-specific camera FFC cable (22-pin 0.5 mm → 15-pin) | Cam 3 ↔ Zero 2 W | 🛒 | D10 |

## Power
| Qty | Item | Notes | Status | Ref |
|----:|------|-------|:------:|-----|
| 3 | 18650 Li-ion cell — **motor pack (3S)** | ≈11.1 V → L298N | ✅ | D13 |
| 2 | 18650 Li-ion cell — **logic pack (2S)** | ≈7.4 V → 5 V regulator | ✅ | D14 |
| 2 | 18650 protection / BMS board | one per pack | ✅ | D15 |
| 1 | **Pololu S13V30F5 (#4082)** 5 V buck-boost regulator | [pololu #4082](https://www.pololu.com/product/4082); 2.8–22 V in, 5 V ~3 A; holds 5 V across full 2S discharge | 🛒 | D31 |
| 1 | Bulk capacitor 470–1000 µF (≥10 V) | on 5 V rail near servos, absorbs spikes | 🛒 | D31 |
| 1 | Main power switch / e-stop | per-rail fusing TBD | 🛒 | — |

## Wiring & misc
| Qty | Item | Notes | Status | Ref |
|----:|------|-------|:------:|-----|
| 2 | 4.7 kΩ resistor — I²C pull-ups | on the Zero ↔ Pico bus | 🛒 | — |
| — | Wiring, connectors, standoffs | common ground bus | partial | D16 |

## Future / reserved
| Qty | Item | Notes | Status | Ref |
|----:|------|-------|:------:|-----|
| 1 | ESP-01S (ESP8266) | backup low-rate command/telemetry radio | ✅ (on hand) | D29 |
| — | Extra 18650 ×2 for **2S2P** logic pack | runtime upgrade | 🔮 | O4 |
| — | 2nd/3rd VL53L0X (side/corner) | needs XSHUT or TCA9548A for addressing | 🔮 | D8 |

> Shelved (not on the robot): **Hailo 8L AI Kit** — perception moved to the PC GPU (D24, D26).
