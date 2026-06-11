# JGB-520 Gear Motor with Encoder Pinout & Specifications

This document outlines the connector types, pin configurations, and operational logic for the **JGB-520** DC gear motor equipped with a rear magnetic quadrature encoder.

---

## Connector Overview

* **Connector Family:** JST-XH Series
* **Pitch:** 2.54 mm (0.1 inches)
* **Type:** 6-Pin Male Header (Side/Top-Entry)
* **Mating Connector:** 6-Pin JST-XH Female Housing with crimp terminals

The connector bridges two completely independent electrical systems: the high-current DC motor armature supply and the low-current digital Hall-effect sensor array.

---

## Pinout Configuration

Looking at the rear blue encoder PCB (with the JST-XH connector positioned as shown in the reference diagrams), the pin allocation from one side to the other follows this standard layout:

| Pin Number | Name | Type | Function |
| :---: | :--- | :--- | :--- |
| **1** | **M+** | Power (Input) | **Motor Power Positive.** Connects to the H-bridge driver output. Driving this HIGH relative to M- spins the motor forward. |
| **2** | **M-** | Power (Input) | **Motor Power Negative.** Connects to the H-bridge driver output. Reversing polarity between M+ and M- reverses the spin direction. |
| **3** | **GND** | Sensor Ground | **Encoder Reference Ground (0V).** Must be tied to the microcontroller's logic ground. |
| **4** | **VCC** | Sensor Power | **Encoder Logic Supply (3.3V to 5V DC).** Powers the internal dual Hall-effect sensor ICs. |
| **5** | **A (OUT A)** | Digital Output | **Encoder Channel A.** Square wave output pulse train used to calculate rotational speed. |
| **6** | **B (OUT B)** | Digital Output | **Encoder Channel B.** Square wave output pulse train offset by 90° relative to Channel A. Used to decode rotation direction. |

---

## Detailed Functional Principles

### 1. Motor Armature Power (`M+`, `M-`)
* **Electrical Isolation:** These pins bypass all logic circuitry on the PCB and route directly to the heavy structural solder pads bonded to the DC motor's brush terminals.
* **Control Requirements:** **Never** connect these pins directly to a microcontroller I/O pin (e.g., Arduino, Raspberry Pi Pico). The stall current of the JGB-520 can easily exceed several amperes, which will immediately destroy logic-level outputs. Always interface via a dedicated motor driver such as the project's Cytron MDD10A.
* **Speed Control:** Achieved by applying a Pulse Width Modulation (PWM) signal through the motor driver to modulate the effective DC voltage.

### 2. Encoder Sensor Power (`VCC`, `GND`)
* **Circuitry:** These pins power the dual Hall-effect sensors mounted on the PCB. A multi-pole magnetic disc is permanently pressed onto the rear extension of the motor armature shaft. As the shaft spins, the alternating magnetic poles pass over the stationary sensors.
* **Logic Levels:** Safe to power directly from the microcontroller's `3.3V` or `5V` regulated power rails. Current draw is minimal (typically $< 20	ext{mA}$).

### 3. Quadrature Signal Logic (`A`, `B`)
The sensor outputs form a **Quadrature Encoder** interface. Because the two Hall-effect sensors are physically offset on the PCB, the resulting square-wave digital signals are shifted in phase by exactly 90 degrees.

```
       CLOCKWISE ROTATION (A leads B)
       ___     ___     ___     ___
CH A _|   |___|   |___|   |___|   |___
         ___     ___     ___     ___
CH B ___|   |___|   |___|   |___|   |___

       COUNTER-CLOCKWISE ROTATION (B leads A)
         ___     ___     ___     ___
CH A ___|   |___|   |___|   |___|   |___
       ___     ___     ___     ___
CH B _|   |___|   |___|   |___|   |___
```

* **Velocity Estimation:** Configure an external input interrupt on your microcontroller to trigger on the rising or falling edges of **Channel A**. By counting the number of pulses within a precise time interval ($\Delta t$), you can calculate the rotational frequency of the motor shaft. Combining this with the known gear reduction ratio yields the exact wheel RPM.
* **Directional Decoding:** Read the instantaneous state of **Channel B** at the exact moment a rising edge occurs on **Channel A**:
  * If Channel B is **LOW** during a rising edge on A, the motor is rotating in one direction (e.g., Clockwise).
  * If Channel B is **HIGH** during a rising edge on A, the motor is rotating in the inverse direction (e.g., Counter-Clockwise).
