# Cyclic Loading Test Rig
**Author:** Linus Ritchie  
**Date:** 2026-01-30

## Overview
This program maintains a constant maximum force applied to a  test rig using a crank–slider mechanism. A control loop adjusts the motor output to match a configured target maximum force, with both the force setpoint and allowable force variation defined in the Python control script.

Force measurement is cycle‑based. When the limit switch is triggered, the system begins recording force values into a buffer. On the next trigger, the maximum recorded value is extracted from that buffer, sent over serial to the PC, and the buffer is cleared for the next cycle. Because the limit switch is mounted on the crank motor, this occurs once per revolution. The limit switch triggers an interupt resulting in the program being able to run a various speeds.

---

## Setup
Use longer, securely soldered wiring. Header pins and loose jumpers can cause intermittent or noisy signals.

The wiring diagram is as follows:

<!-- ![alt text](WiringDiagram.png) -->
<img src="WiringDiagram.png" width="600" height="400">

---

## Limit Switch
The limit switch is configured as an interrupt in `main.cpp`, triggering force-value transmission each revolution.

A test task optionally generates a simulated limit-switch event every 500 ms (2 Hz motor equivalent) for debugging and visualization. This is simply implemented as a task that is triggered on the desired period of a simulated limit switch press. This will need to be disabled when the limit switch is triggered by the motor each run. 

---

## Crank Motor
### Pin Connections
**Motor Driver → Arduino**
- IN1 → 11
- IN2 → 12
- ENA → 10

### Power
Requires **12 V** connected to motor driver Vin and GND.

### Current Monitoring
Motor current is measured using a **1.6 Ω** shunt resistor. Voltage–to–current conversion is implemented in `main.cpp`. Over‑current disables system motion.

---

## Stepper Motor
### Power
Requires **20–50 V**. Recommended: 20 V lab bench supply. A compatible driver capable of running from **12 V** is on order.

### Stepping Mode
Configured for **1/16 microstepping**. DIP switches can be adjusted for smoother or coarser motion.

### Wiring
Already connected to the Arduino. Ensure correct header orientation: GND (black wire) must align with Arduino GND pins.

---

## Load Cell (TODO)
The load cell will use an analog amplifier to generate an ADC‑compatible signal (Range 0-5V).

TODO:
- Wire load cell and amplifier
- Verify amplifier output
- Integrate into control loop

### Backup Amplifier — HX711
HX711 is available as a fallback. Its **80 Hz max** sample rate is insufficient for accurate peak detection at **2 Hz** crank speed (≈40 samples/rev).
This is only suitable for running a force exterted calibration with the cranker locked out at the start of a test.

---

## Temperature Sensor
A temperature sensor is on order. No interface code written yet.

---

## File Structure
### `src/main.cpp`
Contains:
- State machines
- Task definitions and scheduling
- Control loop logic

Tasks use the **Arduino TaskScheduler** library. `setup()` configures tasks; `loop()` runs the scheduler. All intervals are configurable in `main.cpp`.
