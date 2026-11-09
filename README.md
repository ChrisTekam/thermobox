# Thermobox

A simple thermal control system that keeps the temperature inside a small foam box close to a setpoint using a PI controller. Built by group KSTW for the course *Regeltechniek* (Control Systems).

## Overview

A power resistor acts as the heater and is switched by an Arduino Uno through a PWM signal and a transistor/MOSFET driver circuit. A DS18B20 probe measures the temperature, and the Arduino runs a PI control loop (500 ms) to track the reference and reject disturbances (e.g. briefly opening the box). The system is modeled and simulated in Simulink and then tested on real hardware.

**Goal:** minimize settling time and overshoot while rejecting disturbances.

## Hardware

| Component | Role |
| --- | --- |
| Arduino Uno | Runs the PI control loop |
| DS18B20 temperature probe | Feedback sensor (±0.5 °C) |
| 6.2 Ω / 10 W power resistor | Heater (7.5 V supply) |
| IRF44ZN MOSFET + 2N3904 NPN | Heater driver |
| EPS foam box (~0.5 L) | Thermal enclosure |

## Repository structure

| Folder | Contents |
| --- | --- |
| `source/` | Arduino code (PI controller and sensor logic) |
| `scripts/` | All MATLAB scripts |
| `simulation/` | Simulink model (`.slx`) |
| `tests/` | Contains all files regarding tests|
| `report/` | LaTeX source of the practical report |
| `docs/` | Project documentation |
| `assets/` | Images and diagrams of the system |
| `data sheets/` | Component data sheets |
| `libraries/` | Arduino libraries used |

## Getting started

1. Build the circuit as shown in the heater driver diagram (see `assets/`).
2. Open the Arduino code from `source/` in the Arduino IDE, install the libraries from `libraries/`, and upload it to the Uno.
3. To log data, run the script `log_serial.py` while the Arduino is connected (it records one sample per second to a `.csv`).
4. To run the simulation, open the `.slx` files in `simulation/` with MATLAB/Simulink.

## Team

| Name | Role |
| --- | --- |
| Rishaan Khedoe | Components, materials, building the foam box |
| Divyesh Sewratan | Simulink modeling, proofreading, communication |
| Christopher Tekam | Project and report lead |
| Donovan Wartim | Documentation and Arduino programming lead |
