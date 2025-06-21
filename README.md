# AWC Low Level Motor Controller

This project provides low-level motor control and speed feedback for a differential drive robot using an Arduino-compatible microcontroller. It interfaces with two motors via DACs (Adafruit MCP4725), reads wheel speed via pulse inputs, and communicates with a host computer over serial.

## Features

- Controls left and right motors using MCP4725 DACs.
- Reads wheel speed using pulse inputs and calculates RPM.
- Filters out RPM spikes for robust feedback.
- Receives velocity commands and sends back measured RPMs over serial.

## Hardware Connections

- **DACs:**  
  - Right DAC (MCP4725) at I2C address `0x60`  
  - Left DAC (MCP4725) at I2C address `0x61`
- **Direction Pins:**  
  - Left: Digital pin 7  
  - Right: Digital pin 6
- **Speed Pulse Inputs:**  
  - Left: Digital pin 2  
  - Right: Digital pin 3

## Serial Protocol

- **Input:**  
  - 12 bytes per command:  
    - `float x` (forward velocity, m/s)  
    - `float vy` (lateral velocity, m/s, not used)  
    - `float omega` (angular velocity, rad/s)
- **Output:**  
  - 8 bytes per feedback:  
    - `float rpm_left`  
    - `float rpm_right`

## Usage

1. **Wiring:**  
   Connect the DACs, motors, and speed sensors as described above.
2. **Flashing:**  
   Upload `main.cpp` to your Arduino-compatible board.
3. **Serial Communication:**  
   Send 12-byte velocity commands at 115200 baud.  
   Read 8-byte RPM feedback at ~10 Hz.

## Parameters

- `WHEEL_RADIUS`: 0.1651 m
- `WHEEL_BASE`: 0.46 m
- `motorPolePairs`: 15

