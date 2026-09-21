# Mimi Feeder

A two-board ESP32-S3 automated pet feeder and environment monitoring system designed for a class final project. The system dispenses a portion-controlled meal on a persistent four-hour schedule, monitors temperature, humidity, and water level, and provides both a local LCD display and a remote BLE interface for status and manual control.

## Overview

Mimi Feeder was built to solve a real scheduling problem: cats do best with predictable, portion-controlled meals, but a student schedule makes fixed feeding times unreliable. The solution is an automated feeder that preserves a trusted schedule even across power loss, while making the feeding station easy to monitor and control.

The project was implemented as a two-board architecture:

- Board A: the feeder board, responsible for scheduling, sensing, actuation, and persistence
- Board B: the console board, responsible for the LCD, BLE connectivity, and state display

This separation keeps the timing-sensitive stepper motion and sensor tasks away from the more unpredictable BLE stack, improving reliability under a real-time operating system.

## Technical summary

### Hardware architecture

- 2x ESP32-S3 boards
- AM2320 temperature/humidity sensor
- DS1307 RTC with battery-backed NVRAM
- 28BYJ-48 stepper motor with ULN2003 driver
- Analog water-level sensor with power-gated sampling
- 16x2 I2C LCD
- BLE GATT service for command and status communication
- Status LEDs and local controls

### Embedded systems design

The firmware is built in Arduino/ESP32 C++ with FreeRTOS. The design uses:

- task priorities to separate critical timing from less critical telemetry
- queues for inter-task communication
- mutexes for shared hardware like the I2C bus and shared state
- software timers for feed deadline management
- core pinning so sensor and motor work do not compete on the same processor core
- persistent validation of stored schedule values in RTC NVRAM

### Feed scheduling and reliability

The feeder stores an absolute Unix epoch for the next scheduled feed rather than a countdown alone. This makes the schedule more robust against missed ticks and resets. On boot, the system validates stored values before trusting them, and if the stored data is invalid or implausible it resets to a fresh schedule instead of silently using bad data.

### Sensor processing

Both the climate readings and the water readings are debounced and confirmed before they trigger a visible state change. This avoids flicker from noisy sensor values near the thresholds.

### Communication protocol

Board A sends telemetry over UART as framed ASCII messages with a checksum, similar in spirit to NMEA 0183. Each frame includes a type and payload, and the console board validates the framing before updating its mirrored state.

The protocol supports:

- climate telemetry
- water telemetry
- schedule heartbeat reports
- feed completion reports
- boot reports
- command acknowledgements
- fault reports

## Project implementation

This repository contains the firmware used for the final project:

- `BoardA_MimiFeeder_v5.ino` — feeder firmware
- `BoardB_Console_v3.ino` — console firmware

These sketches were implemented to satisfy the project goals of:

- deterministic task scheduling under FreeRTOS
- correct use of queues, mutexes, and timers
- robust inter-board serial communication
- hardware abstraction and safe shared state patterns
- factored code organization and explicit peripheral ownership

## What I contributed

I completed the coding for the project and implemented the majority of the real embedded-systems work, including the sensor logic, scheduling logic, motion control, communication protocol, and firmware integration. My partner contributed to the overall hardware/software collaboration, design integration, testing, and the final report, as described in the project documentation.

This repository is intended to demonstrate the engineering work and technical decisions behind the final implementation while still accurately crediting the collaborative nature of the project.

## Portfolio-friendly summary

This project demonstrates practical experience with:

- FreeRTOS task design and scheduling
- ESP32-S3 development
- multi-board embedded system architecture
- I2C and UART protocol integration
- actuator control with a stepper motor
- BLE and LCD console development
- device persistence and fault recovery
- debugging real hardware under time and correctness constraints

In resume language, this project shows the ability to design and implement a real-time embedded system that balances hardware reliability, software timing, and user-facing status reporting in a constrained environment.

## Repository layout

```text
mimi-feeder/
├── README.md
├── BoardA_MimiFeeder_v5.ino
├── BoardB_Console_v3.ino
├── .gitignore
├── LICENSE
└── docs/
    └── (optional additional notes if needed)
```

## Notes on the design

The most important system-level lesson from this project is that building a device that works is not the same as building a device that communicates failure clearly. The final design includes timeouts, validation checks, sensor confirmation, and fault reporting so the system tells you when something is wrong instead of silently failing.

This is a core embedded-systems skill and one that matters for any unattended or safety-related device.

## Future improvements

Potential enhancements include:

- closed-loop dispense verification using a load cell or optical chute sensor
- interrupt-driven UART receive instead of polling
- bidirectional heartbeat monitoring between the boards
- lower-power sleep modes for battery operation
- more automated validation and testing of the serial protocol and schedule logic

## Credits

This project was completed collaboratively. The final report credits:

- Ayden: Board A firmware responsibilities, including sensor integration, I2C mutex design, stepper/servo drive path, and NVRAM persistence
- Leo Hoffman: Board B firmware responsibilities, including the BLE GATT server, LCD page system, and frame parser
- Joint work: the inter-board protocol, integration, testing, and the report itself

## License

This project is open-source and released under the MIT License.

---

Built as a final project in embedded systems and real-time firmware design.
