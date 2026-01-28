# Hybrid IoT Sensor Hub for Field Monitoring

This project implement a hybrid IoT sensor hub developed as part of ongoing research with **Professor Matthew Caesar**. The system is designed to support experimental field deployments by combining low-level microcontroller sensing with Linux-based orchestration on a Raspberry Pi.

The goal of this project is to create a reliable, extensible sensor platform that can collect environmental, motion, and proximity data while maintaining deterministic behavior under real-world conditions.

## System Architecture

The sensor hub follows a **hybrid MCU → Linux architecture**:

- An **Arduino Nano** handles low-level, timing-sensitive sensor reads.
- A **Raspberry Pi** runs Linux and manages orchestration, logging, and event-driven actions.
- Communication between the Nano and Pi occurs over **UART** using `/dev/serial0`.

This split design isolates real-time sensor acquisition from Linux scheduling jitter, allowing the system to remain responsive even under load.

## Sensor Subsystems

### Arduino Nano (MCU Layer)

The Arduino Nano is responsible for directly interfacing with sensors that require precise timing or blocking operations:

- MQ gas sensor (analog)
- HC-SR04 ultrasonic distance sensor
- BMP280 temperature and pressure sensor
- OLED display for local feedback and debugging

Sensor data is periodically read and streamed over UART in a structured key=value format for easy parsing on the Raspberry Pi.

### Raspberry Pi (Linux Layer)

The Raspberry Pi aggregates data from multiple sources and coordinates higher-level behavior:

- Parses UART telemetry from the Arduino Nano in a dedicated reader thread
- Reads local sensors including:
  - PIR motion sensor (GPIO interrupt-based)
  - DHT11 temperature and humidity sensor
  - MPU6050 IMU over I²C
- Handles event-driven camera capture using `rpicam-still`
- Formats all sensor outputs into a fixed-width, real-time console table

## Event-Driven Design

- PIR motion events trigger non-blocking photo capture using background threads.
- Rate limiting prevents excessive camera activation during rapid motion changes.
- Sensor reads and UART parsing are protected with thread locks to avoid race conditions.
- Linux-side tasks are designed to never block the main loop.

This approach ensures that motion events, sensor polling, and logging can occur concurrently without interfering with each other.

## Data Flow

- Arduino Nano streams sensor telemetry over UART.
- Raspberry Pi parses and stores the most recent MCU state in shared memory.
- Local sensors are sampled on a fixed interval.
- All data is printed in a deterministic, fixed-width table for live monitoring.
- Photo capture events are recorded inline without additional log spam.

## Design Decisions

- Uses UART instead of I²C or SPI for MCU → Pi communication to simplify wiring and debugging.
- Separates real-time sensor logic from Linux user-space to reduce timing variability.
- Avoids excessive logging to keep terminal output readable during long runs.
- Prioritizes robustness and clarity over maximum throughput.

## Tech Stack

- Python (Linux orchestration)
- Arduino C++ (MCU firmware)
- Raspberry Pi OS
- OpenCV camera tools (`rpicam-still`)
- Media interfaces: GPIO, UART, I²C, ADC
- Sensors: PIR, DHT11, MPU6050, MQ series, HC-SR04, BMP280

## Notes

- The system is designed for continuous operation in experimental environments.
- Individual sensors can fail gracefully without crashing the hub.
- This project serves as a research-grade prototype and a foundation for future sensor network expansions under Professor Caesar’s lab.
