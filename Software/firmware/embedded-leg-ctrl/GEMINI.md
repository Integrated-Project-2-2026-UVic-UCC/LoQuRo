# Embedded Leg Controller (ESP32-S3 Firmware)

This project is a high-performance firmware for an ESP32-S3-based robotic leg controller. It manages 12 PWM servos, reads from an MPU9250 IMU, and interfaces with 12 AS5600 encoders. Low-latency communication with ROS2 is handled via **Zenoh-pico**.

## 🚀 Project Overview

- **Hardware:** ESP32-S3 (N16R8) with 8MB PSRAM enabled.
- **Framework:** Arduino (PlatformIO).
- **Communication:** Zenoh-pico (Client Mode) connecting to an external ROS2 bridge.

## 🛠 Development Environment & Commands

All development operations MUST use the PlatformIO executable located at `~/.platformio/penv/bin/pio`.

### Key Commands
- **Build:** `~/.platformio/penv/bin/pio run`
- **Upload:** `~/.platformio/penv/bin/pio run -t upload`
- **Monitor:** `~/.platformio/penv/bin/pio device monitor`
- **Test:** `~/.platformio/penv/bin/pio test`
- **Clean:** `~/.platformio/penv/bin/pio run -t clean`

### Zenoh Bridge
The Zenoh bridge runs in an external Docker container/server. The firmware acts as a client. Do not attempt to run the bridge locally within this environment.

## 🏗 Architecture & Design

### Task Management (FreeRTOS)
| Task | Core | Priority | Frequency | Description |
|------|------|----------|-----------|-------------|
| `writeServosTask` | 1 | 3 | Async | Updates PCA9685 servos from Zenoh commands. |
| `readIMUTask` | 1 | 2 | 100 Hz | High-frequency IMU data acquisition. |
| `readEncodersTask` | 1 | 2 | 10 Hz | Scans 12 encoders via I2C multiplexer (0x70). |
| `sendStatusTask` | 1 | 2 | Async | Serializes and publishes feedback to Zenoh. |
| `watchdogTask` | 0 | 1 | 0.2 Hz | WiFi management and system health. |

### Hardware Mapping
- **I2C0 (Pins 8, 9):** MPU9250 (0x68), TCA9548A Mux (0x70) -> AS5600 Encoders.
- **I2C1 (Pins 17, 18):** PCA9685 PWM Driver (0x40).

## 📝 Key Files
- `src/main.cpp`: System initialization and task pinning.
- `src/tasks.cpp`: Core logic for control loops and sensor telemetry.
- `include/config.h`: Network parameters and Zenoh Key Expressions.
- `platformio.ini`: PSRAM flags (`board_build.arduino.memory_type = qio_opi`) and dependencies.

## ⚠️ Important Notes
- **PSRAM Access:** Always ensure `-mfix-esp32-psram-cache-issue` flag is maintained for stability.
- **I2C Safety:** Avoid blocking I2C calls; tasks are pinned to Core 1 to minimize interference with WiFi/Bluetooth on Core 0.
