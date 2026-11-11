# Embedded Leg Controller (ESP32-S3 firmware)

Firmware of the LoQuRo quadruped: 12 PWM servos through a PCA9685, an MPU9250 IMU, and ROS2 communication over
Wi-Fi with zenoh-pico (client of a `zenoh-bridge-ros2dds` running on a PC). Arduino framework on PlatformIO.

## Commands

Use the PlatformIO executable at `~/.platformio/penv/bin/pio`. Building needs `cmake` (zenoh-pico).

- Build: `pio run`
- Upload: `pio run -t upload`
- Tests on the board: `pio test` (runs `test_imu` and `test_network`), `pio test -f test_legs` (moves the legs),
  `test_leg_limits` (interactive, moves a leg: upload with `--without-testing` and use `pio device monitor`)
- Compile a test without the board: `pio test -f <name> --without-uploading --without-testing`

Never upload the firmware or run `test_legs` / `test_leg_limits` without the user watching: the robot moves.

## Hardware

- I2C0 `Wire` (SDA 8, SCL 9): MPU9250 (0x68) + AK8963 magnetometer (0x0C).
- I2C1 `Wire1` (SDA 17, SCL 18): PCA9685 (0x40). Channels per joint in `ServoParams` (`include/hardware.h`).
- On-board RGB LED: robot state (`RobotState` in `include/hardware.h`).
- Legs: `lf`, `rf`, `lh`, `rh`; joints `haa`, `hfe`, `kfe`. Commands arrive as 12 positions in that order, in radians.
- Servo mapping: 500-2500 us is 0-PI rad of commanded angle. Mechanical range used by the robot is 0-1.57 rad.
- The knee is driven through a linkage: `hfe` and `kfe` are coupled, the mechanism is only valid while `hfe - kfe`
  is inside a band (singular at its edges). `Leg::setCoupledLimit()` enforces it with `DIFF_MIN/MAX` in
  `include/config.h` (left-leg values, right legs mirrored: right = 1.6 - left, band with sign changed). Values from
  `test_leg_limits` on rf, matching the MuJoCo model.

## Structure

- `src/main.cpp`: boot sequence. Hardware check, network, servos to `INIT_POSE`, gyro calibration, tasks, ready.
- `src/utilities.cpp`: Wi-Fi (WiFiManager), bridge discovery by zenoh scouting, CDR (de)serialization.
- `src/zenoh.cpp`: session, subscribers (`joint_commands`, `imu/calibrate`, `robot/time`), publishers (`imu/data_raw`,
  `imu/mag`, `robot/ready`, `imu/calibration_result`).
- `src/tasks.cpp`: zenoh callbacks (commands, calibration request, clock sync from `robot/time`) and FreeRTOS tasks.
- `src/IMUMPU9250.cpp`: IMU driver, calibration and its storage in NVS (namespace `imu`).
- `test/`: PlatformIO tests built with the real `src/` code (`test_build_src = yes`; `main.cpp` is guarded by
  `PIO_UNIT_TESTING`).

| Task | Core | Priority | Rate | Job |
|------|------|----------|------|-----|
| `writeServosTask` | 1 | 3 | on command | Writes the last joint command to the PCA9685. |
| `readIMUTask` | 1 | 2 | 100 Hz | Reads the IMU, runs the calibration when requested, detects lost sensors and retries them (degraded = yellow LED). |
| `sendStatusTask` | 1 | 2 | on IMU sample | Publishes IMU, magnetometer (only valid samples) and `robot/ready` (2 Hz). |
| `watchdogTask` | 0 | 1 | 0.2 Hz | Reconnects the Wi-Fi. |
| `statusLedTask` | 0 | 1 | 10 Hz | Shows `RobotState` in the RGB LED (red / blue / green, blinking while waiting). |

## Rules

- Keep `src/` free of debug prints and commented-out experiments: checks go in `test/`.
- Keep changes minimal; the owner wants the code light and simple.
- Joint commands are only accepted in `STATE_READY`.
- Library versions are pinned in `platformio.ini`. Changing them re-downloads everything.
- The ROS 2 side lives in `../../ros2_ws` (bringup, robot_interface, routines). Topic names and message layouts must
  match between both. `INIT_POSE` must match the `initial_pose` parameter of `routine_player`.
- The bridge is `zenoh-bridge-ros2dds` 1.9.0 (apt), same version as zenoh-pico here. It needs `ROS_DISTRO=humble`
  in its environment and, in docker, `--network=host`.
