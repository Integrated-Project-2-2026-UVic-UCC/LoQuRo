# 3. 💻 `./Software`

A folder meant to save all the softwares of the robot, from the firmware (low-level code) to the control software (high-level code and simulations).

**To make the robot move, follow [QUICKSTART.md](./QUICKSTART.md).**

## 3.1 Control
`./Software/control`:
MuJoCo model, `mjlab` training and trained ONNX policies (`loquro_lab`).

## 3.2 ROS 2
`./Software/ros2_ws`:
ROS 2 Humble workspace to use the real robot: launch file (Zenoh bridge, Madgwick filter, robot model), IMU
calibration action, routine player and robot description. See [ros2_ws/README.md](./ros2_ws/README.md).

## 3.3 Firmware
`./Software/firmware/embedded-leg-ctrl`:
PlatformIO project for the ESP32-S3: drives the 12 servos through a PCA9685 and reads the 9-DOF IMU (MPU9250).
The communication with ROS 2 is wireless with **Zenoh** (zenoh-pico) through `zenoh-bridge-ros2dds`. The robot finds
the bridge in the Wi-Fi network by itself, there is no IP to configure. The on-board RGB LED shows the state (green:
ready). See [the firmware README](./firmware/embedded-leg-ctrl/README.md).
