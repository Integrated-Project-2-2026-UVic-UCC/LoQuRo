# LoQuRo firmware (ESP32-S3)

Firmware of the robot: drives the 12 servos through a PCA9685, reads the IMU (MPU9250 + AK8963 magnetometer) and
talks to ROS 2 over Wi-Fi with Zenoh (zenoh-pico, through `zenoh-bridge-ros2dds` on the PC). The ROS 2 side is in
[`Software/ros2_ws`](../../ros2_ws/README.md).

Follow the steps in order the first time.

---

## 1. Requirements

- PlatformIO (VS Code extension or CLI, `~/.platformio/penv/bin/pio`).
- `cmake` installed on the PC (zenoh-pico needs it to build): `sudo apt install cmake`.
- Permission to use the USB port of the board: `sudo usermod -aG dialout $USER`, then **restart the PC** (logging out
  is often not enough for VS Code).
- A Wi-Fi router dedicated to the robot (2.4 GHz, the ESP32 has no 5 GHz). It does not need internet.

## 2. Build and flash

Connect the board by USB and, from this folder:

```bash
pio run -t upload
```

Library versions are pinned in `platformio.ini` (zenoh-pico **1.9.0**, the same version as the bridge on the PC).
Do not change them unless you change the bridge too.

Before uploading, close any open serial monitor: the port can only be used by one program.

> When the robot connects to the PC, the legs move to the initial pose. Have it lying down or held.

## 3. Network setup

These points made the difference between legs moving smoothly and legs moving in jerks:

1. **Connect the PC to the robot router by Ethernet cable.** With the PC on Wi-Fi, its link stalled for 150–200 ms
   from time to time (measured with ping to the router), the commands arrived in bursts and the legs jumped. The robot
   itself on Wi-Fi is fine.
2. **Use a different subnet for the robot router** (e.g. router at `192.168.10.1`) than any other network the PC uses.
   If both networks are `192.168.0.x`, the PC does not know where to send the robot packets.
3. **The Ethernet connection of the PC must get its IP by DHCP.** If `ip -br addr show enp2s0` shows `169.254.x.x`, the
   connection is in link-local mode: `nmcli connection modify <connection> ipv4.method auto ipv4.never-default yes`.
   `never-default` keeps internet on the other network (the robot router has none).
4. If you must use Wi-Fi on the PC, disable its power saving for that network:
   `nmcli connection modify <robot network> 802-11-wireless.powersave 2`.
5. After any network change on the PC, restart the ROS launch (the bridge keeps the IPs it had when it started).

To check the network, from the PC (`192.168.0.100` = IP of the robot, shown in its serial port):

```bash
ping -c 300 -i 0.02 <robot IP> | tail -2
```

Good: 0 % loss and a maximum of a few tens of ms. A maximum of 100–200 ms means jerky legs.

## 4. Give the Wi-Fi to the robot (first time only)

1. Turn the robot on. The LED blinks **blue** and the robot opens a Wi-Fi network called **LoQuRo**.
2. Connect a phone to **LoQuRo**. A page opens by itself (if not, reconnect and wait a few seconds).
3. Choose the robot router network, write its password and save.

The robot remembers it. If the network is not available it keeps retrying and re-opens the portal. There is no IP to
configure: the robot finds the bridge by itself (Zenoh scouting) when both are in the same network.

## 5. Start the robot

1. Start ROS on the PC: `ros2 launch loquro_bringup robot.launch.py rviz:=true` (see the ROS README).
2. Turn the robot on and watch the LED:

| LED | State | What to do |
|-----|-------|------------|
| Red | The servo driver (PCA9685) does not answer | Turn off, check the wiring. |
| Blue, blinking | Connecting to the Wi-Fi, or the **LoQuRo** portal is open | Wait, or do step 4. |
| Blue | Wi-Fi ok, searching the bridge | Check that the launch is running and the PC is in the robot network. |
| Green, blinking | Going to the initial pose and calibrating the gyroscope | **Do not touch the robot.** |
| Green, blinking fast | Calibrating the magnetometer (only when requested) | Rotate the robot in all directions. |
| Green | Ready, joint commands are accepted | Run a routine from ROS. |
| Yellow | Ready, but the IMU or the magnetometer fails | The robot moves normally. |

What happens at startup:

1. The robot connects to the Wi-Fi and to the bridge. The servos are not driven yet.
2. All joints go to the initial pose (`INIT_POSE` in `include/config.h`). The robot waits until it is at rest and
   calibrates the gyroscope for 2 s.
3. Green LED: it accepts joint commands. It publishes the IMU once it has received the PC clock from ROS.

If the Wi-Fi is lost the LED turns blue and the legs hold their position until the connection is back.

## 6. Calibrate the IMU (first time, and when the orientation looks wrong)

The calibration is requested from ROS, it is stored in the robot (non-volatile memory) and loaded on every startup.
Only the gyroscope is recalibrated at every startup (2 s at rest).

Requirements: green LED, and no joint command in the last 2 s (stop any routine). Otherwise it is rejected.

```bash
ros2 action send_goal --feedback /calibrate_imu loquro_interfaces/action/CalibrateImu "{}"
```

1. LED blinking green slowly: leave the robot still on a flat surface.
2. LED blinking green fast: pick it up and rotate it slowly in all directions for 20 s.
3. Solid green: done. The action says what was calibrated. If the robot was not rotated enough, the magnetometer
   keeps its previous calibration.

To check it:

- Serial port at startup: `IMU: Saved calibration loaded` and the values in use (`No saved calibration` if there is
  none). With the robot flat, the accelerometer z offset must be close to 0.
- ROS, robot still and flat: `ros2 topic echo /imu/data_raw --field linear_acceleration` gives x ≈ 0, y ≈ 0,
  z ≈ −9.8 (the IMU is mounted with z down).
- ROS, rotating the robot: the magnitude of `/imu/mag` stays roughly constant (25–65 µT).

If the IMU fails, the LED turns yellow and the robot keeps working: without magnetometer only `/imu/mag` stops,
without accelerometer / gyroscope no IMU topic is published. It retries every 2 s and goes back to green.

## 7. Hardware tests

Checks that run on the robot (`test/` folder). Run them in this order when something does not work:

| Test | What it checks | Moves the robot |
|------|----------------|-----------------|
| `test_imu` | IMU and magnetometer answer, readings at rest, read time, loaded calibration. | No |
| `test_network` | Saved Wi-Fi, bridge found, Zenoh session. Needs the launch running. | No |
| `test_command_timing` | How regularly the commands arrive (bursts = network problem). Needs the launch and a routine. | No |
| `test_legs` | Moves each joint of the selected legs through its range. | **Yes** |
| `test_leg_limits` | Interactive tool to measure the knee linkage limits (step 8). | **Yes** |

The PlatformIO **Test** button runs `test_imu` and `test_network`. One test:

```bash
pio test -e esp32-s3-devkitc-1 -f test_legs
```

The interactive ones (`test_command_timing`, `test_leg_limits`) are uploaded and then read with the monitor:

```bash
pio test -e esp32-s3-devkitc-1 -f test_command_timing --without-testing
pio device monitor
```

`test_command_timing` with a routine at 50 Hz should show almost every interval in `15-25` ms. Many `<5ms` with gaps of
`>100ms` means the network delivers the commands in bursts (step 3).

After any test, upload the normal firmware again (step 2). The legs to move in `test_legs` and the sequential /
simultaneous mode are set at the top of `test/test_legs/test_legs.cpp`.

## 8. Leg limits (knee linkage)

The knee servo moves the knee through a linkage, so `hfe` and `kfe` are coupled: the mechanism only works while
`hfe − kfe` stays inside a band. At the edges the linkage reaches its dead point and the servo stalls (high current).
So the limit is not a range per joint, it is:

- each joint in 0–1.57 rad, and
- `DIFF_MIN ≤ hfe − kfe ≤ DIFF_MAX` (`haa` is independent).

`Leg::setCoupledLimit()` applies it to every command, moving a command outside the band to the nearest valid point.
The values in `include/config.h` are those of a left leg; the right legs are mirrored (right = 1.6 − left) and use the
band with the sign changed.

Current values −0.84 … +0.81: measured on `rf` (−0.94 … +0.91), which matches the MuJoCo model (−0.95 … +0.90), minus
0.1 rad of margin. The "up" pose of the old routines (`hfe − kfe = ±1.07`) was past the dead point; it is now clipped.

To measure again (e.g. after a mechanical change), with the robot lifted and the legs in the air:

```bash
pio test -e esp32-s3-devkitc-1 -f test_leg_limits --without-testing
pio device monitor
```

Type a leg (`lf`, `rf`, `lh`, `rh`) and Enter. For several values of `hfe`, it moves `kfe` very slowly to each side:
press any key just before the tensor gets aligned with the tibia lever (`q` aborts). It prints the band in the left
convention and the values for `config.h`. Do not stop too early: a band much narrower than the reference means the key
was pressed before the real limit.

---

## Reference

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `joint_commands` | `sensor_msgs/JointState` | to the robot. 12 positions [rad], order `lf rf lh rh` × `haa hfe kfe`. |
| `imu/data_raw` | `sensor_msgs/Imu` | from the robot, 100 Hz, frame `imu_link` |
| `imu/mag` | `sensor_msgs/MagneticField` | from the robot, only valid samples |
| `robot/ready` | `std_msgs/Bool` | from the robot, 2 Hz |
| `robot/time` | `builtin_interfaces/Time` | to the robot, PC clock (1 Hz) |
| `imu/calibrate` / `imu/calibration_result` | `std_msgs/Empty` / `std_msgs/UInt8` | calibration request / result |

The robot only applies the newest joint command (it never queues old ones). Commands are accepted only with the green
LED.

### Timestamps

The robot has no internet. It sets its clock with the PC time that `robot_interface` publishes in `robot/time` and then
corrects it smoothly. It does not publish the IMU until it has that time, so the messages never carry 1970 dates.

### Configuration (`include/config.h`)

| Constant | Meaning |
|----------|---------|
| `INIT_POSE` | Pose of every leg when the robot connects. Must match `initial_pose` of `routine_player` in ROS. |
| `DIFF_MIN`, `DIFF_MAX` | Knee linkage band (step 8). |
| `*_KEYEXPR` | Topic names. Must match the ROS side and the bridge configuration. |
| `LED_BRIGHTNESS` | Brightness of the status LED. |

### Hardware

- `Wire` (SDA 8, SCL 9): MPU9250 (0x68) and AK8963 (0x0C).
- `Wire1` (SDA 17, SCL 18): PCA9685 (0x40). Channel of every joint in `ServoParams` (`include/hardware.h`).
- Servos: 500–2500 µs is 0–π rad of command.
- Status LED: on-board RGB LED.
