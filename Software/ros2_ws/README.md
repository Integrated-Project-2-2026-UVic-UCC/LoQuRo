# LoQuRo ROS 2 workspace (Humble)

ROS 2 side of the LoQuRo quadruped. The robot (ESP32-S3, [`../firmware/embedded-leg-ctrl`](../firmware/embedded-leg-ctrl/README.md))
talks to ROS 2 over Wi-Fi through `zenoh-bridge-ros2dds`.

Follow the steps in order the first time. Everything runs inside a ROS 2 Humble environment (Ubuntu 22.04 or a Humble
docker container started with `--network=host`).

---

## 1. Install (first time only)

### Bridge

The bridge has no rosdep key, it is installed with apt from the Zenoh repository. Use version **1.9.0**: it is the
same version as zenoh-pico in the firmware, and the newer packages (1.10) need Ubuntu 24.04.

```bash
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update
sudo apt install -y zenoh-bridge-ros2dds=1.9.0
sudo apt-mark hold zenoh-bridge-ros2dds
```

If the Zenoh repository is already in `/etc/apt/sources.list`, skip the first two lines (apt fails with duplicated
sources).

### Workspace

```bash
cd <LoQuRo>/Software/ros2_ws
sudo rosdep init        # if it says it is already initialized, continue
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

To avoid sourcing it in every new terminal:

```bash
echo "source <LoQuRo>/Software/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 2. Network

Connect the PC to the robot router **by Ethernet cable** and keep the PC off other networks with the same addresses.
With the PC on Wi-Fi the commands arrived in bursts and the legs moved in jerks. Details and checks in step 3 of the
[firmware README](../firmware/embedded-leg-ctrl/README.md#3-network-setup).

## 3. Run

**Terminal 1**: everything needed to use the robot (bridge, robot interface, IMU filter, robot model, rviz):

```bash
ros2 launch loquro_bringup robot.launch.py rviz:=true
```

Turn the robot on and wait for the solid green LED. The terminal prints `Robot ready`.

**Terminal 2**: what you want the robot to do, for example a routine:

```bash
ros2 run loquro_control routine_player --ros-args -p routine:=greeting
```

Available routines: `squats`, `greeting`. Stop with `Ctrl + C`. The routine goes slowly from the initial pose to its
first pose, waits 2 s and starts.

Before starting a routine again, restart the robot so it goes back to its initial pose (the routine assumes the robot
starts there).

## 4. Calibrate the IMU

First time, and when the orientation in rviz looks wrong. Stop any routine first (no joint command in the last 2 s):

```bash
ros2 action send_goal --feedback /calibrate_imu loquro_interfaces/action/CalibrateImu "{}"
```

1. LED blinking green slowly: leave the robot still on a flat surface (about 3 s).
2. LED blinking green fast: pick it up and rotate it in all directions for 20 s.
3. Solid green: done. The action returns what was calibrated, or why it failed.

The calibration is stored in the robot and loaded on every startup. To check it, with the robot still and flat:

```bash
ros2 topic echo /imu/data_raw --field linear_acceleration   # x ≈ 0, y ≈ 0, z ≈ -9.8 (IMU mounted with z down)
```

## 5. Write a routine

Add `src/loquro_control/routines/<name>.yaml` and play it with `-p routine:=<name>` (it is installed with
`colcon build`; with `--symlink-install` new files need one build):

```yaml
speed: 0.4      # rad/s, maximum speed of every joint (default of every step)
loop: true
leg_poses:      # optional, [haa, hfe, kfe] of one leg
  left_down:  [0.7, 1.5608, 0.8196]
  right_down: [0.7, 0.0392, 0.7804]
poses:          # 12 positions, or one leg pose per leg
  down: {lf: left_down, rf: right_down, lh: left_down, rh: right_down}
  up:   {lf: [0.7, 0.4, 1.2], rf: [0.7, 1.2, 0.4], lh: [0.7, 0.4, 1.2], rh: [0.7, 1.2, 0.4]}
sequence:
  - pose: down
  - {pose: up, speed: 1.6, hold: 0.5}   # hold: seconds to wait at the pose
```

Rules for the poses:

- Joint order of every leg: `[haa, hfe, kfe]` in rad, each one in 0–1.57.
- The right legs are mirrored: the same pose is `1.6 − value` of the left leg (except `haa`).
- Knee linkage: `hfe − kfe` must be between −0.84 and +0.81 on the left legs (sign changed on the right legs).
  Outside that band the firmware clips the command (see the firmware README, step 8).

If the robot stops being ready the routine pauses; if it is gone for more than 5 s the routine assumes the robot
restarted and starts again.

---

## Troubleshooting

| Problem | Cause and fix |
|---------|---------------|
| `Robot ready` never appears, LED solid blue | The robot does not find the bridge. The PC must be in the robot network and the launch running. Do not start `zenoh-bridge-ros2dds` by hand: the launch starts it with the right configuration, and two bridges cannot share the port. |
| Bridge errors `ddsi_udp_conn_write ... failed` | The PC changed network with the launch running. Restart the launch. |
| `The passed action type is invalid` | `source install/setup.bash` missing in that terminal. |
| `Waiting for an action server` | The launch (`robot_interface`) is not running. |
| Topics exist but the IMU publishes nothing | The robot only publishes the IMU after receiving the PC clock from `robot_interface` (launch running). |
| Legs move in jerks | The network delivers commands in bursts. Use the Ethernet cable (step 2). Check with ping and with the firmware test `test_command_timing`. |
| Calibration fails with "receiving joint commands" | A routine or policy is running. Stop it and wait 2 s. |

Things learned about the bridge, in case you run it yourself:

- It needs `ROS_DISTRO=humble` in its environment (set after sourcing ROS). Without it it assumes a newer ROS, does
  not understand the Humble nodes and no topic is routed.
- In docker it needs `--network=host`: the robot finds it by multicast.
- It only creates a topic when a ROS node uses it. `robot_interface` keeps every robot topic declared so they always
  exist.
- With RELIABLE publishers the bridge blocks when packets are lost and old commands arrive late. `/joint_commands`
  publishers must be BEST_EFFORT, depth 1 (`COMMAND_QOS` in `loquro_control/common.py`).

---

## Reference

### Packages

| Package | Contents |
|---------|----------|
| `loquro_bringup` | `robot.launch.py`, bridge and Madgwick configuration. |
| `loquro_control` | Nodes `robot_interface`, `routine_player`, `policy_player` (placeholder) and the routines. |
| `loquro_description` | URDF (visualization and TF) and meshes, generated from the MuJoCo model. |
| `loquro_interfaces` | `CalibrateImu` action. |
| `imu_filter_madgwick` | Copy of the Madgwick filter that keeps working without magnetometer (`LOQURO_CHANGES.md`). |

### Launch arguments (`robot.launch.py`)

| Argument | Default | Meaning |
|----------|---------|---------|
| `rviz` | `false` | Open rviz with the robot model and the IMU orientation. |
| `bridge` | `true` | Start the bridge. `false` if it already runs in another machine. |
| `routine` | empty | Also start `routine_player` with that routine. |
| `policy` | empty | Path of an ONNX model for `policy_player` (placeholder, does not move the robot). |

Only the model, without robot: `ros2 launch loquro_description display.launch.py`.

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/joint_commands` | `sensor_msgs/JointState` | ROS → robot. 12 positions [rad], order `lf rf lh rh` × `haa hfe kfe`. BEST_EFFORT. |
| `/imu/data_raw` | `sensor_msgs/Imu` | robot → ROS, 100 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | robot → ROS, while the magnetometer works |
| `/imu/data` | `sensor_msgs/Imu` | Madgwick orientation (falls back to accel + gyro if the magnetometer is lost) |
| `/robot/ready` | `std_msgs/Bool` | robot → ROS, 2 Hz. True when commands are accepted. |
| `/robot/time` | `builtin_interfaces/Time` | ROS → robot, 1 Hz. PC clock for the robot timestamps. |
| `/imu/calibrate`, `/imu/calibration_result` | `std_msgs/Empty`, `std_msgs/UInt8` | used by `/calibrate_imu` |

### Robot model

`loquro_description` is generated from `../control/loquro_lab/loquro_lab/xmls/loquro.xml`, everything fixed at the
standing pose. When the MuJoCo model changes:

```bash
pip install mujoco numpy
python3 src/loquro_description/scripts/mjcf_to_urdf.py ../control/loquro_lab/loquro_lab/xmls/loquro.xml
```

The IMU position is set in `loquro_description/urdf/loquro.urdf.xacro` (`imu_x`, `imu_y`, `imu_z`).
