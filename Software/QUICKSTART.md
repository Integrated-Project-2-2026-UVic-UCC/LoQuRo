# LoQuRo quick start

How to make the robot move, step by step. You need a PC with ROS 2 Humble (Ubuntu 22.04 or a Humble docker container
started with `--network=host`) and the Wi-Fi router of the robot.

Connect the PC to the robot router **by Ethernet cable** (with the PC on Wi-Fi the legs move in jerks). If the cable
gets an address `169.254.x.x`, see step 3 of the [firmware README](./firmware/embedded-leg-ctrl/README.md#3-network-setup).

## 1. Only the first time: prepare the PC

Open a terminal in the ROS 2 Humble environment and copy these blocks one by one.

Install the bridge that connects ROS 2 with the robot:

```bash
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes --output /etc/apt/keyrings/zenoh-public-key.gpg
echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update
sudo apt install -y zenoh-bridge-ros2dds=1.9.0
sudo apt-mark hold zenoh-bridge-ros2dds
```

Build the LoQuRo packages (replace `<LoQuRo>` with the folder where you downloaded the repository):

```bash
cd <LoQuRo>/Software/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

If `sudo rosdep init` says it is already initialized, ignore it and continue.

## 2. Only the first time: give the Wi-Fi to the robot

1. Turn the robot on. The light blinks **blue**.
2. With your phone, connect to the Wi-Fi network called **LoQuRo**.
3. A page opens by itself. Choose your Wi-Fi, write its password and save.

The robot remembers it. You do not have to write any IP address.

## 3. Every time: start

1. Put the robot on the floor, lying down, and turn it on.
2. In the PC, in the ROS 2 terminal:

```bash
cd <LoQuRo>/Software/ros2_ws
source install/setup.bash
ros2 launch loquro_bringup robot.launch.py rviz:=true
```

3. Look at the light of the robot:

| Light | Meaning | What to do |
|-------|---------|------------|
| Blue, blinking | Looking for the Wi-Fi | Wait. If it never changes, repeat step 2. |
| Blue, solid | Looking for the PC | Check that the launch of the PC is running and both are in the same Wi-Fi. |
| Green, blinking | Standing up and calibrating | **Do not touch the robot** for a few seconds. |
| Green, solid | Ready | Go to step 4. |
| Yellow | Ready, but the motion sensor (IMU) fails | The robot moves anyway. |
| Red | The motors board does not answer | Turn it off and check the cables. |

The robot stands up by itself just before the green blinking. The PC terminal prints `Robot ready`.

## 4. Move the robot

Open **another** terminal (keep the first one running) and play a routine:

```bash
cd <LoQuRo>/Software/ros2_ws
source install/setup.bash
ros2 run loquro_control routine_player --ros-args -p routine:=squats
```

Other routine: `greeting`. Stop it with `Ctrl + C`.


## 5. Stop

Press `Ctrl + C` in the terminals and turn the robot off. Before starting a routine again, restart the robot so it
goes back to its initial pose.

## If something goes wrong

- **The light stays solid blue.** The robot does not find the PC. Check that the PC is connected to the robot router,
  that the launch is running and that you did not start another `zenoh-bridge-ros2dds` by hand. After changing the
  network of the PC, restart the launch. In docker, the container must use `--network=host`.
- **The legs move in jerks.** The PC is on Wi-Fi: use the Ethernet cable.
- **The light is green but the robot does not move.** Check that the first terminal printed `Robot ready` and that the
  routine is running in a second terminal with `source install/setup.bash` done.
- **The orientation in rviz drifts or is wrong.** Recalibrate the motion sensor with no routine running:

  ```bash
  ros2 action send_goal --feedback /calibrate_imu loquro_interfaces/action/CalibrateImu "{}"
  ```

  Light blinking slowly: leave the robot still on a flat surface. Light blinking fast: pick it up and rotate it in
  all directions for 20 seconds. Solid green: done, it is saved in the robot.

More detail: [ROS 2 workspace](./ros2_ws/README.md) and [firmware](./firmware/embedded-leg-ctrl/README.md).
