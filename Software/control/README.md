# LoQuRo control

Simulation and training of the **LoQuRo** quadruped.

- `loquro_lab/`: Python package managed with [uv](https://github.com/astral-sh/uv). MuJoCo model (MJCF and meshes),
  mjlab reinforcement learning environment, trained ONNX policies and `play.py` to run them in the simulator.
  See [loquro_lab/README.md](./loquro_lab/README.md).

The ROS 2 packages to use the real robot (bringup, routines, robot model) are in [`../ros2_ws`](../ros2_ws/README.md).
The routines that were here as scripts (`LoQuRo_squats.py`, `LoQuRo_greeting.py`, `LoQuRo_walk.py`) are now YAML
files played by `routine_player` (`walk` was the same as `squats`).
