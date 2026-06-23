# loquro_lab

UV Python package containing the MuJoCo/mjlab RL environment and inference tooling for the **LoQuRo** custom quadruped robot.

## Setup

```bash
uv sync
```

## Training

From this package directory, sync the environment and check that the LoQuRo task is registered:

```bash
uv sync
uv run list-envs
```

The task list should include `Mjlab-Velocity-Flat-LoQuRo`.

Launch a training run with 4096 parallel environments:

```bash
uv run train Mjlab-Velocity-Flat-LoQuRo --env.scene.num-envs 4096
```

To use TensorBoard instead of the default W&B logger:

```bash
uv run train Mjlab-Velocity-Flat-LoQuRo --env.scene.num-envs 4096 --agent.logger tensorboard
```

Trained checkpoints are saved under `logs/rsl_rl/quad_bot_velocity/<run-timestamp>/`. At the end of training the final policy is exported to ONNX. Copy the `.onnx` file to `models/` and give it a name:

```bash
cp logs/rsl_rl/quad_bot_velocity/<run>/final.onnx models/loco_v0.onnx
```

## Running a trained policy (`play.py`)

`play.py` opens an interactive MuJoCo viewer and runs the robot under a named ONNX policy in real time.

```bash
uv run play.py <model_name> [options]
```

`<model_name>` is the filename (without `.onnx`) of a model stored in the `models/` directory.

### Examples

```bash
# Run loco_v0 with the default forward command (vx=0.3 m/s)
uv run play.py loco_v0

# Custom velocity command
uv run play.py loco_v0 --vx 0.5 --vy 0.0 --omega 0.3

# Stand still and use keyboard to drive
uv run play.py loco_v0 --vx 0.0
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--vx` | `0.3` | Forward velocity command (m/s) |
| `--vy` | `0.0` | Lateral velocity command (m/s) |
| `--omega` | `0.0` | Yaw rate command (rad/s) |

### Viewer keyboard controls

| Key | Action |
|-----|--------|
| `↑` / `↓` | Increase / decrease forward velocity (vx) by 0.05 m/s |
| `←` / `→` | Increase / decrease yaw rate (omega) by 0.1 rad/s |
| `W` / `S` | Increase / decrease lateral velocity (vy) by 0.05 m/s |
| `Space` | Zero all commands (robot stands still) |
| `R` | Reset simulation to initial stance |

The terminal prints the current command and trunk height on every control step so you can monitor the policy without switching windows.

## Robot overview

- **Legs:** 4-bar linkage, 4 legs — FL / FR / RL / RR
- **Actuated joints:** 12 (shoulder, knee, femur per leg) — Feetech SCS20 servos (1.96 Nm)
- **Sensors used by policy:** IMU accelerometer + gyroscope, 12 joint encoders
- **Control frequency:** 50 Hz (physics at 500 Hz, decimation = 10)
- **Model file:** `loquro_lab/xmls/loquro.xml`
