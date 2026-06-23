"""
Interactive MuJoCo viewer that runs a trained LoQuRo locomotion policy.

Usage:
    uv run play.py loco_v0
    uv run play.py loco_v0 --vx 0.5 --vy 0.0 --omega 0.2

Keys:
    Arrow Up/Down   : increase/decrease forward velocity (vx)
    Arrow Left/Right: increase/decrease yaw rate (omega)
    W / S           : increase/decrease lateral velocity (vy)
    R               : reset simulation
    Space           : zero all commands (stand still)
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort

from loquro_lab.quad_bot_constants import QUAD_JOINT_POS, get_quad_bot_cfg
from mjlab.entity.entity import Entity

# ── Constants matching training configuration ─────────────────────────────────
PHYSICS_DT = 0.002          # seconds (cfg.sim.mujoco.timestep)
DECIMATION = 10             # physics steps per control step
CONTROL_DT = PHYSICS_DT * DECIMATION  # 0.02 s → 50 Hz

ACTION_SCALE = 0.25         # cfg.actions["joint_pos"].scale

# Actuated joint names in ctrl / observation order (per-leg, natural XML order)
ACTUATED_JOINT_NAMES = [
    "fl_shoulder_joint", "fl_knee_joint", "fl_femur_joint",
    "fr_shoulder_joint", "fr_knee_joint", "fr_femur_joint",
    "rl_shoulder_joint", "rl_knee_joint", "rl_femur_joint",
    "rr_shoulder_joint", "rr_knee_joint", "rr_femur_joint",
]
DEFAULT_JOINT_POS = np.array(
    [QUAD_JOINT_POS[n] for n in ACTUATED_JOINT_NAMES], dtype=np.float32
)

# Velocity command step sizes for interactive key control
VX_STEP    = 0.05   # m/s
VY_STEP    = 0.05   # m/s
OMEGA_STEP = 0.1    # rad/s

# GLFW key codes used by the passive viewer
_KEY_UP    = 265
_KEY_DOWN  = 264
_KEY_LEFT  = 263
_KEY_RIGHT = 262
_KEY_W     = 87
_KEY_S     = 83
_KEY_R     = 82
_KEY_SPACE = 32


# ── Observation helpers ───────────────────────────────────────────────────────

def _build_index_arrays(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    """Return (qpos_ids, dof_ids) for the 12 actuated joints, in ctrl order."""
    qpos_ids = np.array(
        [int(model.joint(name).qposadr[0]) for name in ACTUATED_JOINT_NAMES],
        dtype=np.intp,
    )
    dof_ids = np.array(
        [int(model.joint(name).dofadr[0]) for name in ACTUATED_JOINT_NAMES],
        dtype=np.intp,
    )
    return qpos_ids, dof_ids


def build_obs(
    data: mujoco.MjData,
    qpos_ids: np.ndarray,
    dof_ids: np.ndarray,
    command: np.ndarray,
) -> np.ndarray:
    """Build the 33-dim policy observation vector (no noise, matching play mode)."""
    joint_pos = (data.qpos[qpos_ids] - DEFAULT_JOINT_POS).astype(np.float32)
    joint_vel = data.qvel[dof_ids].astype(np.float32)

    # sensordata layout: [0:3] imu_accel, [3:6] imu_ang_vel
    imu_accel  = np.clip(data.sensordata[0:3], -30.0, 30.0).astype(np.float32) * 0.1
    imu_angvel = np.clip(data.sensordata[3:6], -10.0, 10.0).astype(np.float32)

    obs = np.concatenate([joint_pos, joint_vel, imu_accel, imu_angvel, command])
    return obs.reshape(1, -1)  # (1, 33) for ONNX batch dim


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Play a trained LoQuRo policy.")
    parser.add_argument("model", help="Model name, e.g. loco_v0")
    parser.add_argument("--vx",    type=float, default=0.3,  help="Forward velocity command (m/s)")
    parser.add_argument("--vy",    type=float, default=0.0,  help="Lateral velocity command (m/s)")
    parser.add_argument("--omega", type=float, default=0.0,  help="Yaw rate command (rad/s)")
    args = parser.parse_args()

    model_path = Path(__file__).parent / "models" / f"{args.model}.onnx"
    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")

    # ── Build MuJoCo model identical to training ──────────────────────────────
    robot = Entity(get_quad_bot_cfg())
    spec = robot.spec

    gnd = spec.worldbody.add_geom()
    gnd.name = "ground"
    gnd.type = mujoco.mjtGeom.mjGEOM_PLANE
    gnd.size = [10.0, 10.0, 0.1]
    gnd.rgba = [0.84, 0.87, 0.82, 1.0]

    mj_model = spec.compile()
    mj_data  = mujoco.MjData(mj_model)

    mj_model.opt.timestep = PHYSICS_DT

    mujoco.mj_resetDataKeyframe(mj_model, mj_data, mj_model.key("init_state").id)
    mujoco.mj_forward(mj_model, mj_data)

    qpos_ids, dof_ids = _build_index_arrays(mj_model)

    # ── Load ONNX policy ──────────────────────────────────────────────────────
    sess = ort.InferenceSession(str(model_path))
    input_name  = sess.get_inputs()[0].name   # "obs"
    output_name = sess.get_outputs()[0].name  # "actions"

    print(f"Loaded policy: {model_path.name}")
    print(f"  obs dim={sess.get_inputs()[0].shape}  action dim={sess.get_outputs()[0].shape}")

    # ── Shared mutable command state (modified by key callback) ───────────────
    command = np.array([args.vx, args.vy, args.omega], dtype=np.float32)
    reset_flag = [False]

    def on_key(key: int) -> None:
        nonlocal command
        if key == _KEY_UP:
            command[0] = round(command[0] + VX_STEP, 3)
        elif key == _KEY_DOWN:
            command[0] = round(command[0] - VX_STEP, 3)
        elif key == _KEY_RIGHT:
            command[2] = round(command[2] - OMEGA_STEP, 3)
        elif key == _KEY_LEFT:
            command[2] = round(command[2] + OMEGA_STEP, 3)
        elif key == _KEY_W:
            command[1] = round(command[1] + VY_STEP, 3)
        elif key == _KEY_S:
            command[1] = round(command[1] - VY_STEP, 3)
        elif key == _KEY_SPACE:
            command[:] = 0.0
        elif key == _KEY_R:
            reset_flag[0] = True

    print()
    print("Viewer controls:")
    print("  Up/Down     : vx ±0.05 m/s")
    print("  Left/Right  : omega ±0.1 rad/s")
    print("  W / S       : vy ±0.05 m/s")
    print("  Space       : zero command")
    print("  R           : reset")
    print()

    # ── Simulation loop ───────────────────────────────────────────────────────
    last_action = np.zeros(12, dtype=np.float32)

    with mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=on_key
    ) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = mj_model.body("trunk").id
        viewer.cam.distance = 1.2
        viewer.cam.elevation = -20.0

        while viewer.is_running():
            step_start = time.perf_counter()

            if reset_flag[0]:
                mujoco.mj_resetDataKeyframe(mj_model, mj_data, mj_model.key("init_state").id)
                mujoco.mj_forward(mj_model, mj_data)
                last_action[:] = 0.0
                reset_flag[0] = False

            # Policy inference at control frequency (once per DECIMATION steps)
            obs = build_obs(mj_data, qpos_ids, dof_ids, command)
            raw_action = sess.run([output_name], {input_name: obs})[0][0]  # (12,)
            last_action = raw_action.astype(np.float32)

            # Convert policy action to joint position targets
            ctrl_targets = last_action * ACTION_SCALE + DEFAULT_JOINT_POS
            mj_data.ctrl[:] = ctrl_targets

            # Advance physics for one control step
            for _ in range(DECIMATION):
                mujoco.mj_step(mj_model, mj_data)

            viewer.sync()

            # Overlay HUD: current command
            cmd = command
            status = (
                f"vx={cmd[0]:+.2f} m/s  vy={cmd[1]:+.2f} m/s  ω={cmd[2]:+.2f} rad/s"
            )
            print(f"\r{status}   trunk_z={mj_data.qpos[2]:.3f} m   ", end="", flush=True)

            # Real-time pacing
            elapsed = time.perf_counter() - step_start
            remaining = CONTROL_DT - elapsed
            if remaining > 0:
                time.sleep(remaining)

    print()


if __name__ == "__main__":
    main()
