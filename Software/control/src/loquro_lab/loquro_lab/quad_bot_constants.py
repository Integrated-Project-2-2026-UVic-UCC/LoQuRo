from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

QUAD_XML: Path = Path(__file__).parent / "xmls" / "loquro.xml"

# Training spawn pose for the floating base.  MuJoCo quaternions are (w, x, y, z).
QUAD_BASE_POS = (0.0197255, 0.0146502, 0.145657)
QUAD_BASE_ROT = (0.996556, 0.049355, 0.00854287, 0.0660914)

# Stable qpos copied from MuJoCo GUI at ctrl=(0.7854, ..., 0.7854).  This must
# include passive joints because the closed-chain legs are not defined only by
# the 12 actuated encoder joints.
QUAD_JOINT_POS = {
    "fl_shoulder_joint": 0.78061,
    "fl_knee_joint": 0.77631,
    "fl_pieza1_pierna_joint": -0.780866,
    "fl_shoulder_passive_joint": 0.783369,
    "fl_tensor_aluminio_passive_joint": 0.014233,
    "fl_femur_joint": 0.79746,
    "fl_tibia_passive_joint": 0.0197625,
    "fr_shoulder_joint": 0.789562,
    "fr_knee_joint": 0.794936,
    "fr_pieza1_pierna_joint": -0.826888,
    "fr_shoulder_passive_joint": -0.827335,
    "fr_tensor_aluminio_passive_joint": 0.0561641,
    "fr_femur_joint": 0.772312,
    "fr_tibia_passive_joint": 0.0600942,
    "rl_shoulder_joint": 0.784472,
    "rl_knee_joint": 0.782318,
    "rl_pieza1_pierna_joint": -0.787276,
    "rl_shoulder_passive_joint": 0.792927,
    "rl_tensor_aluminio_passive_joint": -0.00358814,
    "rl_femur_joint": 0.7894,
    "rl_tibia_passive_joint": -0.00155224,
    "rr_shoulder_joint": 0.786294,
    "rr_knee_joint": 0.788958,
    "rr_pieza1_pierna_joint": -0.82063,
    "rr_shoulder_passive_joint": -0.81786,
    "rr_tensor_aluminio_passive_joint": 0.0381243,
    "rr_femur_joint": 0.780609,
    "rr_tibia_passive_joint": 0.0388843,
}

QUAD_INIT_QPOS = QUAD_BASE_POS + QUAD_BASE_ROT + tuple(QUAD_JOINT_POS.values())
QUAD_INIT_CTRL = (0.7854,) * 12

# Reset perturbation applied on top of the stable keyframe during training. Keep
# z nonnegative so the feet never spawn below the ground-contact stance.
QUAD_RESET_POSE_RANGE = {
    "x": (-0.03, 0.03),
    "y": (-0.03, 0.03),
    "z": (0.0, 0.02),
    "roll": (-0.03, 0.03),
    "pitch": (-0.03, 0.03),
    "yaw": (-0.10, 0.10),
}


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(QUAD_XML))
    key = spec.add_key(name="stable_stance", qpos=list(QUAD_INIT_QPOS))
    key.ctrl = list(QUAD_INIT_CTRL)
    return spec


# Feetech SCS20 (20 kg.cm = 1.96 Nm stall torque).
# stiffness/damping implement the on-board PD controller in simulation.
QUAD_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_shoulder_joint", ".*_knee_joint", ".*_femur_joint"),
    stiffness=50.0,
    damping=3.0,
    effort_limit=1.96,
    armature=0.01,
)

QUAD_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(QUAD_ACTUATOR_CFG,),
    soft_joint_pos_limit_factor=0.9,
)


def get_quad_bot_cfg() -> EntityCfg:
    return EntityCfg(
        spec_fn=get_spec,
        articulation=QUAD_ARTICULATION,
        init_state=EntityCfg.InitialStateCfg(
            pos=QUAD_BASE_POS,
            rot=QUAD_BASE_ROT,
            joint_pos=None,
            joint_vel={".*": 0.0},
        ),
    )


if __name__ == "__main__":
    import time

    import mujoco
    import mujoco.viewer
    import numpy as np
    from mjlab.entity.entity import Entity

    # ── Build the model exactly as training does ───────────────────────────
    # Entity.__init__ adds <position> actuators via BuiltinPositionActuatorCfg
    # and writes the init_state keyframe — the same steps the RL env performs.
    robot = Entity(get_quad_bot_cfg())
    spec = robot.spec

    # Add a ground plane for standalone viewing (the scene provides this during training).
    gnd = spec.worldbody.add_geom()
    gnd.name = "ground"
    gnd.type = mujoco.mjtGeom.mjGEOM_PLANE
    gnd.size = [5.0, 5.0, 0.1]
    gnd.rgba = [0.84, 0.87, 0.82, 1.0]

    model = spec.compile()
    data = mujoco.MjData(model)

    # mjlab sets ctrllimited=False so the PD controller can push against joint limits
    # without zeroing the force. The passive viewer shows [-1, 1] sliders in that case.
    # Override here so the debug viewer reflects the actual joint ranges.
    for i in range(model.nu):
        jnt_id = model.actuator_trnid[i, 0]
        model.actuator_ctrllimited[i] = 1
        model.actuator_ctrlrange[i, 0] = model.jnt_range[jnt_id, 0]
        model.actuator_ctrlrange[i, 1] = model.jnt_range[jnt_id, 1]

    # Apply the init_state keyframe (joints at 0.7 rad, actuator targets at 0.7 rad).
    mujoco.mj_resetDataKeyframe(model, data, model.key("init_state").id)
    mujoco.mj_forward(model, data)

    # Match the training physics timestep (0.002 s); decimation=10 keeps 50 Hz control.
    TRAINING_TIMESTEP = 0.002
    model.opt.timestep = TRAINING_TIMESTEP

    # ── Print model summary ────────────────────────────────────────────────
    act_names = [model.actuator(i).name for i in range(model.nu)]
    sen_names = [model.sensor(i).name for i in range(model.nsensor)]
    jnt_names = [model.joint(i).name for i in range(1, model.njnt)]  # skip freejoint

    print()
    print("=" * 60)
    print(" LoQuRo — model as used in RL training")
    print("=" * 60)
    print(f"  nq={model.nq}  nv={model.nv}  nu={model.nu}  nsensor={model.nsensor}")
    print(f"  Physics timestep : {model.opt.timestep*1000:.1f} ms  "
          f"(control @ {1/(model.opt.timestep*4):.0f} Hz with decimation=4)")
    print()
    print(f"  Actuated joints ({model.nu}):")
    lo, hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
    fl, fh = model.actuator_forcerange[:, 0], model.actuator_forcerange[:, 1]
    for i, name in enumerate(act_names):
        print(f"    [{i:2d}] {name:<30s}  ctrl=[{lo[i]:.3f}, {hi[i]:.3f}] rad  "
              f"effort=[{fl[i]:.2f}, {fh[i]:.2f}] Nm")
    print()
    print(f"  Sensors ({model.nsensor}):")
    for i, name in enumerate(sen_names):
        dim = model.sensor(i).dim[0]
        print(f"    [{i}] {name}  dim={dim}")
    print()
    print(f"  All joints ({len(jnt_names)}) — 12 actuated + 16 passive:")
    for name in jnt_names:
        tag = "(actuated)" if any(name == a for a in act_names) else "(passive)"
        print(f"    {name:<40s} {tag}")
    print()
    print(f"  Initial ctrl (rad): {np.round(data.ctrl, 3)}")
    print(f"  Initial trunk z   : {data.qpos[2]:.3f} m")
    print(f"  Initial trunk quat: {np.round(data.qpos[3:7], 3)}")
    print("=" * 60)
    print()

    # ── Launch passive viewer with real-time step loop ─────────────────────
    print("Launching viewer — close the window or press Ctrl+C to quit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.perf_counter()
            mujoco.mj_step(model, data)
            viewer.sync()
            # Sleep to maintain real-time pace (training timestep = 5 ms).
            elapsed = time.perf_counter() - step_start
            remaining = model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)
