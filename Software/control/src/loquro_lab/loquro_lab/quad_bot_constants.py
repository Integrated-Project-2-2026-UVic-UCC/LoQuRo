from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

QUAD_XML: Path = Path(__file__).parent / "xmls" / "loquro.xml"


def get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(QUAD_XML))


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
            pos=(0.0, 0.0, 0.30),
            joint_pos={
                ".*_shoulder_joint": 0.7854,
                ".*_knee_joint": 0.7854,
                ".*_femur_joint": 0.7854,
            },
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
