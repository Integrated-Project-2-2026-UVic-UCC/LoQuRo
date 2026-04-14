from pathlib import Path
import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

QUAD_XML: Path = Path(__file__).parent / "xmls" / "robot.xml"

def get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(QUAD_XML))

def get_quad_bot_cfg() -> EntityCfg:
    return EntityCfg(
        spec_fn=get_spec,
        articulation=EntityArticulationInfoCfg(
            actuators=(
                BuiltinPositionActuatorCfg(
                    target_names_expr=(".*_hip_roll", ".*_hip_pitch", ".*_knee"),
                    stiffness=50.0,
                    damping=3.0,
                    effort_limit=1.96,
                    armature=0.01,
                ),
            ),
        ),
        init_state=EntityCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.3),
            joint_pos={
                ".*_hip_roll": 0.0,
                ".*_hip_pitch": 0.7,
                ".*_knee": -1.4,
            },
        ),
    )