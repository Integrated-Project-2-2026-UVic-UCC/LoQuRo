from pathlib import Path
import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

# Ruta del modelo
QUAD_XML: Path = Path(__file__).parent / "xmls" / "loquro.xml"

def get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(QUAD_XML))

# Configuración de los servos Feetech 20kg.cm
QUAD_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_shoulder_joint", ".*_knee_joint", ".*_femur_joint"),
    stiffness=50.0,
    damping=3.0,
    effort_limit=1.96,
    armature=0.01,
)

QUAD_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(QUAD_ACTUATOR_CFG,),
)

def get_quad_bot_cfg() -> EntityCfg:
    return EntityCfg(
        spec_fn=get_spec,
        articulation=QUAD_ARTICULATION,
        init_state=EntityCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.3),
            joint_pos={
                # El XML ya nace en 0 gracias a los offsets aplicados
                ".*_shoulder_joint": 0.7,
                ".*_knee_joint": 0.7,
                ".*_femur_joint": 0.7,
            },
        ),
    )

# Cálculo dinámico de la escala de acciones (Estilo ANYmal C)
QUAD_BOT_ACTION_SCALE: dict[str, float] = {}
for _a in QUAD_ARTICULATION.actuators:
    _e = _a.effort_limit
    _s = _a.stiffness
    if _e is not None:
        for _n in _a.target_names_expr:
            QUAD_BOT_ACTION_SCALE[_n] = 0.25 * _e / _s