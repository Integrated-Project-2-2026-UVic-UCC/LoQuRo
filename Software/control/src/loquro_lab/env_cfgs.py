from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from .quad_bot.quad_bot_constants import get_quad_bot_cfg

def quad_bot_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    """Configuración limpia para evitar el error de mismatch de tensores."""
    cfg = make_velocity_env_cfg()

    # 1. Asignar el robot
    cfg.scene.entities = {"robot": get_quad_bot_cfg()}

    # 2. Definir nombres (Tus nombres del XML)
    site_names = ("FL", "FR", "RL", "RR")
    geom_names = ("FL_foot_collision", "FR_foot_collision", "RL_foot_collision", "RR_foot_collision")

    # 3. Sensores (Formato ANYmal C)
    cfg.scene.sensors = (
        ContactSensorCfg(
            name="feet_ground_contact",
            primary=ContactMatch(mode="geom", pattern=geom_names, entity="robot"),
            secondary=ContactMatch(mode="body", pattern="terrain"),
            fields=("found", "force"),
            reduce="netforce",
            track_air_time=True,
        ),
        ContactSensorCfg(
            name="nonfoot_ground_touch",
            primary=ContactMatch(
                mode="geom", entity="robot",
                pattern=r".*_collision\d*$",
                exclude=tuple(geom_names),
            ),
            secondary=ContactMatch(mode="body", pattern="terrain"),
            fields=("found",),
        ),
    )

    # 4. LIMPIEZA AGRESIVA DE RECOMPENSAS (Rewards)
    # Eliminamos las que causan el error de tensores (12 vs 0)
    # y las que buscan nombres de sitios o joints del Go1
    problematic_rewards = ["pose", "foot_clearance", "foot_swing_height", "foot_slip", "joint_pos", "air_time"]
    for r in problematic_rewards:
        if r in cfg.rewards:
            cfg.rewards.pop(r)

    # 5. CONFIGURAR RECOMPENSAS BÁSICAS (Las que NO fallan)
    cfg.rewards["track_linear_velocity"].weight = 2.0
    cfg.rewards["track_angular_velocity"].weight = 1.0
    cfg.rewards["upright"].weight = 0.5
    
    # Re-añadimos el air_time con tu sensor de pies
    cfg.rewards["air_time"] = cfg.rewards.get("air_time")
    if cfg.rewards["air_time"]:
        cfg.rewards["air_time"].params["sensor_names"] = ["feet_ground_contact"]

    # 6. ACCIONES (Mapeo de motores)
    cfg.actions["joint_pos"].scale = {
        ".*_hip_roll": 0.25,
        ".*_hip_pitch": 0.25,
        ".*_knee": 0.25,
    }

    # 7. TERMINACIONES (Muerte del robot)
    cfg.terminations["illegal_contact"] = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={"sensor_name": "nonfoot_ground_touch"},
    )

    # 8. AJUSTES DE ESCENA
    cfg.viewer.body_name = "trunk"
    cfg.scene.entities["robot"].init_state.pos = (0.0, 0.0, 0.3)

    if play:
        cfg.observations["policy"].enable_corruption = False
        cfg.scene.num_envs = 1

    return cfg