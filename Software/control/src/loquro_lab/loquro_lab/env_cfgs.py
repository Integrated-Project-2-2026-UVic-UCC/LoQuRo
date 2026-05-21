# from mjlab.envs import ManagerBasedRlEnvCfg
# from mjlab.managers.termination_manager import TerminationTermCfg
# from mjlab.sensor import ContactMatch, ContactSensorCfg
# from mjlab.tasks.velocity import mdp
# from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

# from .quad_bot_constants import get_quad_bot_cfg, QUAD_BOT_ACTION_SCALE

# def quad_bot_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
#     cfg = make_velocity_env_cfg()
    
#     cfg.scene.entities = {"robot": get_quad_bot_cfg()}

#     # 1. SENSORES DE CONTACTO
#     geom_names = ("fl_foot_collision", "fr_foot_collision", "rl_foot_collision", "rr_foot_collision")
#     cfg.scene.sensors = (
#         ContactSensorCfg(
#             name="feet_ground_contact",
#             primary=ContactMatch(mode="geom", pattern=geom_names, entity="robot"),
#             secondary=ContactMatch(mode="body", pattern="terrain"),
#             fields=("found", "force"),
#             reduce="netforce",
#             track_air_time=True,
#         ),
#         ContactSensorCfg(
#             name="nonfoot_ground_touch",
#             primary=ContactMatch(
#                 mode="geom", entity="robot",
#                 pattern=r".*_collision\d*$",
#                 exclude=tuple(geom_names),
#             ),
#             secondary=ContactMatch(mode="body", pattern="terrain"),
#             fields=("found",),
#         ),
#     )

#     # 2. LIMPIEZA DE OBSERVACIONES (Evita KeyError de sensores de escaneo)
#     # Eliminamos todo lo que dependa de sensores RayCast (láser) que no tenemos
#     for group in ["actor", "critic"]:
#         if group in cfg.observations:
#             for term in ["height_scan", "foot_height"]:
#                 if term in cfg.observations[group].terms:
#                     cfg.observations[group].terms.pop(term)

#     # 3. LIMPIEZA DE RECOMPENSAS
#     problematic_rewards = ["foot_clearance", "foot_swing_height", "foot_slip", "joint_pos", "air_time", "pose"]
#     for r in problematic_rewards:
#         if r in cfg.rewards:
#             cfg.rewards.pop(r)

#     # 4. RECOMPENSAS BÁSICAS
#     cfg.rewards["track_linear_velocity"].weight = 2.0
#     cfg.rewards["track_angular_velocity"].weight = 1.0
#     cfg.rewards["upright"].weight = 0.5
#     if "feet_air_time" in cfg.rewards:
#         cfg.rewards["feet_air_time"].params["sensor_names"] = ["feet_ground_contact"]

#     # 5. CONFIGURACIÓN DE CONTROL Y ESCENA
#     cfg.actions["joint_pos"].scale = QUAD_BOT_ACTION_SCALE
#     cfg.terminations["illegal_contact"] = TerminationTermCfg(
#         func=mdp.illegal_contact,
#         params={"sensor_name": "nonfoot_ground_touch"},
#     )
#     cfg.events.pop("push_robot", None)
#     cfg.viewer.body_name = "trunk"
#     cfg.scene.entities["robot"].init_state.pos = (0.0, 0.0, 0.3)

    
#     # 5. FORZAR SUELO PLANO Y MATAR EL CURRICULUM (Arregla el AssertionError)
#     cfg.scene.terrain.terrain_type = "plane"
#     cfg.scene.terrain.terrain_generator = None
    
#     # Esta es la línea clave para que no busque niveles de terreno
#     if hasattr(cfg.curriculum, "terms"):
#         if "terrain_levels" in cfg.curriculum.terms:
#             cfg.curriculum.terms.pop("terrain_levels")

#     if play:
#         cfg.observations["actor"].enable_corruption = False
#         cfg.scene.num_envs = 1

#     return cfg



from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from .quad_bot_constants import get_quad_bot_cfg, QUAD_BOT_ACTION_SCALE

def quad_bot_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    cfg = make_velocity_env_cfg()
    
    cfg.scene.entities = {"robot": get_quad_bot_cfg()}

    # 1. SENSORES DE CONTACTO
    geom_names = ("fl_foot_collision", "fr_foot_collision", "rl_foot_collision", "rr_foot_collision")
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

    # 2. LIMPIEZA DE OBSERVACIONES (Eliminamos sensores RayCast)
    for group in ["actor", "critic"]:
        if group in cfg.observations:
            for term in ["height_scan", "foot_height"]:
                if term in cfg.observations[group].terms:
                    cfg.observations[group].terms.pop(term)

    # 3. LIMPIEZA DE RECOMPENSAS (Evitamos el error 28 vs 12 de tensores)
    problematic_rewards = ["foot_clearance", "foot_swing_height", "foot_slip", "joint_pos", "air_time", "pose", "dof_pos_limits"]
    for r in problematic_rewards:
        if r in cfg.rewards:
            cfg.rewards.pop(r)

    # 4. RECOMPENSAS BÁSICAS
    cfg.rewards["track_linear_velocity"].weight = 2.0
    cfg.rewards["track_angular_velocity"].weight = 1.0
    cfg.rewards["upright"].weight = 0.5
    if "feet_air_time" in cfg.rewards:
        cfg.rewards["feet_air_time"].params["sensor_names"] = ["feet_ground_contact"]

    # 5. ARREGLO DEL ASSERTIONERROR (Currículo de Terreno)
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None
    
    # Borramos 'terrain_levels' de todas las formas posibles según la versión
    if hasattr(cfg, "curriculum"):
        # Si es un diccionario
        if isinstance(cfg.curriculum, dict):
            cfg.curriculum.pop("terrain_levels", None)
        # Si tiene el atributo .terms (lo más probable por tu log)
        elif hasattr(cfg.curriculum, "terms"):
            if "terrain_levels" in cfg.curriculum.terms:
                cfg.curriculum.terms.pop("terrain_levels")

    # 6. CONFIGURACIÓN DE CONTROL Y ESCENA
    cfg.actions["joint_pos"].scale = QUAD_BOT_ACTION_SCALE
    cfg.terminations["illegal_contact"] = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={"sensor_name": "nonfoot_ground_touch"},
    )
    
    cfg.events.pop("push_robot", None)
    cfg.viewer.body_name = "trunk"
    cfg.scene.entities["robot"].init_state.pos = (0.0, 0.0, 0.3)

    if play:
        if "actor" in cfg.observations:
            cfg.observations["actor"].enable_corruption = False
        cfg.scene.num_envs = 1

    return cfg