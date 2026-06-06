from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise
import torch

from .quad_bot_constants import QUAD_JOINT_POS, QUAD_RESET_POSE_RANGE, get_quad_bot_cfg

_FOOT_GEOM_NAMES = (
    "fl_foot_collision",
    "fr_foot_collision",
    "rl_foot_collision",
    "rr_foot_collision",
)

_ACTUATED_JOINT_NAMES = (
    ".*_shoulder_joint",
    ".*_knee_joint",
    ".*_femur_joint",
)

_ACTUATED_JOINT_CFG = SceneEntityCfg("robot", joint_names=_ACTUATED_JOINT_NAMES)

_ALL_JOINT_NAMES = tuple(QUAD_JOINT_POS.keys())
_ALL_JOINT_CFG = SceneEntityCfg("robot", joint_names=_ALL_JOINT_NAMES, preserve_order=True)
_ACTUATED_JOINT_POS = {
    name: pos
    for name, pos in QUAD_JOINT_POS.items()
    if name.endswith(("_shoulder_joint", "_knee_joint", "_femur_joint"))
}
_BODY_MASS_RANDOMIZATION_NAMES = (
    "trunk",
    ".*_shoulder_servo",
    ".*_knee_servo",
    ".*_hip_servo",
    ".*_servo_u_.*",
    ".*_hombro_pierna",
    ".*_pieza1_pierna",
    ".*_homoplato",
    ".*_tensor_aluminio",
    ".*_femur",
    ".*_tibia",
)
_ACTUATOR_CFG = SceneEntityCfg("robot", actuator_names=(".*",))


def reset_loquro_joints(env, env_ids, asset_cfg: SceneEntityCfg = _ALL_JOINT_CFG) -> None:
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)

    asset = env.scene[asset_cfg.name]
    joint_pos = torch.tensor(
        tuple(QUAD_JOINT_POS.values()), device=env.device, dtype=torch.float32
    ).repeat(len(env_ids), 1)
    joint_vel = torch.zeros_like(joint_pos)
    asset.write_joint_state_to_sim(
        joint_pos, joint_vel, joint_ids=asset_cfg.joint_ids, env_ids=env_ids
    )


def joint_pos_rel_f32(env, **kwargs) -> torch.Tensor:
    return mdp.joint_pos_rel(env, **kwargs).float()


def joint_vel_rel_f32(env, **kwargs) -> torch.Tensor:
    return mdp.joint_vel_rel(env, **kwargs).float()


def builtin_sensor_f32(env, **kwargs) -> torch.Tensor:
    return mdp.builtin_sensor(env, **kwargs).float()


def generated_commands_f32(env, **kwargs) -> torch.Tensor:
    return mdp.generated_commands(env, **kwargs).float()


def last_action_f32(env, **kwargs) -> torch.Tensor:
    return mdp.last_action(env, **kwargs).float()


def foot_air_time_f32(env, **kwargs) -> torch.Tensor:
    return mdp.foot_air_time(env, **kwargs).float()


def foot_contact_f32(env, **kwargs) -> torch.Tensor:
    return mdp.foot_contact(env, **kwargs).float()


def foot_contact_forces_f32(env, **kwargs) -> torch.Tensor:
    return mdp.foot_contact_forces(env, **kwargs).float()


def quad_bot_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    cfg = make_velocity_env_cfg()

    cfg.scene.entities = {"robot": get_quad_bot_cfg()}

    # ── Sensors ──────────────────────────────────────────────────────────────
    # Replace the base raycast sensors with contact sensors for our robot.
    cfg.scene.sensors = (
        ContactSensorCfg(
            name="feet_ground_contact",
            primary=ContactMatch(mode="geom", pattern=_FOOT_GEOM_NAMES, entity="robot"),
            secondary=ContactMatch(mode="body", pattern="terrain"),
            fields=("found", "force"),
            reduce="netforce",
            track_air_time=True,
        ),
        ContactSensorCfg(
            name="nonfoot_ground_touch",
            primary=ContactMatch(
                mode="geom",
                entity="robot",
                pattern=r".*_collision\d*$",
                exclude=_FOOT_GEOM_NAMES,
            ),
            secondary=ContactMatch(mode="body", pattern="terrain"),
            fields=("found",),
        ),
    )

    # ── Observations ─────────────────────────────────────────────────────────
    # LoQuRo hardware exposes 12 joint encoders and an IMU only.
    policy_terms = {
        "joint_pos": ObservationTermCfg(
            func=joint_pos_rel_f32,
            params={"asset_cfg": _ACTUATED_JOINT_CFG, "biased": True},
            noise=Unoise(n_min=-0.01, n_max=0.01),
        ),
        "joint_vel": ObservationTermCfg(
            func=joint_vel_rel_f32,
            params={"asset_cfg": _ACTUATED_JOINT_CFG},
            noise=Unoise(n_min=-0.5, n_max=0.5),
        ),
        "imu_accel": ObservationTermCfg(
            func=builtin_sensor_f32,
            params={"sensor_name": "robot/imu_accel"},
            noise=Unoise(n_min=-0.2, n_max=0.2),
            clip=(-30.0, 30.0),
            scale=0.1,
        ),
        "imu_ang_vel": ObservationTermCfg(
            func=builtin_sensor_f32,
            params={"sensor_name": "robot/imu_ang_vel"},
            noise=Unoise(n_min=-0.05, n_max=0.05),
            clip=(-10.0, 10.0),
        ),
        "command": ObservationTermCfg(
            func=generated_commands_f32,
            params={"command_name": "twist"},
        ),
    }
    policy_group_name = "policy" if "policy" in cfg.observations else "actor"
    cfg.observations[policy_group_name].terms = policy_terms

    # Keep asymmetric critic-only contact information, but remove unavailable
    # terrain/foot-height terms and keep its joint state aligned with encoders.
    if "critic" in cfg.observations:
        cfg.observations["critic"].terms["joint_pos"] = policy_terms["joint_pos"]
        cfg.observations["critic"].terms["joint_vel"] = policy_terms["joint_vel"]
        cfg.observations["critic"].terms["imu_accel"] = policy_terms["imu_accel"]
        cfg.observations["critic"].terms["imu_ang_vel"] = policy_terms["imu_ang_vel"]
        cfg.observations["critic"].terms["actions"] = ObservationTermCfg(
            func=last_action_f32
        )
        cfg.observations["critic"].terms["command"] = policy_terms["command"]
        cfg.observations["critic"].terms["foot_air_time"] = ObservationTermCfg(
            func=foot_air_time_f32,
            params={"sensor_name": "feet_ground_contact"},
        )
        cfg.observations["critic"].terms["foot_contact"] = ObservationTermCfg(
            func=foot_contact_f32,
            params={"sensor_name": "feet_ground_contact"},
        )
        cfg.observations["critic"].terms["foot_contact_forces"] = ObservationTermCfg(
            func=foot_contact_forces_f32,
            params={"sensor_name": "feet_ground_contact"},
        )
        cfg.observations["critic"].terms.pop("base_lin_vel", None)
        cfg.observations["critic"].terms.pop("base_ang_vel", None)
        cfg.observations["critic"].terms.pop("projected_gravity", None)
        cfg.observations["critic"].terms.pop("height_scan", None)
        cfg.observations["critic"].terms.pop("foot_height", None)

    # ── Simulation ───────────────────────────────────────────────────────────
    # Raise decimation to 10 to keep step_dt = 0.002 × 10 = 0.02 s (50 Hz
    # control), so gamma horizon and all reward scales stay unchanged.
    cfg.sim.mujoco.timestep = 0.002
    cfg.decimation = 10

    # ── Terrain ──────────────────────────────────────────────────────────────
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None

    # ── Curriculum ───────────────────────────────────────────────────────────
    # terrain_levels curriculum requires procedural terrain; remove it.
    if isinstance(cfg.curriculum, dict):
        cfg.curriculum.pop("terrain_levels", None)
    elif hasattr(cfg.curriculum, "terms"):
        cfg.curriculum.terms.pop("terrain_levels", None)

    # ── Actions ───────────────────────────────────────────────────────────────
    # Go1 uses ~0.25 rad per unit action (effort/stiffness scaled). 0.25 limits
    # large position jumps that cause jerky behaviour at 0.5.
    cfg.actions["joint_pos"].scale = 0.25
    cfg.actions["joint_pos"].offset = _ACTUATED_JOINT_POS
    cfg.actions["joint_pos"].use_default_offset = False

    # Use the same spawn pose convention as quad_bot_constants.py. The inherited
    # Go1 reset randomized yaw over [-pi, pi], which made LoQuRo start reversed.
    cfg.events["reset_base"].params["pose_range"] = dict(QUAD_RESET_POSE_RANGE)

    # LoQuRo is much smaller than Go1; starting with Go1-sized commands makes
    # early PPO episodes mostly falls.  Keep the same 3-D command interface but
    # begin with reachable flat-ground targets and let curriculum expand them.
    cfg.commands["twist"].ranges.lin_vel_x = (-0.4, 0.6)
    cfg.commands["twist"].ranges.lin_vel_y = (-0.15, 0.15)
    cfg.commands["twist"].ranges.ang_vel_z = (-0.5, 0.5)
    if "command_vel" in cfg.curriculum:
        cfg.curriculum["command_vel"].params["velocity_stages"] = [
            {
                "step": 0,
                "lin_vel_x": (-0.3, 0.4),
                "lin_vel_y": (-0.1, 0.1),
                "ang_vel_z": (-0.4, 0.4),
            },
            {
                "step": 5000 * 24,
                "lin_vel_x": (-0.5, 0.7),
                "lin_vel_y": (-0.15, 0.15),
                "ang_vel_z": (-0.6, 0.6),
            },
            {
                "step": 10000 * 24,
                "lin_vel_x": (-0.8, 1.0),
                "lin_vel_y": (-0.2, 0.2),
                "ang_vel_z": (-0.8, 0.8),
            },
        ]

    # ── Rewards ──────────────────────────────────────────────────────────────
    cfg.rewards["track_linear_velocity"].weight = 2.0
    cfg.rewards["track_angular_velocity"].weight = 2.0   # match Go1 (was 1.0)
    cfg.rewards["upright"].weight = 1.0                  # match Go1 (was 0.5)
    cfg.rewards["upright"].params["asset_cfg"].body_names = ("trunk",)

    # body_ang_vel is kept at weight=0 but needs a valid body.
    cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("trunk",)

    # Configure pose (variable_posture) reward with LoQuRo joint name patterns.
    # Restrict to the 12 actuated joints — the base cfg uses ".*" which includes
    # the 16 passive joints, causing a size mismatch with the 12-entry std tensors.
    cfg.rewards["pose"].params["asset_cfg"].joint_names = _ACTUATED_JOINT_NAMES
    # Tight std when standing, looser when walking/running — same values as Go1.
    cfg.rewards["pose"].params["std_standing"] = {
        r".*_(shoulder|femur)_joint": 0.05,
        r".*_knee_joint": 0.1,
    }
    cfg.rewards["pose"].params["std_walking"] = {
        r".*_(shoulder|femur)_joint": 0.3,
        r".*_knee_joint": 0.6,
    }
    cfg.rewards["pose"].params["std_running"] = {
        r".*_(shoulder|femur)_joint": 0.3,
        r".*_knee_joint": 0.6,
    }

    # Remove only rewards that need terrain-height sensors or foot sites.
    # Keep dof_pos_limits (joint limit enforcement) and pose (configured above).
    for key in (
        "foot_clearance",
        "foot_swing_height",
        "foot_slip",
    ):
        cfg.rewards.pop(key, None)

    # Gait reward: small weight drives foot liftoff on flat terrain.
    # Go1 flat uses 0.0 here; a small non-zero value helps LoQuRo learn a gait.
    cfg.rewards["air_time"].weight = 0.25   # reduced from 1.0
    cfg.rewards["air_time"].params["sensor_name"] = "feet_ground_contact"

    # ── Events ───────────────────────────────────────────────────────────────
    cfg.events.pop("push_robot", None)

    cfg.events["reset_robot_joints"] = EventTermCfg(
        func=reset_loquro_joints,
        mode="reset",
        params={"asset_cfg": _ALL_JOINT_CFG},
    )
    cfg.events["foot_friction"].params["asset_cfg"].geom_names = _FOOT_GEOM_NAMES
    cfg.events["base_com"].params["asset_cfg"].body_names = ("trunk",)
    cfg.events["body_inertia"] = EventTermCfg(
        mode="startup",
        func=mdp.dr.pseudo_inertia,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", body_names=_BODY_MASS_RANDOMIZATION_NAMES
            ),
            # pseudo_inertia scales mass and inertia by exp(2 * alpha).
            # These bounds are approximately equivalent to mass scale [0.93, 1.07].
            "alpha_range": (-0.0363, 0.0338),
        },
    )
    cfg.events["actuator_pd_gains"] = EventTermCfg(
        mode="startup",
        func=mdp.dr.pd_gains,
        params={
            "asset_cfg": _ACTUATOR_CFG,
            "operation": "scale",
            "kp_range": (0.90, 1.10),
            "kd_range": (0.90, 1.10),
        },
    )
    cfg.events["actuator_effort_limits"] = EventTermCfg(
        mode="startup",
        func=mdp.dr.effort_limits,
        params={
            "asset_cfg": _ACTUATOR_CFG,
            "operation": "scale",
            "effort_limit_range": (0.90, 1.10),
        },
    )

    # ── Terminations ─────────────────────────────────────────────────────────
    cfg.terminations.pop("out_of_terrain_bounds", None)
    cfg.terminations["illegal_contact"] = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={"sensor_name": "nonfoot_ground_touch"},
    )

    # ── Viewer ───────────────────────────────────────────────────────────────
    cfg.viewer.body_name = "trunk"

    # ── Play mode ─────────────────────────────────────────────────────────────
    if play:
        cfg.observations[policy_group_name].enable_corruption = False
        cfg.scene.num_envs = 1

    return cfg
