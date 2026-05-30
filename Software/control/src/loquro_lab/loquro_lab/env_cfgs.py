from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from .quad_bot_constants import get_quad_bot_cfg

_FOOT_GEOM_NAMES = (
    "fl_foot_collision",
    "fr_foot_collision",
    "rl_foot_collision",
    "rr_foot_collision",
)


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
    # Remove observation terms that depend on raycast/terrain-height sensors.
    for group in ("actor", "critic"):
        if group in cfg.observations:
            for term in ("height_scan", "foot_height"):
                cfg.observations[group].terms.pop(term, None)

    # ── Simulation ───────────────────────────────────────────────────────────
    # 0.005 s is too coarse for the 4-bar linkage equality constraints.
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
    cfg.rewards["pose"].params["asset_cfg"].joint_names = (
        ".*_shoulder_joint",
        ".*_knee_joint",
        ".*_femur_joint",
    )
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

    cfg.events["foot_friction"].params["asset_cfg"].geom_names = _FOOT_GEOM_NAMES
    cfg.events["base_com"].params["asset_cfg"].body_names = ("trunk",)

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
        cfg.observations["actor"].enable_corruption = False
        cfg.scene.num_envs = 1

    return cfg
