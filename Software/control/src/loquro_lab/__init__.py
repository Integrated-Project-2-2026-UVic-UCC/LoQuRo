from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner
# Importamos el RL del Go1 que sabemos que carga bien
from mjlab.tasks.velocity.config.go1.rl_cfg import unitree_go1_ppo_runner_cfg

from .env_cfgs import quad_bot_flat_env_cfg

register_mjlab_task(
    task_id="quad_bot",
    env_cfg=quad_bot_flat_env_cfg(play=False),
    play_env_cfg=quad_bot_flat_env_cfg(play=True),
    rl_cfg=unitree_go1_ppo_runner_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)