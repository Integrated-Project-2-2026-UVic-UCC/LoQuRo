from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

# 1. Importamos entorno y configuración de IA 
from .env_cfgs import quad_bot_flat_env_cfg
from .rl_cfg import quad_bot_ppo_runner_cfg

# 2. Registramos la tarea
register_mjlab_task(
    task_id="quad_bot",
    env_cfg=quad_bot_flat_env_cfg(play=False),
    play_env_cfg=quad_bot_flat_env_cfg(play=True),
    rl_cfg=quad_bot_ppo_runner_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)