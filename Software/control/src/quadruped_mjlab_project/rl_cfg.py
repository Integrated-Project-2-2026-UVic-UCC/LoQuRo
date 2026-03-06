from mjlab.tasks.velocity.rl import RslRlOnPolicyRunnerCfg

def quad_bot_ppo_runner_cfg():
    return RslRlOnPolicyRunnerCfg(
        num_steps_per_env=24,
        max_iterations=3000,
        save_interval=50,
        experiment_name="quad_bot_velocity",
        empirical_normalization=True,
        policy={
            "class_name": "ActorCritic",
            "activation": "elu",
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
        },
        algorithm={
            "class_name": "PPO",
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "learning_rate": 1e-3,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
        },
    )