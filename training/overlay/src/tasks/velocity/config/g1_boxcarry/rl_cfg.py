"""RL runner config for the G1 box-carry task.

Same PPO hyper-parameters as the stock G1 velocity task (they are well tuned for
this robot); only the experiment name and the iteration budget change. The
arm-conditioned task is a bit harder, so we allow more iterations.
"""

from mjlab.rl import RslRlOnPolicyRunnerCfg

from src.tasks.velocity.config.g1.rl_cfg import unitree_g1_ppo_runner_cfg


def unitree_g1_boxcarry_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  cfg = unitree_g1_ppo_runner_cfg()
  cfg.experiment_name = "g1_boxcarry"
  cfg.max_iterations = 15000
  return cfg
