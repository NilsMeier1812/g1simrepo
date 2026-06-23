"""RL runner config for the G1 box-carry task.

Same PPO hyper-parameters as the stock G1 velocity task (they are well tuned for
this robot); only the experiment name, the iteration budget and the checkpoint
frequency change. The arm-conditioned task is harder than the stock walker, so we
allow many more iterations (good fit for a weekend run on one 4090).
"""

from mjlab.rl import RslRlOnPolicyRunnerCfg

from src.tasks.velocity.config.g1.rl_cfg import unitree_g1_ppo_runner_cfg


def unitree_g1_boxcarry_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  cfg = unitree_g1_ppo_runner_cfg()
  cfg.experiment_name = "g1_boxcarry"
  # Weekend budget. The curriculum is done by ~iter 4000, so the large remainder
  # trains the full target. Override on the CLI with --agent.max-iterations=N.
  cfg.max_iterations = 30000
  # Checkpoint OFTEN so an unattended hard shutdown loses only ~minutes of work
  # (every model_<iter>.pt is a full resume point; policy.onnx is exported too).
  cfg.save_interval = 50
  return cfg
