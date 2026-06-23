"""Registers the Unitree G1 box-carry velocity task.

Auto-discovered by src/tasks/__init__.py (import_packages), exactly like the
stock `config/g1` package.
"""

from mjlab.tasks.registry import register_mjlab_task
from src.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import unitree_g1_boxcarry_flat_env_cfg
from .rl_cfg import unitree_g1_boxcarry_ppo_runner_cfg

register_mjlab_task(
  task_id="Unitree-G1-BoxCarry-Flat",
  env_cfg=unitree_g1_boxcarry_flat_env_cfg(),
  play_env_cfg=unitree_g1_boxcarry_flat_env_cfg(play=True),
  rl_cfg=unitree_g1_boxcarry_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
