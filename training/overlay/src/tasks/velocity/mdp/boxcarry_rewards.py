"""Rewards for the G1 box-carry velocity task."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.entity import Entity
from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def arm_pose_tracking(
  env: "ManagerBasedRlEnv",
  std: float,
  command_name: str,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Reward the arm joints for matching the commanded (front) arm pose.

  reward = exp(-mean_over_joints((q_arm - q_arm_cmd)^2) / std^2)

  IMPORTANT: `asset_cfg.joint_names` MUST be the SAME arm-joint set/regex as the
  `arm_pose` command, so that `asset.data.joint_pos[:, joint_ids]` and the command
  tensor are in the same (model) order.
  """
  asset: Entity = env.scene[asset_cfg.name]
  command = env.command_manager.get_command(command_name)
  assert command is not None, f"Command '{command_name}' not found."
  current = asset.data.joint_pos[:, asset_cfg.joint_ids]
  error = torch.mean(torch.square(current - command), dim=1)
  return torch.exp(-error / std**2)
