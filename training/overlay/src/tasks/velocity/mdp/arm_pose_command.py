"""Arm-pose command term for the G1 box-carry velocity task.

Samples a TARGET joint configuration for the arm joints, biased to FRONT-of-body
poses (suitable for carrying a box in front). The command is provided to the
policy as an observation and tracked by a reward, so the policy learns to walk /
stand at v=0 robustly *for any commanded front arm pose* — instead of relying on
the arms sitting at a single default pose (which is why the stock policy falls
when you move the arms).

Design:
  target = default_arm + delta_scale * U(delta_low, delta_high)   (per joint)
  clamped to the robot's soft joint limits.
- Ranges are defined as a DELTA around the (already front-ish) default pose, with
  an asymmetric, forward-biased shoulder_pitch and a bent-only elbow -> the hands
  stay IN FRONT (never behind / never straight-down-back).
- `delta_scale` is ramped 0.3 -> 1.0 by the `arm_pose_levels` curriculum so the
  policy first masters near-default poses, then the full front envelope.
- A fraction `rel_default_envs` keep the exact default pose every resample, so the
  nominal configuration stays well represented (stable walking is not forgotten).

Mirrors the framework's CommandTerm API (see velocity_command.py).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import torch

from mjlab.entity import Entity
from mjlab.managers.command_manager import CommandTerm, CommandTermCfg
from mjlab.utils.lab_api.string import resolve_matching_names_values

if TYPE_CHECKING:
  from mjlab.envs.manager_based_rl_env import ManagerBasedRlEnv


class ArmPoseCommand(CommandTerm):
  cfg: "ArmPoseCommandCfg"

  def __init__(self, cfg: "ArmPoseCommandCfg", env: "ManagerBasedRlEnv"):
    super().__init__(cfg, env)

    self.robot: Entity = env.scene[cfg.entity_name]
    self.joint_ids, self.joint_names = self.robot.find_joints(list(cfg.joint_names))
    self.num_joints = len(self.joint_ids)

    # Per-joint delta ranges, resolved (regex -> value) in joint_names order.
    _, _, low = resolve_matching_names_values(
      data=dict(cfg.delta_low), list_of_strings=self.joint_names
    )
    _, _, high = resolve_matching_names_values(
      data=dict(cfg.delta_high), list_of_strings=self.joint_names
    )
    self._delta_low = torch.tensor(low, device=self.device, dtype=torch.float32)
    self._delta_high = torch.tensor(high, device=self.device, dtype=torch.float32)

    # Curriculum-controlled magnitude (see arm_pose_levels).
    self.delta_scale = float(cfg.init_delta_scale)

    # Default arm pose (per env), used as the center of the sampling.
    default_joint_pos = self.robot.data.default_joint_pos
    assert default_joint_pos is not None
    self._default_arm = default_joint_pos[:, self.joint_ids].clone()

    self.arm_pose_target = self._default_arm.clone()
    self.metrics["arm_pose_error"] = torch.zeros(self.num_envs, device=self.device)

  @property
  def command(self) -> torch.Tensor:
    return self.arm_pose_target

  def _resample_command(self, env_ids: torch.Tensor) -> None:
    n = len(env_ids)
    if n == 0:
      return
    r = torch.rand(n, self.num_joints, device=self.device)
    delta = (self._delta_low + r * (self._delta_high - self._delta_low)) * self.delta_scale
    target = self._default_arm[env_ids] + delta

    # Clamp to soft joint limits (stay physically reachable).
    limits = self.robot.data.soft_joint_pos_limits[env_ids][:, self.joint_ids, :]
    target = torch.clip(target, limits[:, :, 0], limits[:, :, 1])

    # Keep a fraction exactly at default (nominal config stays well represented).
    keep_default = torch.rand(n, device=self.device) < self.cfg.rel_default_envs
    target[keep_default] = self._default_arm[env_ids][keep_default]

    self.arm_pose_target[env_ids] = target

  def _update_command(self) -> None:
    # Static between resamples (the arm "setpoint" is held, like a held box pose).
    pass

  def _update_metrics(self) -> None:
    cur = self.robot.data.joint_pos[:, self.joint_ids]
    max_step = self.cfg.resampling_time_range[1] / self._env.step_dt
    self.metrics["arm_pose_error"] += (
      torch.norm(cur - self.arm_pose_target, dim=1) / max_step
    )


@dataclass(kw_only=True)
class ArmPoseCommandCfg(CommandTermCfg):
  entity_name: str
  joint_names: tuple[str, ...]
  # Per-joint (regex -> radians) sampling deltas around the default pose.
  delta_low: dict[str, float] = field(default_factory=dict)
  delta_high: dict[str, float] = field(default_factory=dict)
  init_delta_scale: float = 0.3
  rel_default_envs: float = 0.1

  def build(self, env: "ManagerBasedRlEnv") -> ArmPoseCommand:
    return ArmPoseCommand(self, env)
