"""Curriculum terms for the G1 box-carry velocity task."""

from __future__ import annotations

from typing import TYPE_CHECKING, TypedDict

import torch

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


class ArmScaleStage(TypedDict):
  step: int      # env-steps (common_step_counter); ~ iteration * num_steps_per_env
  scale: float   # arm-pose delta magnitude (0..1)


def arm_pose_levels(
  env: "ManagerBasedRlEnv",
  env_ids: torch.Tensor,
  command_name: str,
  scale_stages: list[ArmScaleStage],
) -> dict[str, torch.Tensor]:
  """Ramp the arm-pose sampling magnitude over training.

  Starts narrow (poses near default -> easy, stable walking) and widens to the
  full front envelope, so the legs learn to compensate progressively. Mirrors the
  `commands_vel` curriculum pattern.
  """
  del env_ids  # Unused.
  term = env.command_manager.get_term(command_name)
  assert term is not None
  for stage in scale_stages:
    if env.common_step_counter > stage["step"]:
      term.delta_scale = float(stage["scale"])
  return {"arm_delta_scale": torch.tensor(float(term.delta_scale))}
