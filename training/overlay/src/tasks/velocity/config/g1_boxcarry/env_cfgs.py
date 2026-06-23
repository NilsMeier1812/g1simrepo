"""Unitree G1 BOX-CARRY velocity environment configuration.

Goal: a policy that walks AND holds velocity 0 *without depending on the arms*,
and that tolerates the arms being held in arbitrary FRONT-of-body poses (carrying
an empty box in front). Built as a thin layer on top of the stock G1 flat
velocity config, changing only what's needed:

  1. New `arm_pose` command  -> samples a front arm-joint setpoint, given to the
     policy as observation (it now *knows* where the arms are asked to be).
  2. New `arm_pose_tracking` reward -> arms follow that setpoint.
  3. `pose` + `stand_still` rewards restricted to NON-arm joints -> the arms are
     no longer pulled to a single default pose (the root cause of the stock policy
     falling when you move the arms).
  4. `carry_payload` event -> a small forward CoM offset on the torso (proxy for an
     empty box held in front; the dominant effect is already the forward arm pose).
  5. `arm_pose_levels` curriculum -> widen the arm envelope from near-default to
     the full front range over training.

Everything else (leg/balance rewards, push, friction, terminations) is inherited.
"""

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg

import src.tasks.velocity.mdp as mdp
from src.tasks.velocity.config.g1.env_cfgs import unitree_g1_flat_env_cfg

# ── Joint groups (G1-29 names) ────────────────────────────────────────────────
ARM_JOINTS = (
  ".*_shoulder_pitch_joint",
  ".*_shoulder_roll_joint",
  ".*_shoulder_yaw_joint",
  ".*_elbow_joint",
  ".*_wrist_roll_joint",
  ".*_wrist_pitch_joint",
  ".*_wrist_yaw_joint",
)
NON_ARM_JOINTS = (
  ".*_hip_pitch_joint",
  ".*_hip_roll_joint",
  ".*_hip_yaw_joint",
  ".*_knee_joint",
  ".*_ankle_pitch_joint",
  ".*_ankle_roll_joint",
  "waist_yaw_joint",
  "waist_roll_joint",
  "waist_pitch_joint",
)

# ── FRONT arm envelope (deltas around the front-ish default pose) ─────────────
# Defaults: shoulder_pitch=0.35, elbow=0.87, shoulder_roll=+-0.18, rest=0.
# shoulder_pitch only forward/up (never behind); elbow only more-bent (carry).
ARM_DELTA_LOW = {
  ".*_shoulder_pitch_joint": -0.25,  # -> 0.10 (slightly lowered, still front)
  ".*_shoulder_roll_joint": -0.25,
  ".*_shoulder_yaw_joint": -0.35,
  ".*_elbow_joint": -0.45,           # -> 0.42 (still clearly bent)
  ".*_wrist_roll_joint": -0.20,
  ".*_wrist_pitch_joint": -0.20,
  ".*_wrist_yaw_joint": -0.20,
}
ARM_DELTA_HIGH = {
  ".*_shoulder_pitch_joint": 0.55,   # -> 0.90 (raised in front)
  ".*_shoulder_roll_joint": 0.25,
  ".*_shoulder_yaw_joint": 0.35,
  ".*_elbow_joint": 0.55,            # -> 1.42 (tight carry bend)
  ".*_wrist_roll_joint": 0.20,
  ".*_wrist_pitch_joint": 0.20,
  ".*_wrist_yaw_joint": 0.20,
}

# Pose-reward std for NON-arm joints (legs+waist) — copied from the stock G1
# config (arms removed). Keeps the legs/waist behaviour identical to the proven
# walker.
_STD_WALKING_NONARM = {
  r".*hip_pitch.*": 0.5,
  r".*hip_roll.*": 0.15,
  r".*hip_yaw.*": 0.15,
  r".*knee.*": 0.5,
  r".*ankle_pitch.*": 0.15,
  r".*ankle_roll.*": 0.1,
  r".*waist_yaw.*": 0.15,
  r".*waist_roll.*": 0.1,
  r".*waist_pitch.*": 0.1,
}
_STD_RUNNING_NONARM = {
  r".*hip_pitch.*": 0.5,
  r".*hip_roll.*": 0.25,
  r".*hip_yaw.*": 0.25,
  r".*knee.*": 0.5,
  r".*ankle_pitch.*": 0.25,
  r".*ankle_roll.*": 0.1,
  r".*waist_yaw.*": 0.25,
  r".*waist_roll.*": 0.1,
  r".*waist_pitch.*": 0.1,
}


def unitree_g1_boxcarry_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  cfg = unitree_g1_flat_env_cfg(play=play)

  # 1) Arm-pose command (front envelope, curriculum-scaled magnitude).
  cfg.commands["arm_pose"] = mdp.ArmPoseCommandCfg(
    entity_name="robot",
    joint_names=ARM_JOINTS,
    resampling_time_range=(2.0, 4.0),  # change the held arm pose every 2-4 s
    rel_default_envs=0.1,              # 10% keep the exact default pose
    init_delta_scale=0.3,             # start near default (curriculum widens it)
    delta_low=ARM_DELTA_LOW,
    delta_high=ARM_DELTA_HIGH,
  )

  # 2) Arm-pose setpoint as observation (actor AND critic).
  for group in ("actor", "critic"):
    cfg.observations[group].terms["arm_pose_cmd"] = ObservationTermCfg(
      func=mdp.generated_commands,
      params={"command_name": "arm_pose"},
    )

  # 3) Free the arms from the default-pose pull: restrict pose + stand_still to
  #    non-arm joints. Legs/waist behaviour stays exactly as in the stock walker.
  cfg.rewards["pose"].params["asset_cfg"] = SceneEntityCfg(
    "robot", joint_names=NON_ARM_JOINTS
  )
  cfg.rewards["pose"].params["std_walking"] = _STD_WALKING_NONARM
  cfg.rewards["pose"].params["std_running"] = _STD_RUNNING_NONARM
  # std_standing stays {".*": 0.05} -> matches the non-arm joints only now.
  cfg.rewards["stand_still"].params["asset_cfg"] = SceneEntityCfg(
    "robot", joint_names=NON_ARM_JOINTS
  )

  # 4) Arm-pose tracking reward (same arm-joint set as the command!).
  cfg.rewards["arm_pose_tracking"] = RewardTermCfg(
    func=mdp.arm_pose_tracking,
    weight=1.0,
    params={
      "std": 0.4,
      "command_name": "arm_pose",
      "asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINTS),
    },
  )

  # 5) Payload proxy: small forward/up CoM offset on the torso (empty box in
  #    front). Startup randomization; stacks with the existing base_com jitter.
  cfg.events["carry_payload"] = EventTermCfg(
    func=dr.body_com_offset,
    mode="startup",
    params={
      "asset_cfg": SceneEntityCfg("robot", body_names=("torso_link",)),
      "operation": "add",
      "ranges": {
        0: (0.0, 0.08),    # +x forward (box held in front)
        1: (-0.02, 0.02),  # y
        2: (0.0, 0.05),    # +z up
      },
    },
  )

  # 6) Curriculum: widen the arm envelope as the legs learn to compensate.
  if not play:
    cfg.curriculum["arm_pose_levels"] = CurriculumTermCfg(
      func=mdp.arm_pose_levels,
      params={
        "command_name": "arm_pose",
        "scale_stages": [
          {"step": 0, "scale": 0.3},
          {"step": 1500 * 24, "scale": 0.6},
          {"step": 4000 * 24, "scale": 1.0},
        ],
      },
    )
  else:
    # In play mode show the full envelope right away.
    cfg.commands["arm_pose"].init_delta_scale = 1.0

  return cfg
