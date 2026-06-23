#!/usr/bin/env bash
# Copies the box-carry overlay into a unitree_rl_mjlab checkout and registers the
# new MDP terms. Idempotent.
#
# Usage:  ./apply_overlay.sh /path/to/unitree_rl_mjlab
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
TARGET="${1:-}"

if [ -z "$TARGET" ]; then
  echo "Usage: $0 /path/to/unitree_rl_mjlab"
  exit 1
fi
if [ ! -d "$TARGET/src/tasks/velocity/mdp" ]; then
  echo "ERROR: '$TARGET' does not look like a unitree_rl_mjlab checkout"
  echo "       (missing src/tasks/velocity/mdp)."
  exit 1
fi

echo ">> Copying MDP terms ..."
cp -v "$HERE/overlay/src/tasks/velocity/mdp/arm_pose_command.py"     "$TARGET/src/tasks/velocity/mdp/"
cp -v "$HERE/overlay/src/tasks/velocity/mdp/boxcarry_rewards.py"     "$TARGET/src/tasks/velocity/mdp/"
cp -v "$HERE/overlay/src/tasks/velocity/mdp/boxcarry_curriculum.py"  "$TARGET/src/tasks/velocity/mdp/"

echo ">> Copying task config (g1_boxcarry) ..."
mkdir -p "$TARGET/src/tasks/velocity/config/g1_boxcarry"
cp -v "$HERE"/overlay/src/tasks/velocity/config/g1_boxcarry/*.py "$TARGET/src/tasks/velocity/config/g1_boxcarry/"

echo ">> Registering new MDP terms in mdp/__init__.py ..."
INIT="$TARGET/src/tasks/velocity/mdp/__init__.py"
for mod in arm_pose_command boxcarry_rewards boxcarry_curriculum; do
  if ! grep -q "from .$mod import" "$INIT"; then
    echo "from .$mod import *  # noqa: F401, F403" >> "$INIT"
    echo "   + from .$mod import *"
  else
    echo "   = from .$mod import * (already present)"
  fi
done

echo ""
echo ">> Done. Verify the task is registered:"
echo "     python scripts/list_envs.py | grep -i boxcarry"
