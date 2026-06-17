#!/usr/bin/env bash
# Startet den Sim-Stack im LOCO-Modus: Basis FREI (HOLD_BASE_MODE=off), damit ein
# Loco-/Balance-Controller die Beine regeln kann. Sonst identisch zu run_sim.sh
# (X11-Freigabe + sauberes down + up).
#
# Ablauf: Stack starten. Die Bridge haelt im off-Modus die Basis (Managed-Weld),
# bis loco_sim das Balancieren startet — so faellt der Roboter im Startfenster
# (colcon build/Launch) NICHT um, egal wie langsam der PC ist. START BALANCING:
# Bridge stellt den Roboter in eine saubere Stand-Pose, loest den Weld, Policy laeuft.
set -e
cd "$(dirname "$0")"

xhost +local:root >/dev/null 2>&1 || \
  echo "[run_sim_loco] WARN: 'xhost' nicht verfuegbar - laeuft hier ein X-Server?"

docker compose --profile sim down --remove-orphans

export HOLD_BASE_MODE=off
echo "[run_sim_loco] HOLD_BASE_MODE=off -> Basis ist FREI (Loco-Modus)."
G1_SIM_MODE=true HOLD_BASE_MODE=off docker compose --profile sim up "$@"
