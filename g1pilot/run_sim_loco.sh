#!/usr/bin/env bash
# Startet den Sim-Stack im LOCO-Modus: Basis FREI (HOLD_BASE_MODE=off), damit ein
# Loco-/Balance-Controller die Beine regeln kann. Sonst identisch zu run_sim.sh
# (X11-Freigabe + sauberes down + up).
#
# Ablauf: Stack starten. Die Bridge haelt im off-Modus die Beine/Taille in einer
# Standpose (Loco-Startup-Hold), bis loco_sim verbunden ist und auf rt/lowcmd
# kommandiert — so faellt der Roboter im Startfenster (colcon build/Launch) NICHT
# um. Danach uebernimmt loco_sim (HOLD), und START BALANCING startet die Policy.
set -e
cd "$(dirname "$0")"

xhost +local:root >/dev/null 2>&1 || \
  echo "[run_sim_loco] WARN: 'xhost' nicht verfuegbar - laeuft hier ein X-Server?"

docker compose --profile sim down --remove-orphans

export HOLD_BASE_MODE=off
echo "[run_sim_loco] HOLD_BASE_MODE=off -> Basis ist FREI (Loco-Modus)."
G1_SIM_MODE=true HOLD_BASE_MODE=off docker compose --profile sim up "$@"
