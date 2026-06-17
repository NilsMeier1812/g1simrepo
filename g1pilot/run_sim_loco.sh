#!/usr/bin/env bash
# Startet den Sim-Stack im LOCO-Modus: Basis FREI (HOLD_BASE_MODE=off), damit ein
# Loco-/Balance-Controller die Beine regeln kann. Sonst identisch zu run_sim.sh
# (X11-Freigabe + sauberes down + up).
#
# Ablauf: Stack starten -> Beine bekommen erst Kommandos, wenn ein Loco-Controller
# (sim_leg_hold zum Test, spaeter loco_sim) auf rt/lowcmd publiziert. Ohne das
# faellt der Roboter (keine Basis-Fixierung mehr) — das ist beabsichtigt.
set -e
cd "$(dirname "$0")"

xhost +local:root >/dev/null 2>&1 || \
  echo "[run_sim_loco] WARN: 'xhost' nicht verfuegbar - laeuft hier ein X-Server?"

docker compose --profile sim down --remove-orphans

export HOLD_BASE_MODE=off
echo "[run_sim_loco] HOLD_BASE_MODE=off -> Basis ist FREI (Loco-Modus)."
G1_SIM_MODE=true HOLD_BASE_MODE=off docker compose --profile sim up "$@"
