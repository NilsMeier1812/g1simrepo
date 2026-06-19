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

# LOCKSTEP (deterministische Regelrate): die Sim macht GARANTIERT decimation=20
# Physikschritte pro rt/lowcmd und publiziert danach EINEN frischen rt/lowstate;
# loco_sim rechnet ein Kommando pro State. Damit sieht die 50-Hz-Policy auf JEDEM
# PC exakt ihre trainierten 20 ms Physik/Schritt - ohne das fragile per-PC-Tuning
# von SIM_REALTIME_FACTOR. Der Realtime-Faktor ist im Lockstep irrelevant (kein
# Wall-Clock-Sleep). Der Roboter laeuft auf langsamen PCs optisch langsamer, regelt
# aber korrekt. Abschaltbar mit SIM_LOCKSTEP=0 (dann gilt wieder der Faktor unten).
export SIM_LOCKSTEP=${SIM_LOCKSTEP:-1}
export SIM_REALTIME_FACTOR=${SIM_REALTIME_FACTOR:-1.0}
echo "[run_sim_loco] SIM_LOCKSTEP=$SIM_LOCKSTEP (1=deterministische 50-Hz-Regelrate; Faktor=$SIM_REALTIME_FACTOR, im Lockstep irrelevant)."

G1_SIM_MODE=true HOLD_BASE_MODE=off SIM_LOCKSTEP=$SIM_LOCKSTEP SIM_REALTIME_FACTOR=$SIM_REALTIME_FACTOR \
  docker compose --profile sim up "$@"
