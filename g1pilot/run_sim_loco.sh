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

# Realtime-Faktor: Sim langsamer als Echtzeit, damit die 50-Hz-Policy auf langsamen
# PCs pro Schritt die volle Physik bekommt (sonst driftet/faellt sie). 0.5 = halbe
# Geschwindigkeit (Roboter optisch in Zeitlupe, aber dynamisch korrekt). Tuning:
# FACTOR <= (in loco_sim geloggte policy-Hz)/50. Default 0.5; per Env ueberschreibbar:
#   SIM_REALTIME_FACTOR=0.6 ./run_sim_loco.sh
export SIM_REALTIME_FACTOR=${SIM_REALTIME_FACTOR:-0.5}
echo "[run_sim_loco] SIM_REALTIME_FACTOR=$SIM_REALTIME_FACTOR (Sim-Geschwindigkeit; 1.0=Echtzeit)."

G1_SIM_MODE=true HOLD_BASE_MODE=off SIM_REALTIME_FACTOR=$SIM_REALTIME_FACTOR \
  docker compose --profile sim up "$@"
