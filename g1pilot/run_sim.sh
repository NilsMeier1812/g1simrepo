#!/usr/bin/env bash
# Startet den Sim-Stack zuverlaessig:
#   1) X11 fuer Docker-Container freigeben (xhost; ueberlebt Reboot/Re-Login NICHT,
#      daher hier jedes Mal),
#   2) evtl. alte/hintergruendige Container sauber stoppen,
#   3) Sim hochfahren.
# Zusaetzliche Argumente werden an `docker compose up` durchgereicht
# (z.B. `./run_sim.sh --build` zum Neubauen der Images).
set -e
cd "$(dirname "$0")"

# X-Server-Zugriff fuer lokale Container erlauben (MuJoCo-Viewer + RViz).
xhost +local:root >/dev/null 2>&1 || \
  echo "[run_sim] WARN: 'xhost' nicht verfuegbar - laeuft hier ueberhaupt ein X-Server?"

# Reste eines frueheren Laufs entfernen (Strg+C laesst Container sonst stehen).
docker compose --profile sim down --remove-orphans

G1_SIM_MODE=true docker compose --profile sim up "$@"
