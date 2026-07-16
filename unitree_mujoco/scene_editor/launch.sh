#!/usr/bin/env bash
# =====================================================================
# Starter fuer den mujoco-scene-editor und den MuJoCo-Viewer.
#
#   ./launch.sh edit            Beispiel-Umgebung im Editor oeffnen (Browser)
#   ./launch.sh edit <datei>    beliebige Szene im Editor oeffnen
#   ./launch.sh new             leere Szene im Editor starten
#   ./launch.sh prompt "text"   Szene per Text-Prompt generieren (braucht API-Key)
#   ./launch.sh view            environment_starter.xml im MuJoCo-Viewer ansehen
#   ./launch.sh view-g1         G1 + Objekte (scene_g1_playground.xml) ansehen
#
# Der Editor oeffnet einen lokalen Webserver (http://127.0.0.1:8080).
# =====================================================================
set -euo pipefail
cd "$(dirname "$0")"

VENV=".venv"
if [[ ! -x "$VENV/bin/python" ]]; then
  echo "Kein virtualenv gefunden. Bitte zuerst  ./setup.sh  ausfuehren." >&2
  exit 1
fi

# RoBits-Config persistent halten
export ROBITS_CONFIG_DIR="${ROBITS_CONFIG_DIR:-$(pwd)/.robits_config}"

CMD="${1:-edit}"; shift || true

case "$CMD" in
  new)
    exec "$VENV/bin/mjcreate" "$@"
    ;;
  edit)
    SCENE="${1:-scenes/environment_starter.xml}"
    exec "$VENV/bin/mjedit" "$SCENE"
    ;;
  prompt)
    exec "$VENV/bin/mjprompt" "$@"
    ;;
  view)
    SCENE="${1:-scenes/environment_starter.xml}"
    exec "$VENV/bin/python" -m mujoco.viewer --mjcf="$SCENE"
    ;;
  view-g1)
    exec "$VENV/bin/python" -m mujoco.viewer \
      --mjcf="../unitree_robots/g1/scene_g1_playground.xml"
    ;;
  *)
    echo "Unbekanntes Kommando: $CMD" >&2
    echo "Benutze: new | edit [datei] | prompt \"text\" | view [datei] | view-g1" >&2
    exit 1
    ;;
esac
