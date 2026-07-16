#!/usr/bin/env bash
# =====================================================================
# Starter fuer den mujoco-scene-editor und den MuJoCo-Viewer.
#
# Ohne Argument -> interaktives Menue: listet alle verfuegbaren Szenen
# nummeriert auf; du waehlst per Zahl, welche gestartet wird.
#
#   ./launch.sh                 interaktives Menue (empfohlen)
#   ./launch.sh new             leere Szene im Editor starten
#   ./launch.sh edit <datei>    bestimmte Szene im Editor oeffnen
#   ./launch.sh prompt "text"   Szene per Text-Prompt generieren (braucht API-Key)
#   ./launch.sh view <datei>    Szene im MuJoCo-Viewer ansehen
#   ./launch.sh view-g1         G1 + Objekte (scene_g1_playground.xml) ansehen
#
# Der Editor oeffnet einen lokalen Webserver (http://127.0.0.1:8080).
# Exportierte Szenen landen automatisch im Ordner  scenes/  (siehe run_editor.py).
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

PY="$VENV/bin/python"
G1_SCENE="../unitree_robots/g1/scene_g1_playground.xml"

edit_scene()  { exec "$PY" run_editor.py edit "$1"; }
new_scene()   { exec "$PY" run_editor.py new; }
view_scene()  { exec "$PY" -m mujoco.viewer --mjcf="$1"; }

# --- alle verfuegbaren Szenen einsammeln -----------------------------
collect_scenes() {
  SCENES=(); LABELS=(); DEFACT=()
  local f
  # 1) im Editor gebaute / roboterfreie Umgebungen
  for f in scenes/*.xml; do
    [[ -e "$f" ]] || continue
    SCENES+=("$f"); LABELS+=("Umgebung   $f"); DEFACT+=("e")
  done
  # 2) lauffaehige G1-Szenen (Roboter + Objekte) aus dem g1-Ordner
  for f in ../unitree_robots/g1/scene_g1_playground.xml; do
    [[ -e "$f" ]] || continue
    SCENES+=("$f"); LABELS+=("[G1]       $f"); DEFACT+=("v")
  done
}

# --- interaktives Menue ----------------------------------------------
menu() {
  collect_scenes
  echo ""
  echo "=================== MuJoCo Scene Editor ==================="
  echo "Verfuegbare Szenen:"
  if [[ ${#SCENES[@]} -eq 0 ]]; then
    echo "   (noch keine Szenen in scenes/ - waehle 'n' fuer eine neue)"
  else
    local i
    for i in "${!SCENES[@]}"; do
      printf "   %2d) %s\n" "$((i + 1))" "${LABELS[$i]}"
    done
  fi
  echo "    n) neue leere Szene im Editor"
  echo "    q) beenden"
  echo "----------------------------------------------------------"
  local sel
  read -rp "Auswahl (Zahl / n / q): " sel

  case "$sel" in
    q|Q|"") exit 0 ;;
    n|N)    new_scene ;;
    *[!0-9]*)
      echo "Ungueltige Eingabe." >&2; exit 1 ;;
    *)
      local idx=$((sel - 1))
      if (( idx < 0 || idx >= ${#SCENES[@]} )); then
        echo "Ungueltige Nummer." >&2; exit 1
      fi
      local chosen="${SCENES[$idx]}" def="${DEFACT[$idx]}" act
      echo ""
      echo "Gewaehlt: $chosen"
      read -rp "Aktion [e=Editor bearbeiten / v=Viewer ansehen] (Default $def): " act
      act="${act:-$def}"
      case "$act" in
        v|V) view_scene "$chosen" ;;
        e|E) edit_scene "$chosen" ;;
        *)   echo "Ungueltige Aktion." >&2; exit 1 ;;
      esac
      ;;
  esac
}

# --- Dispatch --------------------------------------------------------
CMD="${1:-menu}"; shift || true

case "$CMD" in
  menu)    menu ;;
  new)     exec "$PY" run_editor.py new "$@" ;;
  edit)    exec "$PY" run_editor.py edit "${1:-scenes/environment_starter.xml}" ;;
  prompt)  exec "$PY" run_editor.py prompt "$@" ;;
  view)    view_scene "${1:-scenes/environment_starter.xml}" ;;
  view-g1) view_scene "$G1_SCENE" ;;
  *)
    echo "Unbekanntes Kommando: $CMD" >&2
    echo "Benutze: (ohne Argument) | new | edit [datei] | prompt \"text\" | view [datei] | view-g1" >&2
    exit 1
    ;;
esac
