#!/usr/bin/env bash
# =====================================================================
# Starter fuer den mujoco-scene-editor und den MuJoCo-Viewer.
#
# Zentraler Umgebungs-Ordner:  scene_editor/scenes/
#   -> Dieselben Umgebungen sind hier UND beim G1-Start (g1pilot/start.sh)
#      waehlbar. Neue Umgebungen einfach im Editor unter scenes/ speichern.
#
# Ohne Argument -> interaktives Menue: listet alle Umgebungen nummeriert auf;
# pro Umgebung kannst du: bearbeiten (Editor), allein ansehen, oder MIT dem
# G1 ansehen.
#
#   ./launch.sh                 interaktives Menue (empfohlen)
#   ./launch.sh new             leere Umgebung im Editor starten
#   ./launch.sh edit <datei>    bestimmte Umgebung im Editor oeffnen
#   ./launch.sh prompt "text"   Umgebung per Text-Prompt generieren (API-Key)
#   ./launch.sh view <datei>    Umgebung allein im MuJoCo-Viewer ansehen
#   ./launch.sh with-g1 <datei> Umgebung + G1 im MuJoCo-Viewer ansehen
#   ./launch.sh view-g1         statisches Beispiel scene_g1_playground.xml
#   ./launch.sh convert [datei] STEP/STP -> STL (ohne Datei: alle in meshes/)
#
# Der Editor oeffnet einen lokalen Webserver (http://127.0.0.1:8080).
# Exportierte Umgebungen landen automatisch in scenes/ (siehe run_editor.py).
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
SCENES_DIR="scenes"
G1_PLAYGROUND="../unitree_robots/g1/scene_g1_playground.xml"

# --- Selbstheilung: fehlende Teile im venv nachinstallieren ------------
# Aeltere venvs (vor dem STEP-Import) haben cadquery-ocp nicht. Statt den
# Nutzer im Editor mit "kein Backend installiert" stehen zu lassen, holen wir
# es hier einmalig nach. Schlaegt das fehl (kein Internet), laeuft der Editor
# trotzdem - nur eben ohne STEP.
ensure_step_backend() {
  if "$PY" step_import.py --check >/dev/null 2>&1; then
    return 0
  fi
  echo ">> STEP/CAD-Import fehlt im venv - installiere cadquery-ocp nach"
  echo "   (einmalig, ~70 MB; danach nie wieder)."
  "$VENV/bin/pip" install cadquery-ocp || true
  if "$PY" step_import.py --check >/dev/null 2>&1; then
    echo ">> STEP-Import ist jetzt aktiv."
  else
    echo ">> WARNUNG: Nachinstallation fehlgeschlagen (kein Internet?)." >&2
    echo "   Der Editor startet trotzdem, kann aber nur STL/OBJ - keine STEP." >&2
  fi
}

edit_scene() { ensure_step_backend; exec "$PY" run_editor.py edit "$1"; }
new_scene()  { ensure_step_backend; exec "$PY" run_editor.py new; }
view_scene() { exec "$PY" -m mujoco.viewer --mjcf="$1"; }

# Umgebung + G1 kombinieren und im Viewer ansehen (ohne Docker-Stack).
view_with_g1() {
  local env="$1"
  local out
  if ! out=$("$PY" build_env_scene.py --env "$env" --inspire "${G1_INSPIRE_HANDS:-0}"); then
    echo "Konnte kombinierte Szene nicht erzeugen." >&2; exit 1
  fi
  echo "Kombiniert: $out"
  exec "$PY" -m mujoco.viewer --mjcf="$out"
}

# --- alle Umgebungen aus dem zentralen Ordner einsammeln -------------
collect_envs() {
  ENVS=()
  local f
  for f in "$SCENES_DIR"/*.xml; do
    [[ -e "$f" ]] || continue
    ENVS+=("$f")
  done
}

# --- interaktives Menue ----------------------------------------------
menu() {
  collect_envs
  echo ""
  echo "=================== MuJoCo Scene Editor ==================="
  echo "Umgebungen in scene_editor/${SCENES_DIR}/"
  echo "(dieselben, die auch beim G1-Start via g1pilot/start.sh waehlbar sind)"
  echo "----------------------------------------------------------"
  if [[ ${#ENVS[@]} -eq 0 ]]; then
    echo "   (noch keine Umgebungen - waehle 'n' fuer eine neue)"
  else
    local i
    for i in "${!ENVS[@]}"; do
      printf "   %2d) %s\n" "$((i + 1))" "$(basename "${ENVS[$i]}" .xml)"
    done
  fi
  echo "    n) neue leere Umgebung im Editor"
  echo "    q) beenden"
  echo "----------------------------------------------------------"
  local sel
  read -rp "Auswahl (Zahl / n / q): " sel

  case "$sel" in
    q|Q|"") exit 0 ;;
    n|N)    new_scene ;;
    *[!0-9]*) echo "Ungueltige Eingabe." >&2; exit 1 ;;
    *)
      local idx=$((sel - 1))
      if (( idx < 0 || idx >= ${#ENVS[@]} )); then
        echo "Ungueltige Nummer." >&2; exit 1
      fi
      local chosen="${ENVS[$idx]}" act
      echo ""
      echo "Gewaehlt: $(basename "$chosen" .xml)"
      echo "Aktion:"
      echo "   e) im Editor bearbeiten"
      echo "   v) allein im Viewer ansehen (ohne Roboter)"
      echo "   g) mit dem G1 im Viewer ansehen"
      read -rp "Auswahl [e/v/g] (Default e): " act
      act="${act:-e}"
      case "$act" in
        e|E) edit_scene "$chosen" ;;
        v|V) view_scene "$chosen" ;;
        g|G) view_with_g1 "$chosen" ;;
        *)   echo "Ungueltige Aktion." >&2; exit 1 ;;
      esac
      ;;
  esac
}

# --- Dispatch --------------------------------------------------------
CMD="${1:-menu}"; shift || true

case "$CMD" in
  menu)    menu ;;
  new)     ensure_step_backend; exec "$PY" run_editor.py new "$@" ;;
  edit)    edit_scene "${1:-$SCENES_DIR/environment_starter.xml}" ;;
  prompt)  ensure_step_backend; exec "$PY" run_editor.py prompt "$@" ;;
  view)    view_scene "${1:-$SCENES_DIR/environment_starter.xml}" ;;
  with-g1) view_with_g1 "${1:-$SCENES_DIR/environment_starter.xml}" ;;
  view-g1) view_scene "$G1_PLAYGROUND" ;;
  convert) ensure_step_backend; exec "$PY" step_import.py "$@" ;;
  *)
    echo "Unbekanntes Kommando: $CMD" >&2
    echo "Benutze: (ohne Argument) | new | edit [datei] | prompt \"text\" | view [datei] | with-g1 [datei] | view-g1 | convert [datei]" >&2
    exit 1
    ;;
esac
