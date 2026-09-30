#!/usr/bin/env bash
# =====================================================================
# Einmaliges Setup fuer den mujoco-scene-editor.
# Legt ein eigenes virtualenv (.venv) an und installiert alles darin,
# damit die Systeminstallation nicht angefasst wird.
#
#   ./setup.sh
#
# Danach: ./launch.sh edit
# =====================================================================
set -euo pipefail
cd "$(dirname "$0")"

# mujoco-scene-editor verlangt Python 3.10-3.12. Neuere Distributionen
# (z.B. Ubuntu 26.04: 3.14) bringen ein zu neues python3 mit - dann ein
# passendes pythonX.Y nehmen, falls installiert, sonst klar abbrechen.
py_ok() { "$1" -c 'import sys; sys.exit(0 if (3, 10) <= sys.version_info[:2] < (3, 13) else 1)' 2>/dev/null; }
PY="${PYTHON:-}"
if [[ -z "$PY" ]]; then
  for c in python3 python3.12 python3.11 python3.10; do
    if command -v "$c" >/dev/null 2>&1 && py_ok "$c"; then PY="$c"; break; fi
  done
fi
if [[ -z "$PY" ]] || ! py_ok "$PY"; then
  echo "FEHLER: kein passendes Python gefunden (gebraucht: 3.10-3.12, vorhanden:" >&2
  echo "        $(python3 --version 2>&1))." >&2
  echo "  - python3.12 installieren und  PYTHON=python3.12 ./setup.sh" >&2
  echo "  - oder den Editor im Container nutzen:  cd g1pilot && make editor" >&2
  exit 1
fi
# Ort des venv: Default .venv/ hier im Ordner. Das Docker-Image
# (g1pilot/docker/Dockerfile.scene_editor) legt es nach /opt, damit es nicht im
# gemounteten Repo landet.
VENV="${SCENE_EDITOR_VENV:-.venv}"

echo ">> Erstelle virtualenv in $VENV ..."
"$PY" -m venv "$VENV"

# pip/setuptools/wheel aktualisieren.
# WICHTIG: ohne aktuelles setuptools scheitert der Build der Abhaengigkeit
# 'GPUtil' (AttributeError: install_layout) mit dem alten System-setuptools.
echo ">> Aktualisiere pip / setuptools / wheel ..."
"$VENV/bin/pip" install --upgrade pip setuptools wheel

echo ">> Installiere mujoco-scene-editor (+ yourdfpy, + CAD-Import) ... das dauert einen Moment."
"$VENV/bin/pip" install -r requirements.txt

# CAD-Import pruefen (cadquery-ocp ist ein grosses Wheel; wenn es fehlt, soll
# das hier auffallen und nicht erst beim Hochladen einer CAD-Datei).
if "$VENV/bin/python" cad_import.py --check >/dev/null 2>&1; then
  echo ">> CAD-Import (STEP/IGES) ist aktiv."
else
  echo ">> WARNUNG: CAD-Import inaktiv (cadquery-ocp fehlt)."
  echo "   Nachinstallieren:  $VENV/bin/pip install cadquery-ocp coacd"
fi

# Persistente RoBits-Config (sonst meckert der Editor und nutzt /tmp)
mkdir -p ".robits_config"

echo ""
echo "============================================================"
echo " Setup fertig."
echo ""
echo " Naechste Schritte:"
echo "   ./launch.sh edit     # Beispiel-Umgebung im Browser bearbeiten"
echo "   ./launch.sh new      # leere Szene starten"
echo "   ./launch.sh view-g1  # G1 + Objekte im MuJoCo-Viewer ansehen"
echo ""
echo " Hinweis: Beim allerersten Start laedt der Editor einmalig den"
echo " Objaverse-Objektkatalog aus dem Netz (braucht Internet, wird"
echo " danach gecacht)."
echo "============================================================"
