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

PY="${PYTHON:-python3}"
VENV=".venv"

echo ">> Erstelle virtualenv in $VENV ..."
"$PY" -m venv "$VENV"

# pip/setuptools/wheel aktualisieren.
# WICHTIG: ohne aktuelles setuptools scheitert der Build der Abhaengigkeit
# 'GPUtil' (AttributeError: install_layout) mit dem alten System-setuptools.
echo ">> Aktualisiere pip / setuptools / wheel ..."
"$VENV/bin/pip" install --upgrade pip setuptools wheel

echo ">> Installiere mujoco-scene-editor (+ yourdfpy, + STEP-Import) ... das dauert einen Moment."
"$VENV/bin/pip" install -r requirements.txt

# STEP-Import pruefen (cadquery-ocp ist ein grosses Wheel; wenn es fehlt, soll
# das hier auffallen und nicht erst beim Hochladen einer CAD-Datei).
if "$VENV/bin/python" step_import.py --help >/dev/null 2>&1 \
   && "$VENV/bin/python" -c "import sys; sys.path.insert(0,'.'); import step_import; sys.exit(0 if step_import.available_backends() else 1)"; then
  echo ">> STEP/STP-Import ist aktiv."
else
  echo ">> WARNUNG: STEP-Import inaktiv (cadquery-ocp fehlt)."
  echo "   Nachinstallieren:  $VENV/bin/pip install cadquery-ocp"
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
