#!/usr/bin/env python
"""
Wrapper um die mujoco-scene-editor CLI.

Zweck (zwei fest verdrahtete Pfade, damit der Editor out-of-the-box passt):

1. EXPORT-Pfad auf  scene_editor/scenes/  vorbelegen. Beim Speichern muss nur
   noch der Name angepasst werden (z.B. scene.xml -> kueche.xml) + "Export
   scene"; die Datei landet automatisch unter scenes/.

2. ASSET-/MESH-Ordner auf  scene_editor/meshes/  vorbelegen. Der Mesh-Import
   des Editors ("Add Assets from File") scannt ein Verzeichnis nach
   STL/OBJ/... - mit diesem Default tauchen die STLs aus meshes/ sofort in der
   Auswahl auf (Ordner aufklappen -> "Scan assets" -> auswaehlen -> "Add
   asset"). Der eingebaute Default (~/temp/ArmarXObjects) existiert sonst nicht,
   dann ist die Liste leer und es wirkt, als gaebe es keinen Import.

Aufruf wie die normale CLI:
    python run_editor.py new
    python run_editor.py edit scenes/environment_starter.xml
    python run_editor.py prompt "eine Kueche"
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

# Ziel-Ordner fuer exportierte Szenen (fest verdrahtet, neben diesem Skript)
SCENES_DIR = HERE / "scenes"
SCENES_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_TARGET = str(SCENES_DIR / "scene.xml")

# Ordner, aus dem der Editor eigene Meshes (STL/OBJ/...) importiert.
MESHES_DIR = HERE / "meshes"
MESHES_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_ASSET_DIR = str(MESHES_DIR)

# Die vorbelegten Pfade in allen Modulen setzen, die sie beim Aufbauen der GUI
# lesen. Wir patchen die schon importierten Modul-Globals, damit es unabhaengig
# von der Importreihenfolge wirkt.
import mujoco_scene_editor.constants as _constants
_constants.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET
_constants.DEFAULT_ASSET_DIR = DEFAULT_ASSET_DIR

import mujoco_scene_editor.layout as _layout
_layout.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET
_layout.DEFAULT_ASSET_DIR = DEFAULT_ASSET_DIR

import mujoco_scene_editor.cli.editor_cli as _editor_cli
_editor_cli.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET


if __name__ == "__main__":
    _editor_cli.cli()
