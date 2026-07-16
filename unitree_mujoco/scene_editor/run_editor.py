#!/usr/bin/env python
"""
Wrapper um die mujoco-scene-editor CLI.

Zweck: den Export-Pfad fest auf  scene_editor/scenes/  vorbelegen. Dadurch
steht im Export-Feld des Editors schon der richtige Ordner - man muss beim
Speichern nur noch den Namen anpassen (z.B. scene.xml -> kueche.xml) und auf
"Export scene" klicken. Die Datei landet dann automatisch unter scenes/.

Aufruf wie die normale CLI:
    python run_editor.py new
    python run_editor.py edit scenes/environment_starter.xml
    python run_editor.py prompt "eine Kueche"
"""
import sys
from pathlib import Path

# Ziel-Ordner fuer exportierte Szenen (fest verdrahtet, neben diesem Skript)
SCENES_DIR = (Path(__file__).resolve().parent / "scenes")
SCENES_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_TARGET = str(SCENES_DIR / "scene.xml")

# Den vorbelegten Export-/Default-Pfad in allen Modulen setzen, die ihn beim
# Aufbauen der GUI lesen. Wir patchen die schon importierten Modul-Globals,
# damit es unabhaengig von der Importreihenfolge wirkt.
import mujoco_scene_editor.constants as _constants
_constants.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET

import mujoco_scene_editor.layout as _layout
_layout.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET

import mujoco_scene_editor.cli.editor_cli as _editor_cli
_editor_cli.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET


if __name__ == "__main__":
    _editor_cli.cli()
