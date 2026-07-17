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


# ---------------------------------------------------------------------------
# Zusaetzlicher Upload-Button: echter Datei-Dialog des Browsers.
# Der eingebaute Import ("Add Assets from File") scannt nur einen Ordner. Fuer
# "Datei aus beliebigem Ordner auswaehlen" haengen wir per viser-Upload-Button
# einen zweiten Weg an: ausgewaehlte Datei wird nach meshes/ gespeichert und
# direkt in die Szene eingefuegt. Umgesetzt ohne Aenderung am Fremdpaket, indem
# wir die Editor-Fabrik umschliessen.
# ---------------------------------------------------------------------------
_UPLOAD_EXTS = ".stl,.obj,.ply,.glb,.gltf,.STL,.OBJ,.PLY,.GLB,.GLTF"


def _install_upload_button(editor) -> None:
    server = editor.layout.server
    try:
        with server.gui.add_folder("Eigene Datei hochladen", expand_by_default=True):
            up = server.gui.add_upload_button(
                "STL/OBJ waehlen ...", mime_type=_UPLOAD_EXTS,
                hint="Datei aus beliebigem Ordner waehlen; wird nach meshes/ "
                     "kopiert und in die Szene eingefuegt.",
            )
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Upload-Button nicht verfuegbar: {exc}", file=sys.stderr)
        return

    @up.on_upload
    def _on_upload(event) -> None:
        f = up.value
        if not f or not f.name:
            return
        dest = MESHES_DIR / Path(f.name).name
        try:
            dest.write_bytes(f.content)
            editor.controller.create_mesh(editor.get_selected_parent(), dest.resolve())
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] Upload fehlgeschlagen: {exc}", file=sys.stderr)
            return
        try:
            event.client.add_notification(
                title="Mesh eingefuegt",
                body=f"{dest.name} nach meshes/ gespeichert und in die Szene gelegt.",
                loading=False,
            )
        except Exception:
            pass


_orig_get_scene_editor = _editor_cli.get_scene_editor


def _get_scene_editor_with_upload(blueprints=None):
    editor = _orig_get_scene_editor(blueprints)
    _install_upload_button(editor)
    return editor


_editor_cli.get_scene_editor = _get_scene_editor_with_upload


if __name__ == "__main__":
    _editor_cli.cli()
