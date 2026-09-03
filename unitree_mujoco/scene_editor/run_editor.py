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

3. STEP/STP-IMPORT (CAD). MuJoCo und der Editor koennen nur Dreiecksnetze,
   kein CAD-BRep - darum wird STEP beim Import automatisch nach STL tesseliert
   (siehe step_import.py). Das gilt fuer den Upload-Button, fuer den Ordner
   meshes/ (STEPs dort werden beim Start konvertiert) und ueber den GUI-Ordner
   "STEP/CAD-Import" (Skalierung/Genauigkeit + Sammel-Konvertierung).

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


# STEP-Konverter (STEP/STP -> STL). Liegt neben diesem Skript.
sys.path.insert(0, str(HERE))
import step_import

# ---------------------------------------------------------------------------
# Zusaetzlicher Upload-Button: echter Datei-Dialog des Browsers.
# Der eingebaute Import ("Add Assets from File") scannt nur einen Ordner. Fuer
# "Datei aus beliebigem Ordner auswaehlen" haengen wir per viser-Upload-Button
# einen zweiten Weg an: ausgewaehlte Datei wird nach meshes/ gespeichert und
# direkt in die Szene eingefuegt. Umgesetzt ohne Aenderung am Fremdpaket, indem
# wir die Editor-Fabrik umschliessen.
#
# STEP/STP wird dabei automatisch nach STL konvertiert (MuJoCo und der Editor
# koennen nur Dreiecksnetze, kein CAD-BRep) - siehe step_import.py.
# ---------------------------------------------------------------------------
_MESH_EXTS = ".stl,.obj,.ply,.glb,.gltf"
_STEP_EXTS = ".step,.stp"
_UPLOAD_EXTS = ",".join(
    [e for e in (_MESH_EXTS + "," + _STEP_EXTS).split(",")]
    + [e.upper() for e in (_MESH_EXTS + "," + _STEP_EXTS).split(",")]
)


# Konvertier-Einstellungen fuer STEP (im GUI-Ordner "STEP/CAD-Import" aenderbar)
_STEP_OPTS = {
    "scale": step_import.DEFAULT_SCALE,     # CAD ist mm, MuJoCo m
    "quality": step_import.DEFAULT_QUALITY,
}


def _prepare_upload(name: str, content: bytes):
    """Hochgeladene Datei nach meshes/ schreiben, STEP dabei nach STL wandeln.

    Gibt (mesh_pfad, hinweistext) zurueck.
    """
    dest = MESHES_DIR / Path(name).name
    dest.write_bytes(content)
    if not step_import.is_step_file(dest):
        return dest, f"{dest.name} nach meshes/ gespeichert."
    stl = step_import.convert_step_to_stl(
        dest, scale=_STEP_OPTS["scale"], quality=_STEP_OPTS["quality"])
    return stl, (f"{dest.name} ist eine CAD-Datei und wurde nach {stl.name} "
                 f"konvertiert (Skalierung {_STEP_OPTS['scale']}).")


def _install_upload_button(editor) -> None:
    server = editor.layout.server
    step_ok = bool(step_import.available_backends())
    label = "STL/OBJ/STEP waehlen ..." if step_ok else "STL/OBJ waehlen ..."
    hint = ("Datei aus beliebigem Ordner waehlen; wird nach meshes/ kopiert und "
            "in die Szene eingefuegt.")
    if step_ok:
        hint += " STEP/STP wird automatisch nach STL konvertiert."
    try:
        with server.gui.add_folder("Eigene Datei hochladen", expand_by_default=True):
            up = server.gui.add_upload_button(
                label, mime_type=_UPLOAD_EXTS if step_ok else _MESH_EXTS,
                hint=hint,
            )
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Upload-Button nicht verfuegbar: {exc}", file=sys.stderr)
        return

    @up.on_upload
    def _on_upload(event) -> None:
        f = up.value
        if not f or not f.name:
            return
        if step_import.is_step_file(f.name) and not step_ok:
            _notify(event, "STEP nicht moeglich", step_import.NO_BACKEND_HINT)
            return
        try:
            mesh, info = _prepare_upload(f.name, f.content)
            editor.controller.create_mesh(editor.get_selected_parent(), mesh.resolve())
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] Upload fehlgeschlagen: {exc}", file=sys.stderr)
            _notify(event, "Import fehlgeschlagen", str(exc))
            return
        _notify(event, "Mesh eingefuegt", f"{info} In die Szene gelegt.")


def _notify(event, title, body):
    try:
        event.client.add_notification(title=title, body=body, loading=False)
    except Exception:
        pass


def _install_mesh_scale_control(editor) -> None:
    """Skalier-Control fuer Meshes (fehlt im eingebauten Editor).

    Der Properties-Panel des Editors kann nur Box/Zylinder/Kugel-Masse aendern,
    aber importierte Meshes/STLs nicht skalieren. Hier: gewaehltes Mesh oben
    unter "Elements" waehlen, Faktor eingeben, anwenden. Loest auch mm->m
    (CAD-STL in mm -> Faktor 0.001).
    """
    from robits.sim.blueprints import MeshBlueprint

    server = editor.layout.server
    ctrl = editor.controller
    try:
        with server.gui.add_folder("Mesh skalieren", expand_by_default=True):
            num = server.gui.add_number(
                "Faktor", initial_value=1.0, min=0.0001, max=10000.0, step=0.01,
                hint="Skaliert das oben gewaehlte Mesh. CAD-STL in mm -> 0.001.")
            btn = server.gui.add_button("Auf gewaehltes Mesh anwenden")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Skalier-Control nicht verfuegbar: {exc}", file=sys.stderr)
        return

    # Beim Auswaehlen eines Meshes den aktuellen Faktor ins Feld holen (viser
    # haengt zusaetzliche on_update-Callbacks an, ersetzt die vorhandenen nicht).
    @editor.layout.element_list.on_update
    def _sync_scale(_evt) -> None:
        bp = ctrl.state.blueprints.get(editor.layout.element_list.value)
        if isinstance(bp, MeshBlueprint):
            try:
                num.value = float(getattr(bp, "scale", 1.0) or 1.0)
            except Exception:
                pass

    @btn.on_click
    def _apply_scale(event) -> None:
        name = editor.layout.element_list.value
        bp = ctrl.state.blueprints.get(name)
        if not isinstance(bp, MeshBlueprint):
            _notify(event, "Kein Mesh gewaehlt",
                    "Bitte oben unter 'Elements' ein importiertes Mesh auswaehlen.")
            return
        factor = float(num.value)
        if factor <= 0:
            _notify(event, "Ungueltiger Faktor", "Faktor muss > 0 sein.")
            return
        try:
            ctrl.state.update(name, scale=factor)
            ctrl.renderer.render_from_state(list(ctrl.state.blueprints.values()))
            editor.layout.element_list.value = name  # Auswahl/Gizmo wiederherstellen
            ctrl.select(name)
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] Skalieren fehlgeschlagen: {exc}", file=sys.stderr)
            return
        _notify(event, "Mesh skaliert", f"Faktor {factor} angewendet.")


def _install_step_controls(editor) -> None:
    """Ordner "STEP/CAD-Import": Konvertier-Optionen + Sammel-Konvertierung.

    Der eingebaute Ordner-Scan ("Add Assets from File") kennt nur Mesh-Formate.
    Damit STEP-Dateien, die einfach nach meshes/ kopiert wurden, dort auftauchen,
    wandelt dieser Knopf sie alle nach STL - danach findet "Scan assets" sie.
    """
    server = editor.layout.server
    if not step_import.available_backends():
        try:
            with server.gui.add_folder("STEP/CAD-Import", expand_by_default=False):
                server.gui.add_markdown(
                    "STEP-Import inaktiv - kein Backend installiert.\n\n"
                    "`.venv/bin/pip install cadquery-ocp`")
        except Exception:
            pass
        return

    try:
        with server.gui.add_folder("STEP/CAD-Import", expand_by_default=False):
            scale = server.gui.add_number(
                "Skalierung", initial_value=float(_STEP_OPTS["scale"]),
                min=0.000001, max=1000.0, step=0.0001,
                hint="Beim Konvertieren angewendet. CAD ist meist in mm -> 0.001 "
                     "ergibt Meter. 1 = Einheiten unveraendert.")
            quality = server.gui.add_dropdown(
                "Genauigkeit", options=("coarse", "normal", "fine"),
                initial_value=str(_STEP_OPTS["quality"]),
                hint="Feinheit der Tesselierung (fine = mehr Dreiecke).")
            btn = server.gui.add_button("STEP-Dateien in meshes/ konvertieren")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] STEP-Controls nicht verfuegbar: {exc}", file=sys.stderr)
        return

    @scale.on_update
    def _sync_scale(_evt) -> None:
        _STEP_OPTS["scale"] = float(scale.value)

    @quality.on_update
    def _sync_quality(_evt) -> None:
        _STEP_OPTS["quality"] = str(quality.value)

    @btn.on_click
    def _convert_all(event) -> None:
        try:
            made = step_import.convert_folder(
                MESHES_DIR, overwrite=True,
                scale=_STEP_OPTS["scale"], quality=_STEP_OPTS["quality"])
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] STEP-Konvertierung fehlgeschlagen: {exc}",
                  file=sys.stderr)
            _notify(event, "Konvertierung fehlgeschlagen", str(exc))
            return
        if not made:
            _notify(event, "Nichts zu konvertieren",
                    "Keine STEP/STP-Dateien in meshes/ gefunden.")
            return
        _notify(event, f"{len(made)} STEP konvertiert",
                ", ".join(p.name for p in made)
                + " - jetzt unter 'Add Assets from File' -> 'Scan assets'.")


def _convert_steps_at_startup() -> None:
    """Neue STEP-Dateien in meshes/ schon beim Start nach STL wandeln.

    So findet der eingebaute Ordner-Scan sie sofort, ohne dass man erst einen
    Knopf druecken muss. Bereits konvertierte (STL neuer als STEP) bleiben.
    """
    if not step_import.available_backends():
        return
    try:
        made = step_import.convert_folder(MESHES_DIR, overwrite=False)
    except Exception as exc:  # pragma: no cover - Laufzeit
        print(f"[run_editor] STEP-Vorkonvertierung fehlgeschlagen: {exc}",
              file=sys.stderr)
        return
    for out in made:
        print(f"[run_editor] STEP konvertiert -> {out.name}")


_orig_get_scene_editor = _editor_cli.get_scene_editor


def _get_scene_editor_with_extras(blueprints=None):
    editor = _orig_get_scene_editor(blueprints)
    _install_upload_button(editor)
    _install_mesh_scale_control(editor)
    _install_step_controls(editor)
    return editor


_editor_cli.get_scene_editor = _get_scene_editor_with_extras


if __name__ == "__main__":
    _convert_steps_at_startup()
    _editor_cli.cli()
