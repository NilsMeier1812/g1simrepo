#!/usr/bin/env python
"""
Wrapper um die mujoco-scene-editor CLI.

Warum ueberhaupt ein Wrapper? Der Editor ist ein Fremdpaket; hier wird er auf
dieses Repo eingestellt und um genau das ergaenzt, was fuers Umgebungen-Bauen
fehlt - ohne das Paket zu patchen:

1. SPEICHERN NUR MIT NAMEN. Der eingebaute Export fragt nach einem kompletten
   DATEIPFAD ("~/temp/export/scene.json"). Das ist die haeufigste Fehlerquelle:
   ein Tippfehler im Pfad und die Umgebung landet irgendwo, wo sie niemand
   findet. Hier gibt es stattdessen oben den Ordner "Umgebung speichern" mit
   EINEM Feld: dem Namen. Gespeichert wird immer nach  scene_editor/scenes/ .

2. MESHES AUS scenes/ HERAUS PORTABEL. Beim Speichern werden Mesh-Dateien, die
   ausserhalb des Repos liegen, nach  scene_editor/meshes/imported/  kopiert und
   im XML relativ referenziert - sonst findet der (read-only gemountete)
   Docker-Container sie spaeter nicht.

3. UMBENENNEN. Der eingebaute Editor kann Objekte nicht umbenennen - dabei
   entscheidet genau der Name, ob ein Objekt ein Hindernis oder ein GREIFBARES
   Objekt ist (Praefix "grasp_", siehe build_env_scene.py). Dafuer gibt es hier
   den Ordner "Objekt umbenennen".

4. UPLOAD-BUTTON + MESH-SKALIERUNG (echter Datei-Dialog bzw. Faktor fuer STLs).

5. ASSET-/MESH-Ordner auf  scene_editor/meshes/  vorbelegt (der eingebaute
   Default ~/temp/ArmarXObjects existiert nicht - dann wirkt der Ordner-Scan,
   als gaebe es keinen Import).

6. STEP/STP-IMPORT (CAD). MuJoCo und der Editor koennen nur Dreiecksnetze,
   kein CAD-BRep - darum wird STEP beim Import automatisch nach STL tesseliert
   (siehe step_import.py). Das gilt fuer den Upload-Button, fuer den Ordner
   meshes/ (STEPs dort werden beim Start konvertiert) und ueber den GUI-Ordner
   "STEP/CAD-Import" (Skalierung/Genauigkeit + Sammel-Konvertierung).

7. MESH-TUERSTEHER. Jedes Mesh wird geprueft, BEVOR es in die Szene kommt:
   ASCII-STL wird binaer neu geschrieben, zu feine CAD-Netze werden beim
   Konvertieren vergroebert, und was MuJoCo trotzdem nicht laden koennte
   (> 200000 Dreiecke, fremdes Format), wird gar nicht erst eingefuegt. Sonst
   steckt ein unladbares Objekt in der Szene und auch das Speichern scheitert.

Aufruf wie die normale CLI:
    python run_editor.py new
    python run_editor.py edit scenes/environment_starter.xml
    python run_editor.py prompt "eine Kueche"
"""
import os
import re
import shutil
import socket
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent

# Ziel-Ordner fuer exportierte Szenen (fest verdrahtet, neben diesem Skript)
SCENES_DIR = HERE / "scenes"
SCENES_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_NAME = "meine_umgebung"
DEFAULT_TARGET = str(SCENES_DIR / f"{DEFAULT_NAME}.xml")

# Ordner, aus dem der Editor eigene Meshes (STL/OBJ/...) importiert.
MESHES_DIR = HERE / "meshes"
MESHES_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_ASSET_DIR = str(MESHES_DIR)

# Der Editor-Webserver (viser) haengt an diesem Port. Muss zu der URL passen,
# die g1_gui.py/launch.sh im Browser oeffnen.
EDITOR_PORT = int(os.environ.get("SCENE_EDITOR_PORT", "8080"))
os.environ.setdefault("_VISER_PORT_OVERRIDE", str(EDITOR_PORT))

# Gemeinsame Helfer mit dem Szenen-Generator (liegt im selben Ordner).
sys.path.insert(0, str(HERE))
import build_env_scene as bes  # noqa: E402


def _port_in_use(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", port))
        except OSError:
            return True
    return False


if _port_in_use(EDITOR_PORT):
    sys.exit(
        f"[run_editor] Port {EDITOR_PORT} ist schon belegt - vermutlich laeuft der Editor\n"
        f"             bereits (Browser: http://127.0.0.1:{EDITOR_PORT}).\n"
        f"             In der GUI: 'Laufende Prozesse' -> Stoppen. Oder einen anderen\n"
        f"             Port nehmen: SCENE_EDITOR_PORT=8081 ./launch.sh ...")


# Die vorbelegten Pfade in allen Modulen setzen, die sie beim Aufbauen der GUI
# lesen. Wir patchen die schon importierten Modul-Globals, damit es unabhaengig
# von der Importreihenfolge wirkt.
import mujoco_scene_editor.constants as _constants  # noqa: E402
_constants.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET
_constants.DEFAULT_ASSET_DIR = DEFAULT_ASSET_DIR

import mujoco_scene_editor.layout as _layout  # noqa: E402
_layout.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET
_layout.DEFAULT_ASSET_DIR = DEFAULT_ASSET_DIR

import mujoco_scene_editor.cli.editor_cli as _editor_cli  # noqa: E402
_editor_cli.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET

import mujoco_scene_editor.scene_editor as _scene_editor_mod  # noqa: E402


# ---------------------------------------------------------------------------
# Ohne Internet ueberhaupt starten koennen.
# Der Editor laedt beim Start den Objaverse-Online-Katalog (Objekt-Bibliothek).
# Faellt der aus (kein/gesperrtes Internet, HTTP 403), stirbt sonst der GANZE
# Editor - obwohl man ihn zum Bauen eigener Umgebungen gar nicht braucht.
# ---------------------------------------------------------------------------
_orig_load_inventory = _scene_editor_mod.SceneEditor._load_inventory


def _load_inventory_offline_tolerant(self):
    try:
        _orig_load_inventory(self)
        return
    except Exception as exc:
        print(f"[run_editor] Objekt-Katalog (Objaverse) nicht erreichbar: {exc}\n"
              "             Editor laeuft weiter - eigene Shapes/STLs gehen normal, "
              "nur die Online-Bibliothek bleibt leer.", file=sys.stderr)
    # Lokale Meshes trotzdem anbieten, Online-Liste leer lassen.
    try:
        items = self.inventory.list(
            root=Path(self.layout.assets_dir.value.strip()).expanduser())
        self.layout.asset_items = {m.name: m for m in items}
    except Exception:
        self.layout.asset_items = {}
    self.layout.update_assets_dropdown()
    self.layout.objaverse_labels = ()
    self.layout.update_objaverse_label_dropdown()
    self.layout.objaverse_items = {}
    self.layout.update_objaverse_dropdown()


_scene_editor_mod.SceneEditor._load_inventory = _load_inventory_offline_tolerant


# ---------------------------------------------------------------------------
# Namen
# ---------------------------------------------------------------------------
_UMLAUTS = {"ä": "ae", "ö": "oe", "ü": "ue", "ß": "ss",
            "Ä": "Ae", "Ö": "Oe", "Ü": "Ue"}


def sanitize_env_name(raw: str) -> str:
    """Aus einer Nutzereingabe einen sauberen Umgebungs-Namen machen.

    Der Name wird zum Dateinamen UND zum Wert von G1_ENV (Env-Variable, die per
    docker-compose weitergereicht wird) - deshalb bewusst streng: nur
    Buchstaben/Ziffern/_/-, keine Pfade, keine Leerzeichen, keine Umlaute.
    Rueckgabe "" heisst: unbrauchbar.
    """
    name = (raw or "").strip()
    name = Path(name).name                      # evtl. mitgetippte Pfade weg
    for k, v in _UMLAUTS.items():
        name = name.replace(k, v)
    if name.lower().endswith((".xml", ".json")):
        name = name.rsplit(".", 1)[0]
    name = re.sub(r"\s+", "_", name)
    name = re.sub(r"[^A-Za-z0-9_\-]", "_", name)
    name = re.sub(r"_{2,}", "_", name).strip("_-")
    return name


def _initial_name_from_argv() -> str:
    """Beim Bearbeiten einer vorhandenen Umgebung deren Namen vorbelegen."""
    argv = sys.argv[1:]
    if len(argv) >= 2 and argv[0] == "edit":
        name = sanitize_env_name(Path(argv[1]).stem)
        if name:
            return name
    return DEFAULT_NAME


INITIAL_NAME = _initial_name_from_argv()


# ---------------------------------------------------------------------------
# Speichern (Export) - nur der Name wird gefragt
# ---------------------------------------------------------------------------
def _relocate_meshes(xml_path: Path, notes: list) -> None:
    """Mesh-Referenzen im exportierten XML repo-relativ machen.

    Der Exporter schreibt ABSOLUTE Pfade (z.B. /home/du/Downloads/kiste.stl).
    Auf einem anderen Rechner - und im Docker-Container, der nur
    unitree_mujoco/ sieht - existieren die nicht. Also: Datei nach
    scene_editor/meshes/ (bzw. meshes/imported/) holen und im XML als
    "../meshes/..." referenzieren (relativ zu scenes/, wie im Starter-Beispiel).
    """
    try:
        tree = ET.parse(xml_path)
    except (OSError, ET.ParseError) as exc:
        notes.append(f"XML nicht nachbearbeitbar: {exc}")
        return

    root = tree.getroot()
    changed = False
    for asset in root.findall("asset"):
        for mesh in asset.findall("mesh"):
            file_attr = mesh.get("file")
            if not file_attr:
                continue
            src = Path(os.path.expanduser(file_attr))
            if not src.is_absolute():
                src = (xml_path.parent / src)
            try:
                src = src.resolve()
            except OSError:
                continue
            if not src.is_file():
                notes.append(f"Mesh fehlt: {file_attr}")
                continue
            try:
                inside = src.is_relative_to(MESHES_DIR)
            except AttributeError:  # Python < 3.9
                inside = str(src).startswith(str(MESHES_DIR))
            if inside:
                dest = src
            else:
                bes.IMPORTED_MESHES_DIR.mkdir(parents=True, exist_ok=True)
                dest = bes.IMPORTED_MESHES_DIR / src.name
                try:
                    if not dest.is_file() or dest.stat().st_size != src.stat().st_size:
                        shutil.copy2(src, dest)
                    notes.append(f"{src.name} -> meshes/imported/")
                except OSError as exc:
                    notes.append(f"{src.name} konnte nicht kopiert werden: {exc}")
                    continue
            rel = os.path.relpath(dest, SCENES_DIR).replace(os.sep, "/")
            if rel != file_attr:
                mesh.set("file", rel)
                changed = True

    if changed:
        try:
            tree.write(xml_path, encoding="utf-8", xml_declaration=False)
        except OSError as exc:
            notes.append(f"XML nicht schreibbar: {exc}")


def _repair_orphan_parents(editor, notes: list) -> None:
    """Verwaiste Eltern-Pfade reparieren, BEVOR exportiert wird.

    Hintergrund (Bug im Fremdpaket 'robits', das der Editor fuers Exportieren
    nutzt): neue Objekte bekommen als Eltern-Pfad das, was gerade oben unter
    "Elements" ausgewaehlt ist. Ist dabei ein normales Objekt (Box/Mesh/...)
    statt eine ECHTE Gruppe ausgewaehlt, haengt der Editor das neue Objekt
    intern unter diesen Pfad (z.B. "/box_0001/box_0002"). Der MuJoCo-Exporter
    traegt aber NUR Gruppen (BlueprintGroup) in seine interne Eltern-Zuordnung
    ein -- fuer alles andere schlaegt die Suche fehl und der Export stuerzt mit
    "AttributeError: 'NoneType' object has no attribute 'joints'" ab (robits/
    sim/scene/mjcf_utils.py:has_freejoint auf einem nicht gefundenen Parent).

    Wir heben solche Objekte hier auf die oberste Ebene, statt den Export
    scheitern zu lassen - fuer unsere Umgebungen (nur Hindernisse/Greif-
    objekte, siehe scene_editor/README.md) macht flach vs. verschachtelt
    ohnehin keinen Unterschied: build_env_scene.py flacht beim Kombinieren
    sowieso alles ab.
    """
    from dataclasses import replace as _dc_replace

    from robits.sim.blueprints import BlueprintGroup

    ctrl = editor.controller
    blueprints = ctrl.state.blueprints
    top_level_groups = {
        p.strip("/").split("/", 1)[0]
        for p, bp in blueprints.items()
        if isinstance(bp, BlueprintGroup)
    }
    used_top_names = {p.strip("/").split("/", 1)[0] for p in blueprints}

    fixed = {}
    for path, bp in blueprints.items():
        parts = path.strip("/").split("/")
        if len(parts) <= 1 or parts[0] in top_level_groups:
            continue
        leaf = parts[-1] or "objekt"
        new_path = f"/{leaf}"
        n = 2
        while new_path in blueprints or new_path in fixed or \
                new_path.strip("/") in used_top_names:
            new_path = f"/{leaf}_{n}"
            n += 1
        fixed[new_path] = _dc_replace(bp, path=new_path)
        used_top_names.add(new_path.strip("/"))
        notes.append(f"'{path}' hatte keinen gueltigen Eltern-Pfad mehr "
                     f"-> auf oberste Ebene gehoben als '{new_path}'.")

    if not fixed:
        return

    ctrl.state.push_state_to_history()
    for old_path in list(blueprints):
        parts = old_path.strip("/").split("/")
        if len(parts) > 1 and parts[0] not in top_level_groups:
            del blueprints[old_path]
    blueprints.update(fixed)
    ctrl.renderer.render_from_state(list(blueprints.values()))


def _fix_bad_stl_meshes(editor, notes: list) -> None:
    """STL-Meshes reparieren, BEVOR robits die Szene zum Exportieren kompiliert.

    MuJoCo laedt nur BINAERE STL-Dateien. ASCII-STL (haeufig bei Assets aus
    Blender/Sketchfab/Objaverse-Downloads) laesst robits' Kompilierschritt mit
    "ValueError: ... stl_decoder: ... perhaps this is an ASCII file?" abstuerzen
    - noch bevor build_env_scene.py ueberhaupt ins Spiel kommt. Alle im
    aktuellen Editor-Stand referenzierten STLs werden deshalb hier schon
    geprueft und bei Bedarf binaer neu geschrieben (mit trimesh, das der Editor
    ohnehin als Abhaengigkeit mitbringt).
    """
    from robits.sim.blueprints import MeshBlueprint

    for bp in editor.controller.state.blueprints.values():
        if not isinstance(bp, MeshBlueprint):
            continue
        mesh_path = Path(bp.mesh_path)
        if mesh_path.suffix.lower() != ".stl" or not mesh_path.is_file():
            continue
        if bes.is_valid_binary_stl(mesh_path):
            continue
        bes.convert_stl_to_binary(mesh_path, notes)


def save_environment(editor, name: str):
    """Szene als scenes/<name>.xml speichern.

    Rueckgabe: (ok, titel, text). Es wird IMMER erst in einen temporaeren Ordner
    exportiert und nur das Ergebnis nach scenes/ verschoben - der Exporter legt
    naemlich zusaetzlich eine Datei "MuJoCo Model.xml" ab, die sonst als
    Geister-Umgebung in jeder Auswahlliste auftauchen wuerde.
    """
    clean = sanitize_env_name(name)
    if not clean:
        return False, "Name fehlt", (
            "Bitte einen Namen eingeben (Buchstaben, Ziffern, _ und -), "
            "z.B. 'kueche'.")

    target = SCENES_DIR / f"{clean}.xml"
    existed = target.is_file()
    notes = []

    try:
        _repair_orphan_parents(editor, notes)
        _fix_bad_stl_meshes(editor, notes)
        with tempfile.TemporaryDirectory(prefix="scene_export_") as td:
            tmp_xml = Path(td) / f"{clean}.xml"
            editor.controller.export_scene(tmp_xml)
            if not tmp_xml.is_file():
                return False, "Speichern fehlgeschlagen", \
                    "Der Editor hat keine XML-Datei erzeugt."
            _relocate_meshes(tmp_xml, notes)

            SCENES_DIR.mkdir(parents=True, exist_ok=True)
            shutil.move(str(tmp_xml), str(target))
            tmp_json = tmp_xml.with_suffix(".json")
            if tmp_json.is_file():
                shutil.move(str(tmp_json), str(target.with_suffix(".json")))
    except Exception as exc:  # pragma: no cover - Laufzeit
        return False, "Speichern fehlgeschlagen", f"{type(exc).__name__}: {exc}"

    # Kontrolle: laesst sich die Umgebung ueberhaupt laden?
    problem = bes.validate_scene(target)

    lines = [f"{'Ueberschrieben' if existed else 'Gespeichert'}: scenes/{clean}.xml"]
    lines.append("Beim Sim-Start unter 'Umgebung' als '%s' waehlbar." % clean)
    if notes:
        lines.append("Hinweise: " + "; ".join(notes))
    if problem:
        lines.append("ACHTUNG: MuJoCo kann die Datei nicht laden -> " + problem.strip())
        return False, "Gespeichert, aber fehlerhaft", "\n".join(lines)
    return True, "Umgebung gespeichert", "\n".join(lines)


def _notify(event, title, body):
    try:
        event.client.add_notification(title=title, body=body, loading=False)
    except Exception:
        print(f"[run_editor] {title}: {body}")


def _install_save_control(editor) -> None:
    """Ordner "Umgebung speichern" - EIN Feld: der Name."""
    server = editor.layout.server
    try:
        with server.gui.add_folder("Umgebung speichern", order=1.1,
                                   expand_by_default=True):
            txt = server.gui.add_text(
                "Name", initial_value=INITIAL_NAME,
                hint="Nur der Name, kein Pfad. Gespeichert wird immer nach "
                     "scene_editor/scenes/<name>.xml")
            btn = server.gui.add_button("Speichern", color="green")
            server.gui.add_markdown(
                "Ziel: `scene_editor/scenes/<name>.xml` — genau diese Namen "
                "stehen beim Sim-Start unter *Umgebung* zur Auswahl.")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Speichern-Control nicht verfuegbar: {exc}", file=sys.stderr)
        return

    # Der eingebaute Export-Pfad wird ausgeblendet (er verwirrt nur), bleibt aber
    # als Handle bestehen: "Launch MuJoCo" liest ihn aus.
    for handle in (editor.layout.export_path, editor.layout.btn_export_mj):
        try:
            handle.visible = False
        except Exception:
            pass
    editor.layout.export_path.value = str(SCENES_DIR / f"{INITIAL_NAME}.xml")

    @btn.on_click
    def _save(event) -> None:
        btn.disabled = True
        try:
            ok, title, body = save_environment(editor, txt.value)
            clean = sanitize_env_name(txt.value)
            if clean:
                txt.value = clean
                editor.layout.export_path.value = str(SCENES_DIR / f"{clean}.xml")
            print(f"[run_editor] {title}: {body.replace(chr(10), ' | ')}")
            _notify(event, title, body)
        finally:
            btn.disabled = False


# ---------------------------------------------------------------------------
# Umbenennen (macht die grasp_-Konvention ueberhaupt erst benutzbar)
# ---------------------------------------------------------------------------
def _install_rename_control(editor) -> None:
    from dataclasses import replace as _dc_replace

    from mujoco_scene_editor.constants import NO_SELECTION

    server = editor.layout.server
    ctrl = editor.controller
    try:
        with server.gui.add_folder("Objekt umbenennen", order=1.2,
                                   expand_by_default=True):
            txt = server.gui.add_text(
                "Neuer Name", initial_value="",
                hint="Oben unter 'Elements' ein Objekt waehlen. Praefix "
                     "'grasp_' = greifbares Objekt (beweglich), sonst Hindernis.")
            btn = server.gui.add_button("Umbenennen")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Umbenennen-Control nicht verfuegbar: {exc}", file=sys.stderr)
        return

    @editor.layout.element_list.on_update
    def _sync_name(_evt) -> None:
        sel = editor.layout.element_list.value
        if sel and sel != NO_SELECTION:
            txt.value = sel.rsplit("/", 1)[-1]

    @btn.on_click
    def _rename(event) -> None:
        old = editor.layout.element_list.value
        if not old or old == NO_SELECTION or old not in ctrl.state.blueprints:
            _notify(event, "Kein Objekt gewaehlt",
                    "Bitte oben unter 'Elements' ein Objekt auswaehlen.")
            return
        new_leaf = bes.sanitize_name(txt.value)
        if not new_leaf:
            _notify(event, "Name ungueltig",
                    "Erlaubt sind Buchstaben, Ziffern, _ und - (keine Slashes).")
            return
        parent = old.rsplit("/", 1)[0]
        new = f"{parent}/{new_leaf}"
        if new == old:
            return
        if new in ctrl.state.blueprints:
            _notify(event, "Name schon vergeben",
                    f"'{new_leaf}' gibt es in dieser Ebene bereits.")
            return

        try:
            ctrl.state.push_state_to_history()
            renamed = {}
            for path, bp in ctrl.state.blueprints.items():
                if path == old:
                    renamed[new] = _dc_replace(bp, path=new)
                elif path.startswith(old + "/"):
                    child = new + path[len(old):]
                    renamed[child] = _dc_replace(bp, path=child)
                else:
                    renamed[path] = bp
            ctrl.state.blueprints = renamed
            ctrl.renderer.render_from_state(list(ctrl.state.blueprints.values()))
            editor.layout.element_list.value = new
            ctrl.select(new)
        except Exception as exc:  # pragma: no cover - Laufzeit
            _notify(event, "Umbenennen fehlgeschlagen", f"{type(exc).__name__}: {exc}")
            return
        kind = "greifbares Objekt" if bes.is_grasp_name(new_leaf) else "Hindernis"
        _notify(event, "Umbenannt", f"{old.rsplit('/', 1)[-1]} -> {new_leaf}  ({kind})")


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
# Nur, was MuJoCo spaeter auch laden kann - der Editor selbst wuerde mehr
# rendern (PLY/GLB), beim Export in die Szene faellt das aber durch.
_MESH_EXTS = ",".join(step_import.MJ_MESH_SUFFIXES)
_STEP_EXTS = ",".join(step_import.STEP_SUFFIXES)
# Datei-Dialoge filtern nach exakter Endung - Gross- und Kleinschreibung
# beide anbieten, CAD-Programme schreiben gern ".STEP".
_ALL_EXTS = (_MESH_EXTS + "," + _STEP_EXTS).split(",")
_UPLOAD_EXTS = ",".join(_ALL_EXTS + [e.upper() for e in _ALL_EXTS])


# Konvertier-Einstellungen fuer STEP (im GUI-Ordner "STEP/CAD-Import" aenderbar)
_STEP_OPTS = {
    "scale": step_import.DEFAULT_SCALE,     # CAD ist mm, MuJoCo m
    "quality": step_import.DEFAULT_QUALITY,
}


def _prepare_upload(name: str, content: bytes):
    """Hochgeladene Datei nach meshes/ schreiben, STEP dabei nach STL wandeln.

    Gibt (mesh_pfad, hinweistext) zurueck. Das Ergebnis ist garantiert etwas,
    das MuJoCo laden kann - sonst RuntimeError mit Klartext. Wichtig, weil ein
    unbrauchbares Mesh sonst erst tief im Editor beim Kompilieren knallt
    ("stl_decoder: number of faces should be between 1 and 200000").
    """
    dest = MESHES_DIR / Path(name).name
    dest.write_bytes(content)
    notes = []
    if step_import.is_step_file(dest):
        mesh = step_import.convert_step_to_stl(
            dest, scale=_STEP_OPTS["scale"], quality=_STEP_OPTS["quality"],
            notes=notes)
        info = (f"{dest.name} ist eine CAD-Datei und wurde nach {mesh.name} "
                f"konvertiert (Skalierung {_STEP_OPTS['scale']}, "
                f"{step_import.stl_face_count(mesh)} Dreiecke).")
    else:
        mesh = dest
        problem = step_import.make_mujoco_ready(mesh, notes)
        if problem:
            raise RuntimeError(
                f"{problem}\n"
                "Das Mesh in einem CAD-/Mesh-Programm vereinfachen (Ziel: unter "
                f"{step_import.MJ_MAX_FACES} Dreiecke) und binaer als STL "
                "exportieren - oder gleich die STEP-Datei hochladen, die wird "
                "hier automatisch passend tesseliert.")
        info = f"{mesh.name} nach meshes/ gespeichert."
    if notes:
        info += " " + " ".join(notes)
    return mesh, info


def _install_upload_button(editor) -> None:
    server = editor.layout.server
    step_ok = bool(step_import.available_backends())
    label = "STL/OBJ/STEP waehlen ..." if step_ok else "STL/OBJ waehlen ..."
    hint = ("Datei aus beliebigem Ordner waehlen; wird nach meshes/ kopiert und "
            "in die Szene eingefuegt.")
    if step_ok:
        hint += " STEP/STP wird automatisch nach STL konvertiert."
    try:
        with server.gui.add_folder("Eigene Datei hochladen", order=1.3,
                                   expand_by_default=True):
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


def _broadcast(editor, title, body) -> None:
    """Meldung an alle offenen Browser-Tabs (fuer Stellen ohne GUI-Event)."""
    try:
        for client in editor.layout.server.get_clients().values():
            client.add_notification(title=title, body=body, loading=False)
    except Exception:
        pass


def _guard_create_mesh(editor) -> None:
    """Kein Mesh in die Szene lassen, das MuJoCo nicht laden kann.

    Alle Wege, ein Mesh einzufuegen (Upload-Knopf, "Add Assets from File" ->
    "Scan assets", Objaverse), laufen durch controller.create_mesh(). Das legt
    das Blueprint erst in den Szenen-Zustand und rendert es dann - fliegt beim
    Rendern ein Fehler ("stl_decoder: number of faces should be between 1 and
    200000"), steckt das kaputte Objekt schon in der Szene und auch das
    Speichern geht nicht mehr. Darum wird hier VORHER geprueft (und ASCII-STL
    gleich repariert).
    """
    ctrl = editor.controller
    orig = ctrl.create_mesh

    def create_mesh(parent_name, mesh_path, *args, **kwargs):
        notes = []
        try:
            problem = step_import.make_mujoco_ready(Path(mesh_path), notes)
        except Exception as exc:  # pragma: no cover - Laufzeit
            problem = f"{Path(mesh_path).name}: Pruefung fehlgeschlagen ({exc})"
        for note in notes:
            print(f"[run_editor] {note}")
        if problem:
            msg = (f"{problem} Das Objekt wurde NICHT eingefuegt (die Szene "
                   "bliebe sonst kaputt). Mesh vereinfachen oder die STEP-Datei "
                   "laden - die wird beim Import automatisch passend tesseliert.")
            print(f"[run_editor] Mesh abgelehnt: {problem}", file=sys.stderr)
            _broadcast(editor, "Mesh nicht verwendbar", msg)
            raise RuntimeError(msg)
        return orig(parent_name, mesh_path, *args, **kwargs)

    ctrl.create_mesh = create_mesh


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
        with server.gui.add_folder("Mesh skalieren", order=1.4,
                                   expand_by_default=True):
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
        try:
            factor = float(num.value)
        except (TypeError, ValueError):
            _notify(event, "Ungueltiger Faktor", "Bitte eine Zahl eingeben.")
            return
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
            _notify(event, "Skalieren fehlgeschlagen", f"{type(exc).__name__}: {exc}")
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
        notes = []
        try:
            made = step_import.convert_folder(
                MESHES_DIR, overwrite=True, notes=notes,
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
        body = (", ".join(f"{p.name} ({step_import.stl_face_count(p)} Dreiecke)"
                          for p in made)
                + " - jetzt unter 'Add Assets from File' -> 'Scan assets'.")
        if notes:
            body += "\n" + "\n".join(notes)
        _notify(event, f"{len(made)} STEP konvertiert", body)


def _convert_steps_at_startup() -> None:
    """Neue STEP-Dateien in meshes/ schon beim Start nach STL wandeln.

    So findet der eingebaute Ordner-Scan sie sofort, ohne dass man erst einen
    Knopf druecken muss. Bereits konvertierte (STL neuer als STEP) bleiben.
    """
    if step_import.available_backends():
        notes = []
        try:
            made = step_import.convert_folder(MESHES_DIR, overwrite=False, notes=notes)
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] STEP-Vorkonvertierung fehlgeschlagen: {exc}",
                  file=sys.stderr)
            made = []
        for out in made:
            print(f"[run_editor] STEP konvertiert -> {out.name} "
                  f"({step_import.stl_face_count(out)} Dreiecke)")
        for note in notes:
            print(f"[run_editor] {note}")
    _check_meshes_at_startup()


def _check_meshes_at_startup() -> None:
    """Alle STL in meshes/ vorab pruefen (ASCII reparieren, zu grosse melden).

    Der eingebaute Ordner-Scan ("Add Assets from File") bietet blind alles an,
    was in meshes/ liegt - ein ASCII-STL oder ein Netz mit ueber 200000
    Dreiecken laesst den Editor dann beim Einfuegen mit
    "stl_decoder: number of faces ..." abbrechen. Lieber vorher wissen.
    """
    for stl in sorted(MESHES_DIR.glob("*.stl")):
        notes = []
        try:
            problem = step_import.make_mujoco_ready(stl, notes)
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] {stl.name}: Pruefung fehlgeschlagen ({exc})",
                  file=sys.stderr)
            continue
        for note in notes:
            print(f"[run_editor] {note}")
        if not problem:
            continue
        # Gibt es die CAD-Quelle noch, wird einfach groeber neu tesseliert -
        # damit repariert sich ein zu feines STL aus einer aelteren Sitzung
        # beim naechsten Start von allein.
        if _reconvert_from_step(stl):
            continue
        print(f"[run_editor] WARNUNG: {problem}\n"
              f"             MuJoCo kann '{stl.name}' nicht laden - bitte "
              "nicht in die Szene einfuegen (Editor bricht sonst ab).",
              file=sys.stderr)


def _giveup_marker(stl: Path) -> Path:
    """Merker: aus dieser STEP-Datei kam schon einmal kein brauchbares STL."""
    return stl.with_name(f".{stl.name}.unconvertible")


def _reconvert_from_step(stl: Path) -> bool:
    """Zu feines/kaputtes STL aus der zugehoerigen STEP-Datei neu bauen.

    Bei einer grossen Baugruppe kostet das Minuten. Scheitert es, wird das
    vermerkt - sonst wuerde bei JEDEM Editor-Start dieselbe aussichtslose
    Konvertierung neu laufen und das Starten ewig dauern.
    """
    if not step_import.available_backends():
        return False
    for suffix in step_import.STEP_SUFFIXES:
        for src in (stl.with_suffix(suffix), stl.with_suffix(suffix.upper())):
            if not src.is_file():
                continue
            marker = _giveup_marker(stl)
            stamp = f"{src.stat().st_mtime}"
            if marker.is_file() and marker.read_text().strip() == stamp:
                print(f"[run_editor] {stl.name}: aus {src.name} laesst sich "
                      "kein MuJoCo-taugliches Netz erzeugen (schon versucht). "
                      f"Merker loeschen zum erneuten Versuch: {marker.name}",
                      file=sys.stderr)
                return False
            notes = []
            try:
                step_import.convert_step_to_stl(
                    src, stl, scale=_STEP_OPTS["scale"],
                    quality=_STEP_OPTS["quality"], notes=notes)
            except Exception as exc:  # pragma: no cover - Laufzeit
                print(f"[run_editor] {stl.name}: Neu-Konvertierung aus "
                      f"{src.name} fehlgeschlagen: {exc}", file=sys.stderr)
                try:
                    marker.write_text(stamp)
                except OSError:
                    pass
                return False
            marker.unlink(missing_ok=True)
            print(f"[run_editor] {stl.name} war fuer MuJoCo unbrauchbar und "
                  f"wurde aus {src.name} neu erzeugt "
                  f"({step_import.stl_face_count(stl)} Dreiecke).")
            for note in notes:
                print(f"[run_editor] {note}")
            return True
    return False


_orig_get_scene_editor = _editor_cli.get_scene_editor


def _get_scene_editor_with_extras(blueprints=None):
    editor = _orig_get_scene_editor(blueprints)
    _guard_create_mesh(editor)
    _install_save_control(editor)
    _install_rename_control(editor)
    _install_upload_button(editor)
    _install_mesh_scale_control(editor)
    _install_step_controls(editor)
    print(f"[run_editor] Editor laeuft: http://127.0.0.1:{EDITOR_PORT}")
    return editor


_editor_cli.get_scene_editor = _get_scene_editor_with_extras


# ---------------------------------------------------------------------------
# 'prompt': Umgebung per Text erzeugen UND danach im Editor oeffnen.
# Die eingebaute CLI schreibt nur eine Datei und gibt einen Hinweis aus - wer
# ueber launch.sh/GUI startet, wartet dann vergeblich auf den Editor.
# ---------------------------------------------------------------------------
def _run_prompt(rest) -> int:
    text = " ".join(a for a in rest if not a.startswith("-")).strip()
    if not text:
        print("[run_editor] Bitte eine Beschreibung angeben, z.B.:\n"
              '    ./launch.sh prompt "a kitchen with a table and two boxes"',
              file=sys.stderr)
        return 2
    if not os.environ.get("OPENAI_API_KEY"):
        print("[run_editor] Fuer 'prompt' wird ein OpenAI-API-Key gebraucht:\n"
              "    export OPENAI_API_KEY=sk-...\n"
              "Ohne Key: leere Umgebung starten und die Objekte selbst setzen.",
              file=sys.stderr)
        return 2

    base = sanitize_env_name(text)[:40] or "prompt"
    out = SCENES_DIR / f"{base}.xml"
    i = 2
    while out.exists():
        out = SCENES_DIR / f"{base}_{i}.xml"
        i += 1

    print(f"[run_editor] Erzeuge Umgebung aus Text -> {out.name} (kann dauern) ...")
    try:
        _editor_cli.cli.main(["prompt", "--output-model-name", str(out), text],
                             standalone_mode=False)
    except Exception as exc:
        print(f"[run_editor] Text-Generierung fehlgeschlagen: {exc}", file=sys.stderr)
        return 1
    if not out.is_file():
        print("[run_editor] Es wurde keine Umgebung erzeugt (Antwort nicht verwertbar).",
              file=sys.stderr)
        return 1

    print(f"[run_editor] Umgebung erzeugt: scenes/{out.name} - oeffne sie im Editor.")
    global INITIAL_NAME
    INITIAL_NAME = out.stem
    try:
        _editor_cli.cli.main(["edit", str(out)], standalone_mode=False)
    except Exception as exc:
        print(f"[run_editor] Editor konnte die erzeugte Umgebung nicht oeffnen: {exc}",
              file=sys.stderr)
        return 1
    return 0


def main() -> int:
    _convert_steps_at_startup()
    argv = sys.argv[1:]
    if argv and argv[0] == "prompt":
        return _run_prompt(argv[1:])
    if argv and argv[0] == "edit":
        target = Path(argv[1]) if len(argv) > 1 else None
        if target is not None and not target.is_file():
            found = bes.resolve_env_path(str(target))
            if found.is_file():
                sys.argv[2] = str(found)
            else:
                envs = ", ".join(bes.available_envs()) or "(keine)"
                print(f"[run_editor] Umgebung nicht gefunden: {target}\n"
                      f"             Vorhanden in scenes/: {envs}", file=sys.stderr)
                return 2
    _editor_cli.cli()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
