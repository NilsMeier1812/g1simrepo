#!/usr/bin/env python3
"""
STEP-Import fuer den Scene Editor: STEP/STP -> STL (binaer, in Metern).

Warum ueberhaupt konvertieren?
-----------------------------
MuJoCo kann **kein** STEP laden. STEP ist ein CAD-Format mit exakten Flaechen
(BRep/NURBS), MuJoCo braucht dagegen ein Dreiecksnetz (STL/OBJ/MSH). Auch der
Editor (viser/robits) rendert nur Dreiecksnetze. Der Import muss die STEP-
Geometrie also **tesselieren** (in Dreiecke zerlegen) - genau das macht dieses
Modul, transparent im Hintergrund beim Hochladen.

Backends (das erste verfuegbare wird genommen):
  1. **OCP** (`cadquery-ocp`, OpenCascade-Python-Bindings) - reines pip-Paket,
     braucht keine zusaetzlichen System-Bibliotheken. Standard, wird von
     setup.sh installiert. (`pythonocc-core` mit `OCC.Core.*` geht auch.)
  2. **gmsh** (`pip install gmsh`) - Fallback; braucht System-Libs
     (libGLU/libXft ...), liefert dafuer ein sauber vernetztes Oberflaechennetz.

Einheiten
---------
CAD/STEP rechnet praktisch immer in **Millimetern**, MuJoCo in **Metern**.
Darum wird beim Konvertieren per Default mit **0.001** skaliert, das Ergebnis
ist direkt in Metern und passt ohne weiteres `scale`-Gefummel in die Szene.
Mit `--scale 1` bleibt die Datei in Original-Einheiten.

Aufruf als CLI:
    python3 step_import.py bauteil.step                 # -> meshes/bauteil.stl
    python3 step_import.py bauteil.stp -o /tmp/x.stl --quality fine
    python3 step_import.py --scale 1 bauteil.step       # ohne mm->m
"""
from __future__ import annotations

import argparse
import importlib
import re
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
MESHES_DIR = HERE / "meshes"

#: Endungen, die als STEP gelten (Gross-/Kleinschreibung egal).
STEP_SUFFIXES = (".step", ".stp")

#: Voreinstellung: CAD ist in mm, MuJoCo in m.
DEFAULT_SCALE = 0.001

#: Tesselierungs-Guete: (lineare Abweichung relativ zur Bauteil-Diagonale,
#: Winkelabweichung in rad). Kleiner = feiner = mehr Dreiecke.
QUALITY = {
    "coarse": (0.005, 0.7),
    "normal": (0.002, 0.4),
    "fine": (0.0005, 0.2),
}
DEFAULT_QUALITY = "normal"

#: MuJoCos harte Obergrenze fuer STL-Dreiecke. Darueber bricht das Laden mit
#: "stl_decoder: number of faces should be between 1 and 200000" ab - dieselbe
#: Meldung, die MuJoCo auch fuer ASCII-STL ausgibt (es liest den ASCII-Text als
#: Face-Zaehler und landet ausserhalb des Bereichs). Beides faengt dieses Modul
#: ab, siehe make_mujoco_ready() bzw. die Vergroeberungs-Schleife in
#: convert_step_to_stl().
MJ_MAX_FACES = 200_000

#: Mesh-Formate, die MuJoCo laden kann. Der Editor zeigt zwar mehr an (er
#: rendert per viser), beim Export in die Szene faellt alles andere durch.
MJ_MESH_SUFFIXES = (".stl", ".obj", ".msh")

#: Faktoren, mit denen die Tesselierung nacheinander vergroebert wird, wenn ein
#: Bauteil/eine Baugruppe ueber MJ_MAX_FACES landet.
_COARSEN_STEPS = (1.0, 2.5, 6.0, 15.0, 40.0, 100.0)


def is_step_file(path) -> bool:
    """True, wenn der Dateiname auf .step/.stp endet."""
    return Path(path).suffix.lower() in STEP_SUFFIXES


# ---------------------------------------------------------------------------
# STL-Pruefung/Reparatur (MuJoCo laedt nur binaeres STL mit <= 200000 Faces)
# ---------------------------------------------------------------------------
def stl_face_count(path) -> int | None:
    """Face-Anzahl einer BINAEREN STL - None, wenn die Datei nicht binaer ist.

    Binaeres STL: 80-Byte-Header + 4-Byte-Face-Anzahl + Faces*50 Byte, exakt
    passend zur Dateigroesse. ASCII-STL erfuellt das praktisch nie.
    """
    try:
        with open(path, "rb") as fh:
            head = fh.read(84)
            size = Path(path).stat().st_size
    except OSError:
        return None
    if len(head) < 84:
        return None
    ntri = struct.unpack_from("<I", head, 80)[0]
    if size != 84 + ntri * 50:
        return None
    return ntri


def is_binary_stl(path) -> bool:
    """True, wenn die Datei als binaere STL gelesen werden kann."""
    return stl_face_count(path) is not None


def ascii_stl_to_binary(src, dest=None) -> int:
    """ASCII-STL nach binaerem STL wandeln. Gibt die Face-Anzahl zurueck.

    Bewusst ohne trimesh/numpy: dieselbe Funktion wird auch vom blanken
    System-python3 aus benutzt (start.sh -> build_env_scene.py), wo im Zweifel
    gar nichts installiert ist.
    """
    src = Path(src)
    dest = Path(dest) if dest is not None else src
    text = src.read_text(encoding="utf-8", errors="replace")

    tris = []
    num = r"[-+0-9.eEdD]+"
    facet_re = re.compile(
        r"facet\s+normal\s+(%s)\s+(%s)\s+(%s).*?outer\s+loop(.*?)endloop" % (num, num, num),
        re.IGNORECASE | re.DOTALL)
    vertex_re = re.compile(r"vertex\s+(%s)\s+(%s)\s+(%s)" % (num, num, num), re.IGNORECASE)

    def _f(s: str) -> float:
        # Fortran-Exponenten (1.0D-3) kommen in aelteren CAD-Exporten vor.
        return float(s.replace("D", "E").replace("d", "e"))

    for m in facet_re.finditer(text):
        verts = vertex_re.findall(m.group(4))
        if len(verts) != 3:
            continue
        normal = tuple(_f(m.group(i)) for i in (1, 2, 3))
        tris.append((normal, [tuple(_f(c) for c in v) for v in verts]))

    if not tris:
        raise ValueError(f"{src.name}: keine Dreiecke gefunden (keine gueltige ASCII-STL?)")

    out = bytearray(b"\0" * 80)
    out += struct.pack("<I", len(tris))
    for normal, verts in tris:
        out += struct.pack("<3f", *normal)
        for v in verts:
            out += struct.pack("<3f", *v)
        out += struct.pack("<H", 0)
    dest.write_bytes(bytes(out))
    return len(tris)


def stl_problem(path) -> str | None:
    """Warum MuJoCo diese STL NICHT laden kann (sonst None) - im Klartext."""
    faces = stl_face_count(path)
    if faces is None:
        return (f"'{Path(path).name}' ist keine binaere STL (MuJoCo kann nur "
                "binaeres STL - die Meldung 'perhaps this is an ASCII file?' "
                "kommt genau daher).")
    if faces > MJ_MAX_FACES:
        # Tausenderpunkte (deutsch) nur an den Zahlen, nicht am Dateinamen.
        have = f"{faces:,}".replace(",", ".")
        limit = f"{MJ_MAX_FACES:,}".replace(",", ".")
        return (f"'{Path(path).name}' hat {have} Dreiecke - MuJoCos Grenze "
                f"liegt bei {limit}.")
    if faces < 1:
        return f"'{Path(path).name}' enthaelt keine Dreiecke."
    return None


def make_mujoco_ready(path, notes=None) -> str | None:
    """STL fuer MuJoCo brauchbar machen: ASCII -> binaer, sonst Problem melden.

    Rueckgabe: None, wenn die Datei (danach) ladbar ist, sonst der Grund als
    Text. Hinweise ueber durchgefuehrte Reparaturen landen in `notes`.
    """
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix != ".stl":
        if suffix not in MJ_MESH_SUFFIXES:
            return (f"'{path.name}': MuJoCo kann dieses Mesh-Format nicht laden "
                    f"(nur {', '.join(MJ_MESH_SUFFIXES)}). Die Datei in einem "
                    "Mesh-Programm als STL/OBJ exportieren.")
        return None                      # OBJ/MSH prueft MuJoCo selbst
    if stl_face_count(path) is None:
        try:
            faces = ascii_stl_to_binary(path)
        except (OSError, ValueError, struct.error) as exc:
            return f"'{path.name}' laesst sich nicht als STL lesen ({exc})."
        if notes is not None:
            notes.append(f"{path.name} war eine ASCII-STL und wurde binaer neu "
                         f"geschrieben ({faces} Dreiecke).")
    return stl_problem(path)


# ---------------------------------------------------------------------------
# Backend 1: OpenCascade (OCP / pythonocc-core)
# ---------------------------------------------------------------------------
def _occ_module(name: str):
    """Modul aus OCP (cadquery-ocp) oder OCC.Core (pythonocc-core) holen."""
    try:
        return importlib.import_module(f"OCP.{name}")
    except ImportError:
        return importlib.import_module(f"OCC.Core.{name}")


def _have_occ() -> bool:
    try:
        _occ_module("STEPControl")
        return True
    except ImportError:
        return False


def _convert_with_occ(src: Path, dest: Path, scale: float, quality: str,
                      coarsen: float = 1.0) -> None:
    STEPControl = _occ_module("STEPControl")
    IFSelect = _occ_module("IFSelect")
    BRepMesh = _occ_module("BRepMesh")
    StlAPI = _occ_module("StlAPI")
    Bnd = _occ_module("Bnd")
    BRepBndLib = _occ_module("BRepBndLib")
    gp = _occ_module("gp")
    BRepBuilderAPI = _occ_module("BRepBuilderAPI")

    reader = STEPControl.STEPControl_Reader()
    status = reader.ReadFile(str(src))
    # OCP kapselt den Enum in eine Klasse, pythonocc legt ihn aufs Modul.
    ret_done = getattr(
        getattr(IFSelect, "IFSelect_ReturnStatus", IFSelect),
        "IFSelect_RetDone",
    )
    if status != ret_done:
        raise RuntimeError(f"STEP-Datei konnte nicht gelesen werden: {src}")
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape is None or shape.IsNull():
        raise RuntimeError(f"STEP-Datei enthaelt keine Geometrie: {src}")

    if scale != 1.0:
        trsf = gp.gp_Trsf()
        trsf.SetScale(gp.gp_Pnt(0.0, 0.0, 0.0), float(scale))
        shape = BRepBuilderAPI.BRepBuilderAPI_Transform(shape, trsf, True).Shape()

    # Lineare Abweichung relativ zur Bauteilgroesse: sonst ist ein fester Wert
    # bei kleinen Teilen zu grob und bei grossen zu fein (Millionen Dreiecke).
    rel_lin, ang = QUALITY[quality]
    # coarsen > 1: absichtlich groebere Tesselierung (Baugruppe sprengt sonst
    # MuJoCos Face-Limit). Der Winkel darf dabei nicht ueber ~1.2 rad gehen,
    # sonst werden Rundungen zu Kanten-Kraut.
    rel_lin *= float(coarsen)
    ang = min(ang * float(coarsen) ** 0.5, 1.2)
    box = Bnd.Bnd_Box()
    # OCP: BRepBndLib.Add_s(...) | pythonocc: brepbndlib_Add(...) bzw. brepbndlib.Add
    for adder in (
        getattr(getattr(BRepBndLib, "BRepBndLib", None), "Add_s", None),
        getattr(BRepBndLib, "brepbndlib_Add", None),
        getattr(getattr(BRepBndLib, "brepbndlib", None), "Add", None),
    ):
        if adder is not None:
            adder(shape, box)
            break
    else:
        raise RuntimeError("BRepBndLib.Add nicht gefunden (unbekannte OCC-Bindings)")
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    diag = max(
        ((xmax - xmin) ** 2 + (ymax - ymin) ** 2 + (zmax - zmin) ** 2) ** 0.5,
        1e-9,
    )
    lin = max(diag * rel_lin, 1e-9)

    BRepMesh.BRepMesh_IncrementalMesh(shape, lin, False, ang, True)

    writer = StlAPI.StlAPI_Writer()
    # Binaeres STL erzwingen - OpenCascade schreibt sonst ASCII, das MuJoCo
    # nicht laedt. Je nach Bindings heisst das anders; greift keine Variante,
    # repariert make_mujoco_ready() das Ergebnis hinterher.
    if hasattr(writer, "SetASCIIMode"):
        writer.SetASCIIMode(False)      # pythonocc
    else:
        try:
            writer.ASCIIMode = False    # OCP (Property)
        except AttributeError:
            pass
    if not writer.Write(shape, str(dest)):
        raise RuntimeError(f"STL konnte nicht geschrieben werden: {dest}")


# ---------------------------------------------------------------------------
# Backend 2: gmsh
# ---------------------------------------------------------------------------
def _have_gmsh() -> bool:
    try:
        importlib.import_module("gmsh")
        return True
    except Exception:
        # gmsh wirft OSError, wenn System-Libs (libGLU ...) fehlen.
        return False


def _convert_with_gmsh(src: Path, dest: Path, scale: float, quality: str,
                       coarsen: float = 1.0) -> None:
    import gmsh

    rel_lin, _ang = QUALITY[quality]
    rel_lin *= float(coarsen)
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add(src.stem)
        gmsh.model.occ.importShapes(str(src))
        gmsh.model.occ.synchronize()

        box = gmsh.model.getBoundingBox(-1, -1)
        diag = max(
            ((box[3] - box[0]) ** 2 + (box[4] - box[1]) ** 2 + (box[5] - box[2]) ** 2) ** 0.5,
            1e-9,
        )
        gmsh.option.setNumber("Mesh.MeshSizeMax", diag * rel_lin * 20)
        gmsh.option.setNumber("Mesh.MeshSizeMin", 0.0)
        gmsh.model.mesh.generate(2)      # nur Oberflaechennetz - mehr will MuJoCo nicht

        gmsh.option.setNumber("Mesh.ScalingFactor", float(scale))
        gmsh.option.setNumber("Mesh.Binary", 1)
        gmsh.write(str(dest))
    finally:
        gmsh.finalize()


BACKENDS = (
    ("ocp", _have_occ, _convert_with_occ),
    ("gmsh", _have_gmsh, _convert_with_gmsh),
)

NO_BACKEND_HINT = (
    "Kein STEP-Backend installiert. Im Editor-venv nachinstallieren:\n"
    "    cd unitree_mujoco/scene_editor && .venv/bin/pip install cadquery-ocp\n"
    "(oder ./setup.sh erneut laufen lassen). Alternativ das STEP vorher in "
    "einem CAD-Programm als STL exportieren."
)


def available_backends() -> list[str]:
    """Namen der aktuell nutzbaren Konverter-Backends."""
    return [name for name, have, _fn in BACKENDS if have()]


def backend_installed() -> bool:
    """Schnell-Check, ob ueberhaupt ein Backend-Paket im venv liegt.

    Nutzt find_spec statt eines echten Imports - OCP zu importieren dauert
    spuerbar. Fuer launch.sh/setup.sh, die das bei jedem Start pruefen.
    """
    from importlib.util import find_spec

    for mod in ("OCP", "OCC", "gmsh"):
        try:
            if find_spec(mod) is not None:
                return True
        except (ImportError, ValueError):
            continue
    return False


def _convert_within_limits(fn, src: Path, dest: Path, scale: float,
                           quality: str, notes) -> None:
    """Ein Backend so oft (vergroebert) laufen lassen, bis MuJoCo das STL mag.

    Grosse Baugruppen sprengen bei "normal"/"fine" muehelos MuJoCos Grenze von
    200000 Dreiecken. Statt den Nutzer mit "stl_decoder: number of faces ..."
    stehen zu lassen, wird die Tesselierung schrittweise vergroebert und das
    Ergebnis am Ende gemeldet.
    """
    last = None
    for coarsen in _COARSEN_STEPS:
        fn(src, dest, float(scale), quality, coarsen)
        if not (dest.is_file() and dest.stat().st_size > 0):
            raise RuntimeError("leere Ausgabedatei")
        problem = make_mujoco_ready(dest, notes)
        faces = stl_face_count(dest)
        if problem is None:
            if coarsen > 1.0:
                notes.append(
                    f"{src.name}: Baugruppe war zu fein fuer MuJoCo - "
                    f"Tesselierung um Faktor {coarsen:g} vergroebert "
                    f"({faces} Dreiecke).")
            return
        last = problem
        if faces is None:      # kein Face-Problem -> Vergroebern hilft nicht
            break
    raise RuntimeError(
        f"{last}\n"
        "  Auch die groebste Tesselierung reicht nicht. Moeglichkeiten:\n"
        "  - im Ordner 'STEP/CAD-Import' die Genauigkeit auf 'coarse' stellen\n"
        "  - die Baugruppe im CAD in einzelne Bauteile aufteilen und einzeln laden\n"
        "  - nur die wirklich gebrauchten Bauteile exportieren "
        "(Schrauben/Innenleben weglassen)")


def convert_step_to_stl(
    src,
    dest=None,
    scale: float = DEFAULT_SCALE,
    quality: str = DEFAULT_QUALITY,
    overwrite: bool = True,
    notes=None,
) -> Path:
    """Konvertiert eine STEP/STP-Datei in ein binaeres STL.

    src      : Pfad zur STEP-Datei
    dest     : Ziel-STL (Default: gleicher Name mit .stl in meshes/)
    scale    : Skalierung beim Konvertieren (Default 0.001 = mm -> m)
    quality  : "coarse" | "normal" | "fine" (Feinheit der Tesselierung)
    overwrite: False -> vorhandenes STL wird wiederverwendet (kein Neu-Bauen)
    notes    : optionale Liste, in die Hinweise (Vergroeberung, ASCII-Reparatur)
               geschrieben werden

    Das Ergebnis ist garantiert etwas, das MuJoCo laden kann: binaeres STL mit
    hoechstens MJ_MAX_FACES Dreiecken - sonst fliegt ein RuntimeError mit
    Klartext-Begruendung.
    """
    src = Path(src).expanduser().resolve()
    if not src.is_file():
        raise FileNotFoundError(f"STEP-Datei nicht gefunden: {src}")
    if quality not in QUALITY:
        raise ValueError(f"Unbekannte Qualitaet {quality!r}, erlaubt: {sorted(QUALITY)}")
    if notes is None:
        notes = []

    if dest is None:
        MESHES_DIR.mkdir(parents=True, exist_ok=True)
        dest = MESHES_DIR / (src.stem + ".stl")
    dest = Path(dest).expanduser().resolve()
    dest.parent.mkdir(parents=True, exist_ok=True)

    if dest.is_file() and not overwrite:
        return dest

    errors = []
    for name, have, fn in BACKENDS:
        if not have():
            continue
        try:
            _convert_within_limits(fn, src, dest, scale, quality, notes)
        except Exception as exc:  # naechstes Backend probieren
            errors.append(f"{name}: {exc}")
            continue
        return dest

    if not errors:
        raise RuntimeError(NO_BACKEND_HINT)
    raise RuntimeError(
        "STEP-Konvertierung fehlgeschlagen:\n  " + "\n  ".join(errors)
    )


def convert_folder(folder=MESHES_DIR, overwrite: bool = False, notes=None,
                   **kwargs) -> list[Path]:
    """Konvertiert alle STEP-Dateien eines Ordners nach STL (fuer 'Scan assets').

    Ohne overwrite werden nur STEPs konvertiert, zu denen noch kein STL
    existiert bzw. deren STL aelter ist als die STEP-Datei. Hinweise (z.B.
    "musste vergroebert werden") landen in `notes`.
    """
    if notes is not None:
        kwargs["notes"] = notes
    folder = Path(folder)
    made = []
    for src in sorted(folder.iterdir() if folder.is_dir() else []):
        if not src.is_file() or not is_step_file(src):
            continue
        dest = folder / (src.stem + ".stl")
        if (not overwrite and dest.is_file()
                and dest.stat().st_mtime >= src.stat().st_mtime):
            continue
        made.append(convert_step_to_stl(src, dest, **kwargs))
    return made


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="STEP/STP -> STL (binaer) fuer MuJoCo/Scene Editor.")
    ap.add_argument("step", nargs="?", help="STEP-Datei (ohne Angabe: alle in meshes/)")
    ap.add_argument("-o", "--out", help="Ziel-STL (Default: meshes/<name>.stl)")
    ap.add_argument("--scale", type=float, default=DEFAULT_SCALE,
                    help=f"Skalierung (Default {DEFAULT_SCALE} = mm->m; 1 = unveraendert)")
    ap.add_argument("--quality", choices=sorted(QUALITY), default=DEFAULT_QUALITY,
                    help="Feinheit der Tesselierung (Default: %(default)s)")
    ap.add_argument("--force", action="store_true",
                    help="vorhandene STLs neu erzeugen (nur im Ordner-Modus relevant)")
    ap.add_argument("--check", action="store_true",
                    help="nur pruefen, ob ein STEP-Backend installiert ist "
                         "(Exit 0 = ja, 2 = nein); von launch.sh/setup.sh genutzt")
    ap.add_argument("--check-meshes", action="store_true",
                    help="alle STL in meshes/ auf MuJoCo-Tauglichkeit pruefen "
                         "(binaer, <= 200000 Dreiecke) und ASCII reparieren")
    args = ap.parse_args(argv)

    if args.check:
        return 0 if backend_installed() else 2

    if args.check_meshes:
        return _check_meshes(Path(args.step) if args.step else MESHES_DIR)

    backends = available_backends()
    if not backends:
        print(NO_BACKEND_HINT, file=sys.stderr)
        return 2

    notes = []
    try:
        if args.step:
            out = convert_step_to_stl(args.step, args.out, args.scale, args.quality,
                                      notes=notes)
            print(f"OK: {out}  ({stl_face_count(out)} Dreiecke)")
        else:
            made = convert_folder(overwrite=args.force, scale=args.scale,
                                  quality=args.quality, notes=notes)
            if not made:
                print(f"Nichts zu tun - keine neuen STEP-Dateien in {MESHES_DIR}")
            for out in made:
                print(f"OK: {out}  ({stl_face_count(out)} Dreiecke)")
    except Exception as exc:
        print(f"Fehler: {exc}", file=sys.stderr)
        return 1
    for note in notes:
        print(f"Hinweis: {note}")
    return 0


def _check_meshes(target: Path) -> int:
    """Alle STL unter `target` pruefen (und ASCII gleich reparieren)."""
    files = sorted(target.rglob("*.stl")) if target.is_dir() else [target]
    if not files:
        print(f"Keine STL-Dateien in {target}")
        return 0
    bad = 0
    for f in files:
        notes = []
        problem = make_mujoco_ready(f, notes)
        for note in notes:
            print(f"repariert: {note}")
        if problem:
            bad += 1
            print(f"PROBLEM  : {problem}", file=sys.stderr)
        else:
            print(f"ok       : {f.name} ({stl_face_count(f)} Dreiecke)")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
