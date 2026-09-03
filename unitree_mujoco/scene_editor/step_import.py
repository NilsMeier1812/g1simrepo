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


def is_step_file(path) -> bool:
    """True, wenn der Dateiname auf .step/.stp endet."""
    return Path(path).suffix.lower() in STEP_SUFFIXES


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


def _convert_with_occ(src: Path, dest: Path, scale: float, quality: str) -> None:
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
    if hasattr(writer, "SetASCIIMode"):
        writer.SetASCIIMode(False)      # pythonocc
    else:
        writer.ASCIIMode = False        # OCP: binaeres STL, mag MuJoCo lieber
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


def _convert_with_gmsh(src: Path, dest: Path, scale: float, quality: str) -> None:
    import gmsh

    rel_lin, _ang = QUALITY[quality]
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


def convert_step_to_stl(
    src,
    dest=None,
    scale: float = DEFAULT_SCALE,
    quality: str = DEFAULT_QUALITY,
    overwrite: bool = True,
) -> Path:
    """Konvertiert eine STEP/STP-Datei in ein binaeres STL.

    src      : Pfad zur STEP-Datei
    dest     : Ziel-STL (Default: gleicher Name mit .stl in meshes/)
    scale    : Skalierung beim Konvertieren (Default 0.001 = mm -> m)
    quality  : "coarse" | "normal" | "fine" (Feinheit der Tesselierung)
    overwrite: False -> vorhandenes STL wird wiederverwendet (kein Neu-Bauen)

    Gibt den Pfad zum erzeugten STL zurueck.
    """
    src = Path(src).expanduser().resolve()
    if not src.is_file():
        raise FileNotFoundError(f"STEP-Datei nicht gefunden: {src}")
    if quality not in QUALITY:
        raise ValueError(f"Unbekannte Qualitaet {quality!r}, erlaubt: {sorted(QUALITY)}")

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
            fn(src, dest, float(scale), quality)
        except Exception as exc:  # naechstes Backend probieren
            errors.append(f"{name}: {exc}")
            continue
        if dest.is_file() and dest.stat().st_size > 0:
            return dest
        errors.append(f"{name}: leere Ausgabedatei")

    if not errors:
        raise RuntimeError(NO_BACKEND_HINT)
    raise RuntimeError(
        "STEP-Konvertierung fehlgeschlagen:\n  " + "\n  ".join(errors)
    )


def convert_folder(folder=MESHES_DIR, overwrite: bool = False, **kwargs) -> list[Path]:
    """Konvertiert alle STEP-Dateien eines Ordners nach STL (fuer 'Scan assets').

    Ohne overwrite werden nur STEPs konvertiert, zu denen noch kein STL
    existiert bzw. deren STL aelter ist als die STEP-Datei.
    """
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
    args = ap.parse_args(argv)

    backends = available_backends()
    if not backends:
        print(NO_BACKEND_HINT, file=sys.stderr)
        return 2

    try:
        if args.step:
            out = convert_step_to_stl(args.step, args.out, args.scale, args.quality)
            print(f"OK: {out}")
        else:
            made = convert_folder(overwrite=args.force, scale=args.scale,
                                  quality=args.quality)
            if not made:
                print(f"Nichts zu tun - keine neuen STEP-Dateien in {MESHES_DIR}")
            for out in made:
                print(f"OK: {out}")
    except Exception as exc:
        print(f"Fehler: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
