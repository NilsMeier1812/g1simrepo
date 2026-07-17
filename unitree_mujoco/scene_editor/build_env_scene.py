#!/usr/bin/env python3
"""
Kombiniert das FESTE G1-Modell mit einer im Editor gebauten UMGEBUNG.

Liest eine roboterfreie Umgebungsszene (z.B. scene_editor/scenes/<name>.xml)
und schreibt eine lauffaehige Szene in den g1-Ordner, die
  * den G1 unveraendert per <include> einbindet und
  * die Umgebung (Boden, Objekte, eigene STLs) uebernimmt.

Der G1 bleibt also immer gleich - nur die Welt drumherum wechselt.

Warum ein Generator und kein simples <include>? MuJoCos <compiler meshdir>
gilt global fuer die ganze Szene. Das Robotermodell setzt meshdir="meshes";
damit die Roboter-Meshes gefunden werden, muss die Szene im g1-Ordner liegen.
Dieses Skript schreibt deshalb die kombinierte Szene dorthin und rechnet die
Mesh-Pfade der Umgebung passend um - als RELATIVE Pfade, damit sie sowohl auf
dem Host als auch im (read-only gemounteten) Docker-Container stimmen.

Aufruf:
    python3 build_env_scene.py --env scenes/environment_starter.xml
    python3 build_env_scene.py --env scenes/warehouse.xml --inspire 1
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent          # .../unitree_mujoco/scene_editor
MJ_ROOT = HERE.parent                            # .../unitree_mujoco
G1_DIR = MJ_ROOT / "unitree_robots" / "g1"
G1_MESHDIR = G1_DIR / "meshes"                    # meshdir des Robotermodells

# Roboter-Basismodell je nach Hand-Variante (wie config.py)
ROBOT_FILES = {
    "0": "g1_29dof.xml",
    "1": "g1_29dof_inspire_ftp.xml",
}


def rel_to_meshdir(mesh_abs: Path) -> str:
    """Pfad relativ zu g1/meshes, plattform-neutral (mit '/')."""
    return os.path.relpath(mesh_abs, G1_MESHDIR).replace(os.sep, "/")


def main() -> None:
    ap = argparse.ArgumentParser(description="G1 + Umgebung zu lauffaehiger Szene kombinieren")
    ap.add_argument("--env", required=True,
                    help="Umgebungs-XML (roboterfrei), z.B. scenes/warehouse.xml")
    ap.add_argument("--inspire", default="0", choices=["0", "1"],
                    help="0 = Rubber-Hand-G1, 1 = Inspire-FTP-G1 (Default 0)")
    ap.add_argument("--out",
                    help="Zieldatei (Default: unitree_robots/g1/scene_env_<name>.xml)")
    args = ap.parse_args()

    env_path = Path(args.env)
    if not env_path.is_absolute():
        env_path = (Path.cwd() / env_path).resolve()
    if not env_path.is_file():
        sys.exit(f"[build_env_scene] Umgebung nicht gefunden: {env_path}")
    env_dir = env_path.parent

    robot_file = ROBOT_FILES[args.inspire]
    if not (G1_DIR / robot_file).is_file():
        sys.exit(f"[build_env_scene] Robotermodell fehlt: {G1_DIR / robot_file}")

    name = env_path.stem
    out_path = Path(args.out).resolve() if args.out else (G1_DIR / f"scene_env_{name}.xml")

    # Kommentare vor dem Parsen entfernen. MuJoCos Parser ist tolerant, Pythons
    # ElementTree aber strikt (z.B. sind '--'-Folgen in XML-Kommentaren ungueltig).
    # Kommentare brauchen wir in der generierten Datei ohnehin nicht.
    raw = env_path.read_text(encoding="utf-8")
    cleaned = re.sub(r"<!--.*?-->", "", raw, flags=re.DOTALL)
    root = ET.fromstring(cleaned)  # <mujoco>

    # Mesh-Pfade der Umgebung auf g1/meshes-relativ umschreiben.
    warnings = []
    for mesh in root.findall(".//asset/mesh"):
        f = mesh.get("file")
        if not f:
            continue
        p = Path(f)
        mesh_abs = p if p.is_absolute() else (env_dir / p).resolve()
        if not mesh_abs.is_file():
            warnings.append(f"  ! Mesh nicht gefunden: {f}  ({mesh_abs})")
        try:
            mesh_abs.relative_to(MJ_ROOT)
        except ValueError:
            warnings.append(f"  ! Mesh liegt ausserhalb von unitree_mujoco/ "
                            f"(im Docker-Container evtl. nicht sichtbar): {mesh_abs}")
        mesh.set("file", rel_to_meshdir(mesh_abs))

    # Kombinierte Szene aufbauen: Roboter zuerst, dann Umgebung.
    out = ET.Element("mujoco", {"model": f"g1_env_{name}"})
    ET.SubElement(out, "include", {"file": robot_file})
    for tag in ("statistic", "visual", "asset", "worldbody"):
        for el in root.findall(tag):
            out.append(el)

    ET.indent(out, space="  ")
    header = (
        "<!-- AUTO-GENERIERT von scene_editor/build_env_scene.py.\n"
        f"     Umgebung: {env_path.name}    Roboter: {robot_file}\n"
        "     NICHT von Hand editieren - wird bei jeder Umgebungs-Auswahl neu erzeugt.\n"
        "     Umgebung bearbeiten: scene_editor -> Editor -> neu generieren. -->\n"
    )
    out_path.write_text(header + ET.tostring(out, encoding="unicode") + "\n", encoding="utf-8")

    for w in warnings:
        print(w, file=sys.stderr)
    print(out_path)


if __name__ == "__main__":
    main()
