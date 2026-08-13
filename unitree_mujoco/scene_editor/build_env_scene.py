#!/usr/bin/env python3
"""
Kombiniert die feste G1-BASIS mit einer UMGEBUNG zu einer lauffaehigen Szene.

Idee des Systems:
  * BASIS (kommt IMMER automatisch dazu, hier fest verdrahtet):
      - der G1 (per <include>, Hand-Variante je nach --inspire)
      - Lichtquelle
      - Boden + Skybox/Groundplane
      - der Weld "hold_base_weld" (haelt den G1 am Anfang an torso_link fest;
        wird vom Sim per Name gesteuert, siehe hold_base.py)
      - visual/statistic-Grundeinstellungen
  * UMGEBUNG (scene_editor/scenes/<name>.xml): enthaelt NUR Hindernisse bzw.
    Objekte zum Interagieren/Greifen - also nur <asset> (eigene Meshes) und
    die Objekte im <worldbody>. KEIN Roboter, KEIN Boden, KEIN Licht, KEIN Weld.

Dieses Skript erzeugt die kombinierte Szene  unitree_robots/g1/scene_env_<name>.xml
= BASIS + Objekte der Umgebung. Mesh-Pfade der Umgebung werden relativ
umgerechnet, damit sie auf dem Host UND im read-only gemounteten Docker-Container
stimmen.

Hindernis vs. Greif-Objekt (fuer Nav/IK, siehe g1pilot/docs/51_navigation_technik.md):
Klassifikation per Namenskonvention -- ein Objekt-Name, der mit "grasp_"
beginnt, wird HIER automatisch von einem statischen <geom> in einen FREIEN
Koerper umgewandelt (<body><freejoint/><geom/></body>): nur so kann MuJoCo es
beim Anfassen/Greifen bewegen (Masse/Traegheit werden von MuJoCo automatisch
aus Geometrie + Default-Dichte berechnet). Alle anderen Objekte bleiben
statische Hindernisse. Dieselbe Namenskonvention wird von
unitree_mujoco/simulate_python/scene_objects.py (Sim-Seite) und
g1pilot/g1pilot/navigation/scene_bridge.py (ROS-Seite) verwendet.

Aufruf:
    python3 build_env_scene.py --env scenes/warehouse.xml
    python3 build_env_scene.py --env scenes/warehouse.xml --inspire 1
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

GRASP_PREFIX_RE = re.compile(r"^grasp_", re.IGNORECASE)

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


def build_base(robot_file: str, model_name: str):
    """Baut die feste Basis (G1 + Licht + Boden + Weld + Technik).

    Gibt (mujoco_root, asset_element, worldbody_element) zurueck, damit die
    Objekte der Umgebung anschliessend in asset/worldbody eingemischt werden.
    """
    mj = ET.Element("mujoco", {"model": model_name})
    ET.SubElement(mj, "include", {"file": robot_file})
    ET.SubElement(mj, "statistic", {"center": "0 0 0.5", "extent": "2.0"})

    vis = ET.SubElement(mj, "visual")
    ET.SubElement(vis, "headlight",
                  {"diffuse": "0.6 0.6 0.6", "ambient": "0.3 0.3 0.3", "specular": "0 0 0"})
    ET.SubElement(vis, "rgba", {"haze": "0.15 0.25 0.35 1"})
    ET.SubElement(vis, "global", {"azimuth": "-130", "elevation": "-20"})

    asset = ET.SubElement(mj, "asset")
    ET.SubElement(asset, "texture",
                  {"type": "skybox", "builtin": "gradient",
                   "rgb1": "0.3 0.5 0.7", "rgb2": "0 0 0", "width": "512", "height": "3072"})
    ET.SubElement(asset, "texture",
                  {"type": "2d", "name": "groundplane", "builtin": "checker", "mark": "edge",
                   "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3", "markrgb": "0.8 0.8 0.8",
                   "width": "300", "height": "300"})
    ET.SubElement(asset, "material",
                  {"name": "groundplane", "texture": "groundplane", "texuniform": "true",
                   "texrepeat": "5 5", "reflectance": "0.2"})

    wb = ET.SubElement(mj, "worldbody")
    # Lichtquelle (Basis)
    ET.SubElement(wb, "light", {"pos": "0 0 1.5", "dir": "0 0 -1", "directional": "true"})
    # Boden (Basis)
    ET.SubElement(wb, "geom",
                  {"name": "floor", "size": "0 0 0.05", "type": "plane", "material": "groundplane"})

    # Weld, der den G1 am Anfang festhaelt (per Name vom Sim gesteuert).
    eq = ET.SubElement(mj, "equality")
    ET.SubElement(eq, "weld",
                  {"name": "hold_base_weld", "body1": "torso_link",
                   "solref": "0.01 1", "solimp": "0.99 0.999 0.001 0.5 2"})

    return mj, asset, wb


def _wrap_grasp_geom(geom_el):
    """Wandelt ein statisches <geom name="grasp_..."/> in einen freien Koerper
    um (<body><freejoint/><geom/></body>), damit MuJoCo es beim Greifen/
    Anfassen bewegen kann. Body UND Geom tragen denselben Namen (in MuJoCo
    erlaubt -- Bodies und Geoms haben getrennte Namensraeume); das ist wichtig,
    weil die Sim-Seite (scene_objects.py) die LIVE-Pose ueber den BODY-Namen
    aufloest. Body erbt Pose (pos/quat) vom Geom, das Geom selbst wird auf
    Identity relativ zum Body gesetzt (pos/quat entfernt).
    """
    name = geom_el.get("name")
    pos = geom_el.get("pos", "0 0 0")
    quat = geom_el.get("quat")

    body = ET.Element("body", {"name": name, "pos": pos})
    if quat:
        body.set("quat", quat)
    ET.SubElement(body, "freejoint")

    inner = ET.Element("geom", dict(geom_el.attrib))
    inner.attrib.pop("pos", None)
    inner.attrib.pop("quat", None)
    body.append(inner)
    return body


def _is_grasp_geom(el) -> bool:
    return el.tag == "geom" and bool(GRASP_PREFIX_RE.match(el.get("name") or ""))


def merge_environment(env_root, asset, wb, env_dir, warnings):
    """Mischt NUR die Objekte der Umgebung in die Basis ein.

    - <asset>: eigene Meshes/Texturen/Materialien der Objekte (Mesh-Pfade
      werden umgeschrieben). Doppelte Namen und ein zweiter Skybox werden
      uebersprungen.
    - <worldbody>: alle Objekte (geoms/bodies). Ein evtl. mitgespeicherter
      Boden (<geom type="plane">) und Lichtquellen werden weggelassen - die
      kommen aus der Basis.
    """
    used_asset_names = {el.get("name") for el in asset if el.get("name")}

    for env_asset in env_root.findall("asset"):
        for el in list(env_asset):
            if el.tag == "texture" and el.get("type") == "skybox":
                continue  # nur ein Skybox erlaubt (Basis hat schon einen)
            nm = el.get("name")
            if nm and nm in used_asset_names:
                warnings.append(f"  ! Asset-Name '{nm}' schon in der Basis -> uebersprungen")
                continue
            if el.tag == "mesh" and el.get("file"):
                p = Path(el.get("file"))
                mesh_abs = p if p.is_absolute() else (env_dir / p).resolve()
                if not mesh_abs.is_file():
                    warnings.append(f"  ! Mesh nicht gefunden: {el.get('file')}  ({mesh_abs})")
                else:
                    try:
                        mesh_abs.relative_to(MJ_ROOT)
                    except ValueError:
                        warnings.append(
                            "  ! Mesh liegt ausserhalb von unitree_mujoco/ (im Docker-"
                            f"Container evtl. nicht sichtbar): {mesh_abs}")
                el.set("file", rel_to_meshdir(mesh_abs))
            asset.append(el)
            if nm:
                used_asset_names.add(nm)

    for env_wb in env_root.findall("worldbody"):
        for el in list(env_wb):
            if el.tag == "light":
                continue  # Licht kommt aus der Basis
            if el.tag == "geom" and el.get("type") == "plane":
                continue  # Boden kommt aus der Basis
            if _is_grasp_geom(el):
                wb.append(_wrap_grasp_geom(el))
                continue
            wb.append(el)

    # Auf Abschnitte hinweisen, die eine reine Objekt-Umgebung normalerweise
    # nicht enthalten sollte (werden bewusst NICHT uebernommen).
    for tag in ("equality", "actuator", "default", "contact", "tendon", "sensor"):
        if env_root.find(tag) is not None:
            warnings.append(f"  ! <{tag}> in der Umgebung wird ignoriert "
                            "(Umgebungen sollen nur Objekte enthalten).")


def main() -> None:
    ap = argparse.ArgumentParser(description="G1-Basis + Umgebung zu lauffaehiger Szene kombinieren")
    ap.add_argument("--env", required=True,
                    help="Umgebungs-XML (nur Objekte), z.B. scenes/warehouse.xml")
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

    # Umgebung einlesen (Kommentare vorher strippen: MuJoCos Parser ist tolerant,
    # ElementTree strikt - z.B. sind '--'-Folgen in XML-Kommentaren ungueltig).
    raw = env_path.read_text(encoding="utf-8")
    cleaned = re.sub(r"<!--.*?-->", "", raw, flags=re.DOTALL)
    env_root = ET.fromstring(cleaned)

    warnings = []
    mj, asset, wb = build_base(robot_file, f"g1_env_{name}")
    merge_environment(env_root, asset, wb, env_dir, warnings)

    ET.indent(mj, space="  ")
    header = (
        "<!-- AUTO-GENERIERT von scene_editor/build_env_scene.py.\n"
        f"     Basis (G1 + Licht + Boden + Weld) + Umgebung: {env_path.name}\n"
        f"     Roboter: {robot_file}\n"
        "     NICHT von Hand editieren - wird bei jeder Umgebungs-Auswahl neu erzeugt. -->\n"
    )
    out_path.write_text(header + ET.tostring(mj, encoding="unicode") + "\n", encoding="utf-8")

    for w in warnings:
        print(w, file=sys.stderr)
    print(out_path)


if __name__ == "__main__":
    main()
