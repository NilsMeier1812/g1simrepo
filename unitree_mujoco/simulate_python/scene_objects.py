# -*- coding: utf-8 -*-
"""
scene_objects.py — liest die Umgebungs-Objekte direkt aus der geladenen
MuJoCo-Szene (scene_env_<name>.xml bzw. scene.xml), damit sie an ROS/RViz
weitergereicht werden koennen (siehe scene_state_publisher.py).

Warum aus der XML und nicht aus mj_model? Weil wir hier auch die STATISCHEN
Eigenschaften brauchen, die MuJoCo beim Kompilieren nicht mehr 1:1 vorhaelt
(Original-Geom-Typ/-Groesse ist zwar auch in mj_model, aber Mesh-Dateiname und
das <asset>-Mapping sind in der XML direkter zugaenglich). Die eigentlichen
LIVE-Posen greifbarer Objekte kommen weiterhin aus mj_data (siehe
scene_state_publisher.SceneStatePublisher).

Warum das ueberhaupt ein eigenes Modul ist: build_env_scene.py (im
scene_editor) baut die kombinierte Szene so, dass NUR die Objekte der
Umgebung direkte Kinder von <worldbody> sind -- der G1 kommt komplett ueber
<include file="..."/> (liegt NICHT inline in dieser Datei) und Boden/Licht
sind die einzigen anderen direkten Kinder. Direkte <worldbody>-Kinder (ausser
"floor" und <light>) sind also GENAU die Umgebungs-Objekte -- ohne dass wir
irgendetwas von der Roboter-Kinematik wissen muessen.

Klassifikation Hindernis vs. Greif-Objekt: per Namenskonvention. Ein Objekt-
Name, der mit "grasp_" beginnt (Gross-/Kleinschreibung egal), ist ein
GREIFBARES Objekt (wird von build_env_scene.py als freier Koerper -- also mit
<freejoint/> -- gebaut, siehe dort). Alles andere ist ein Hindernis. Rein
dekorative Objekte (contype=0 UND conaffinity=0) werden komplett
ausgeklammert (weder Hindernis noch greifbar) -- dieselbe Konvention wie im
Scene-Editor-README.
"""
import os
import re
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

GRASP_PREFIX_RE = re.compile(r"^grasp_", re.IGNORECASE)


def classify_name(name: str) -> str:
    return "grasp" if GRASP_PREFIX_RE.match(name or "") else "obstacle"


def _floats(s, default):
    if not s:
        return list(default)
    return [float(x) for x in s.split()]


def _quat_mul(a, b):
    """Hamilton-Produkt zweier Quaternionen (w,x,y,z), pure Python (keine
    numpy-Abhaengigkeit -- dieses Modul soll ohne mujoco/numpy testbar sein)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_rotate(q, v):
    """Rotiert Vektor v=(x,y,z) mit Quaternion q=(w,x,y,z)."""
    w, x, y, z = q
    vx, vy, vz = v
    # v' = q * v * q^-1, ausmultipliziert (q ist normiert angenommen).
    qv = (0.0, vx, vy, vz)
    qc = (w, -x, -y, -z)
    _, rx, ry, rz = _quat_mul(_quat_mul(q, qv), qc)
    return (rx, ry, rz)


def _compose_pose(base_pos, base_quat, local_pos, local_quat):
    """Weltpose = base * local (base=umschliessende Body, local=Geom-eigene
    pos/quat relativ dazu). Fuer direkte worldbody-Kinder ist base=Identity ->
    das Ergebnis ist dann einfach local (= die eigenen Geom-Attribute)."""
    rotated = _quat_rotate(base_quat, local_pos)
    world_pos = (base_pos[0] + rotated[0], base_pos[1] + rotated[1], base_pos[2] + rotated[2])
    world_quat = _quat_mul(base_quat, local_quat)
    return list(world_pos), list(world_quat)


def _is_decorative(attrs: dict) -> bool:
    ct = attrs.get("contype")
    ca = attrs.get("conaffinity")
    # MuJoCo-Default fuer beide ist 1 (kollidiert) -- dekorativ nur, wenn BEIDE
    # explizit auf 0 stehen (Konvention aus scene_editor/README.md).
    try:
        return ct is not None and ca is not None and float(ct) == 0.0 and float(ca) == 0.0
    except ValueError:
        return False


def _mesh_local_aabb_half(stl_path: Path):
    """Halbe Bounding-Box (lokal, VOR Skalierung) eines STL-Meshes.

    Reine Bounding-Box, kein Convex-Hull -- reicht fuer die 2D-Fussabdruck-
    Rasterung (Nav) und als konservative Kollisions-Naeherung (IK). Unterstuetzt
    binaeres STL (Standardfall der Editor-Exporte); bei ASCII-STL wird ein
    einfacher Text-Parser als Fallback versucht.
    """
    try:
        data = stl_path.read_bytes()
    except OSError:
        return None
    if len(data) >= 84:
        ntri = struct.unpack_from("<I", data, 80)[0]
        expected = 84 + ntri * 50
        if ntri > 0 and expected == len(data):
            xs = [1e30, -1e30]
            ys = [1e30, -1e30]
            zs = [1e30, -1e30]
            off = 84
            for _ in range(ntri):
                # 12 floats: normal(3) + v1(3) + v2(3) + v3(3), dann 2 Byte attr.
                vals = struct.unpack_from("<12f", data, off)
                for k in range(3):
                    vx, vy, vz = vals[3 + 3 * k], vals[4 + 3 * k], vals[5 + 3 * k]
                    xs[0] = min(xs[0], vx); xs[1] = max(xs[1], vx)
                    ys[0] = min(ys[0], vy); ys[1] = max(ys[1], vy)
                    zs[0] = min(zs[0], vz); zs[1] = max(zs[1], vz)
                off += 50
            return ((xs[1] - xs[0]) / 2.0, (ys[1] - ys[0]) / 2.0, (zs[1] - zs[0]) / 2.0)
    # ASCII-STL-Fallback: Zeilen "vertex x y z" auswerten.
    try:
        text = data.decode("ascii", errors="ignore")
    except Exception:
        return None
    xs = [1e30, -1e30]; ys = [1e30, -1e30]; zs = [1e30, -1e30]
    found = False
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            parts = line.split()
            if len(parts) >= 4:
                vx, vy, vz = float(parts[1]), float(parts[2]), float(parts[3])
                xs[0] = min(xs[0], vx); xs[1] = max(xs[1], vx)
                ys[0] = min(ys[0], vy); ys[1] = max(ys[1], vy)
                zs[0] = min(zs[0], vz); zs[1] = max(zs[1], vz)
                found = True
    if not found:
        return None
    return ((xs[1] - xs[0]) / 2.0, (ys[1] - ys[0]) / 2.0, (zs[1] - zs[0]) / 2.0)


def _resolve_mesh_file(file_: str, scene_dir: Path, meshdir: str):
    """Mesh-Datei auf der Platte finden. MuJoCos <compiler meshdir="..."> gilt
    GLOBAL (auch fuer geerbte/inkludierte Modelle) und macht `file`-Attribute
    relativ zu scene_dir/meshdir -- die kombinierte Szene (scene_env_<name>.xml)
    erbt meshdir="meshes" vom G1-Modell, obwohl das <compiler>-Tag im
    inkludierten g1_29dof.xml steht (ElementTree loest Includes NICHT auf, wir
    sehen es hier also nicht direkt). Deshalb mehrere Basis-Kandidaten
    durchprobieren und den ersten existierenden Pfad nehmen:
      1. scene_dir/meshdir/file  (kombinierte Szene: meshdir='meshes')
      2. scene_dir/file          (Quell-Szene aus scene_editor/scenes/: '../meshes/..')
      3. absoluter Pfad
    Existiert keiner, den meshdir-Kandidaten zurueckgeben (basename bleibt
    korrekt; die AABB entfaellt dann nur -> konservative Default-Box)."""
    p = Path(file_)
    candidates = []
    if p.is_absolute():
        candidates.append(p)
    else:
        candidates.append((scene_dir / meshdir / file_))
        candidates.append((scene_dir / file_))
    for c in candidates:
        rc = c.resolve()
        if rc.is_file():
            return rc
    return candidates[0].resolve()


def _collect_mesh_assets(root, scene_dir: Path):
    """<asset><mesh name=".." file=".." scale=".."/></asset> -> dict name -> info."""
    # meshdir aus <compiler> lesen, falls im Datei selbst vorhanden; sonst der
    # G1-Default 'meshes' (die kombinierte Szene erbt ihn, siehe
    # _resolve_mesh_file).
    meshdir = "meshes"
    comp = root.find("compiler")
    if comp is not None and comp.get("meshdir"):
        meshdir = comp.get("meshdir")

    meshes = {}
    for asset_el in root.findall("asset"):
        for m in asset_el.findall("mesh"):
            name = m.get("name")
            file_ = m.get("file")
            if not name or not file_:
                continue
            scale = _floats(m.get("scale"), (1.0, 1.0, 1.0))
            if len(scale) == 1:
                scale = [scale[0]] * 3
            mesh_path = _resolve_mesh_file(file_, scene_dir, meshdir)
            meshes[name] = {
                "file": str(mesh_path),
                "basename": os.path.basename(file_),
                "scale": tuple(scale[:3]),
            }
    return meshes


def _geom_spec(geom_el, meshes: dict, base_pos, base_quat):
    """Baut die Objekt-Spec aus einem einzelnen <geom>-Element.

    base_pos/base_quat = Pose der umschliessenden <body> (0/Identity, falls
    das Geom direkt in <worldbody> liegt) -- fuer greifbare Objekte (in einen
    <body><freejoint/> gewrapt, siehe build_env_scene.py) traegt die BODY die
    Pose, das Geom selbst liegt bei "0 0 0"/Identity relativ dazu.
    """
    name = geom_el.get("name") or ""
    gtype = geom_el.get("type", "box")
    rgba = _floats(geom_el.get("rgba"), (0.7, 0.7, 0.7, 1.0))
    attrs = {"contype": geom_el.get("contype"), "conaffinity": geom_el.get("conaffinity")}
    if _is_decorative(attrs):
        return None

    local_pos = _floats(geom_el.get("pos"), (0.0, 0.0, 0.0))
    local_quat = _floats(geom_el.get("quat"), (1.0, 0.0, 0.0, 0.0))
    world_pos, world_quat = _compose_pose(base_pos, base_quat, local_pos, local_quat)

    spec = {
        "name": name,
        "class": classify_name(name),
        "type": gtype,
        "pos": world_pos,
        "quat": world_quat,
        "rgba": list(rgba[:4]),
        "size": None,
        "mesh_basename": None,
        "aabb_half": None,
    }

    if gtype == "mesh":
        mesh_name = geom_el.get("mesh")
        info = meshes.get(mesh_name)
        if info is None:
            return None
        spec["mesh_basename"] = info["basename"]
        spec["size"] = list(info["scale"])  # MJCF-Mesh-Skalierung (fuers Rendering)
        local_half = _mesh_local_aabb_half(Path(info["file"]))
        if local_half is not None:
            sx, sy, sz = info["scale"]
            spec["aabb_half"] = [abs(local_half[0] * sx), abs(local_half[1] * sy),
                                 abs(local_half[2] * sz)]
    else:
        spec["size"] = _floats(geom_el.get("size"), (0.1, 0.1, 0.1))

    return spec


def parse_scene_objects(scene_xml_path: str):
    """Liste der Umgebungs-Objekte (Hindernisse + Greif-Objekte) aus der
    kombinierten Szene. Jeder Eintrag ist ein dict wie in _geom_spec()
    beschrieben. Robuster Rueckgabewert [] bei jedem Parse-Problem (der Aufrufer
    laeuft dann einfach ohne Szenen-Objekte weiter -- kein harter Fehler)."""
    path = Path(scene_xml_path)
    if not path.is_file():
        return []
    try:
        raw = path.read_text(encoding="utf-8")
        cleaned = re.sub(r"<!--.*?-->", "", raw, flags=re.DOTALL)
        root = ET.fromstring(cleaned)
    except (OSError, ET.ParseError):
        return []

    scene_dir = path.parent
    meshes = _collect_mesh_assets(root, scene_dir)

    specs = []
    for wb in root.findall("worldbody"):
        for el in list(wb):
            if el.tag == "light":
                continue
            if el.tag == "geom":
                if el.get("name") == "floor" and el.get("type") == "plane":
                    continue
                spec = _geom_spec(el, meshes, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
                if spec is not None and spec["name"]:
                    specs.append(spec)
            elif el.tag == "body":
                body_pos = _floats(el.get("pos"), (0.0, 0.0, 0.0))
                body_quat = _floats(el.get("quat"), (1.0, 0.0, 0.0, 0.0))
                inner_geom = el.find("geom")
                if inner_geom is None:
                    continue
                spec = _geom_spec(inner_geom, meshes, body_pos, body_quat)
                if spec is not None:
                    # Bei gewrappten Greif-Objekten traegt die BODY den Namen
                    # (siehe build_env_scene.py) -- der ist massgeblich fuer die
                    # Live-Pose-Aufloesung (mj_data.xpos[body_id]) und die
                    # ROS-Identifikation.
                    spec["name"] = el.get("name") or spec["name"]
                    spec["class"] = classify_name(spec["name"])
                    spec["has_freejoint"] = el.find("freejoint") is not None
                    specs.append(spec)
    return specs
