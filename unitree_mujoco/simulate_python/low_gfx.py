# -*- coding: utf-8 -*-
"""
low_gfx.py — "Sparsame Grafik" fuer den MuJoCo-Viewer (opt-in, G1_LOW_GFX=1).

Reine ANZEIGE-Optimierung fuer schwache PCs. Physik, Kollision, Massen und
Regelung bleiben unveraendert:

  * Schatten und Bodenspiegelung aus. Jede davon kostet einen kompletten
    zusaetzlichen Render-Durchgang der Szene.
  * Kollisions-Meshes werden nicht mehr gezeichnet (Gruppe 3, im Viewer
    standardmaessig aus). Am G1 liegen sie deckungsgleich unter den Visual-
    Meshes -> der Roboter wurde bisher doppelt gezeichnet. Sie kollidieren
    weiter wie bisher, nur unsichtbar.
  * Was sichtbar bleibt, zeigt ein vereinfachtes Mesh (Vertex-Clustering, s.
    simplify_triangles). Dafuer bekommt jedes Kollisions-Mesh ohne eigenes
    Visual-Geom ein unsichtbares Visual-Gegenstueck: contype=conaffinity=0
    (kollidiert nie) und density=0 (keine Masse/Traegheit).

Die Kollisions-Geoms behalten ihre Original-Meshes; die Vereinfachung betrifft
ausschliesslich Geoms, die nie kollidieren. Abschalten: G1_LOW_GFX=0 (Default).

Die Clustering-Funktionen gibt es in identischer Form auch auf der ROS-Seite
(g1pilot/g1pilot/utils/lowpoly.py, fuer RViz) -- die beiden Container teilen
keinen Python-Code.
"""
import os
import struct

import mujoco
import numpy as np

#: Clustering-Raster [m] fuer den Roboter (feine Teile) und die Umgebung.
ROBOT_CELL_M = float(os.environ.get("G1_LOW_GFX_ROBOT_CELL", "0.003"))
ENV_CELL_M = float(os.environ.get("G1_LOW_GFX_ENV_CELL", "0.005"))

#: Lohnt sich die Vereinfachung nicht (Rest > dieser Anteil), Original behalten.
_MIN_GAIN = 0.9
_SUFFIX = "__lowgfx"


# ── Mesh-Vereinfachung (identisch zu g1pilot/utils/lowpoly.py) ──────────────
def read_stl(path):
    """STL (binaer oder ASCII) -> Dreiecke als (N, 3, 3) float64. None bei Fehler."""
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError:
        return None
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(
                data, count=n, offset=84,
                dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
            return rec["v"].astype(np.float64)
    vals = [ln.split()[1:4] for ln in data.decode("ascii", "ignore").splitlines()
            if ln.strip().startswith("vertex")]
    if not vals or len(vals) % 3:
        return None
    try:
        return np.array(vals, dtype=np.float64).reshape(-1, 3, 3)
    except ValueError:
        return None


def simplify_triangles(tris, cell):
    """Vertex-Clustering: alle Ecken in einer Rasterzelle der Kantenlaenge
    `cell` zu ihrem Mittelpunkt verschmelzen, entartete und doppelte Dreiecke
    verwerfen. -> (verts (V,3), faces (F,3) int). Schnell (rein numpy) und fuer
    die Anzeige aus Betrachterabstand ausreichend genau."""
    v = tris.reshape(-1, 3)
    k = np.floor(v / cell).astype(np.int64)
    k -= k.min(axis=0)
    dims = k.max(axis=0) + 1
    key = (k[:, 0] * dims[1] + k[:, 1]) * dims[2] + k[:, 2]
    _, inv = np.unique(key, return_inverse=True)
    inv = inv.reshape(-1)
    cnt = np.bincount(inv)
    verts = np.stack([np.bincount(inv, weights=v[:, i]) / cnt for i in range(3)], axis=1)
    faces = inv.reshape(-1, 3)
    ok = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
          & (faces[:, 0] != faces[:, 2]))
    faces = faces[ok]
    if len(faces):
        _, idx = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
        faces = faces[np.sort(idx)]
    used = np.unique(faces)
    remap = np.full(len(verts), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    return verts[used], remap[faces]


# ── Anwendung auf die Szene ─────────────────────────────────────────────────
def _is_visual(g):
    return g.contype == 0 and g.conaffinity == 0


def _mesh_path(scene_dir, meshdir, file_):
    if os.path.isabs(file_):
        return file_
    return os.path.normpath(os.path.join(scene_dir, meshdir or "", file_))


def _robot_geom(g, world):
    return g.parent.name != world.name


def _twin_key(g, meshname=None):
    """Body + Mesh + Pose: ein Visual-Geom ist nur dann das Gegenstueck eines
    Kollisions-Geoms, wenn es dasselbe Mesh an derselben Stelle zeigt (in CAD-
    Umgebungen kommt dasselbe Mesh oft mehrfach an anderen Orten vor)."""
    return (g.parent.name, meshname or g.meshname,
            tuple(np.round(np.asarray(g.pos, dtype=float), 6)),
            tuple(np.round(np.asarray(g.quat, dtype=float), 6)),
            tuple(np.round(np.asarray(g.alt.euler, dtype=float), 6)))


def apply_to_spec(spec, scene_path):
    """Visual-Geoms auf vereinfachte Meshes umstellen, Kollisions-Geoms
    ausblenden (Gruppe 3). Vor spec.compile() aufrufen."""
    scene_dir = os.path.dirname(os.path.abspath(scene_path))
    meshes = {m.name: m for m in spec.meshes}
    world = spec.worldbody
    geoms = world.find_all(mujoco.mjtObj.mjOBJ_GEOM)

    lowpoly = {}            # Original-Mesh-Name -> Name des vereinfachten Meshes
    stats = {"tris_in": 0, "tris_out": 0}

    def lowpoly_mesh(name, cell_m):
        if name in lowpoly:
            return lowpoly[name]
        lowpoly[name] = name     # Fallback: Original
        src = meshes.get(name)
        if src is None or not src.file or not src.file.lower().endswith(".stl"):
            return name
        tris = read_stl(_mesh_path(scene_dir, spec.meshdir, src.file))
        if tris is None or len(tris) == 0:
            return name
        scale = float(np.max(np.abs(src.scale))) or 1.0
        verts, faces = simplify_triangles(tris, cell_m / scale)
        if len(faces) < 4 or len(faces) > _MIN_GAIN * len(tris):
            return name
        m = spec.add_mesh()
        m.name = name + _SUFFIX
        m.uservert = verts.astype(np.float32).reshape(-1).tolist()
        m.userface = faces.astype(np.int32).reshape(-1).tolist()
        m.scale = src.scale
        m.refpos = src.refpos
        m.refquat = src.refquat
        # Nur Anzeige: Schalen-Traegheit rechnet auch fuer offene/flache
        # Vereinfachungen (Volumen-Traegheit koennte daran scheitern). Die Geoms
        # haben ohnehin density=0.
        m.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
        stats["tris_in"] += len(tris)
        stats["tris_out"] += len(faces)
        lowpoly[name] = m.name
        return m.name

    # Sichtbare (Gruppe < 3) Visual-Meshes je Body -> hat ein Kollisions-Geom
    # schon ein deckungsgleiches Visual-Gegenstueck?
    visual_twins = set()
    for g in geoms:
        if g.type == mujoco.mjtGeom.mjGEOM_MESH and _is_visual(g) and g.group < 3:
            visual_twins.add(_twin_key(g))

    n_hidden = n_added = n_swapped = 0
    for g in geoms:
        if g.type != mujoco.mjtGeom.mjGEOM_MESH or g.group >= 3:
            continue
        cell = ROBOT_CELL_M if _robot_geom(g, world) else ENV_CELL_M
        if _is_visual(g):
            new = lowpoly_mesh(g.meshname, cell)
            if new != g.meshname:
                g.meshname = new
                n_swapped += 1
            continue
        # Kollisions-Geom: ausblenden, Kollision bleibt unveraendert.
        if _twin_key(g) not in visual_twins:
            twin = g.parent.add_geom()
            twin.type = mujoco.mjtGeom.mjGEOM_MESH
            twin.meshname = lowpoly_mesh(g.meshname, cell)
            twin.pos = g.pos
            twin.quat = g.quat
            twin.alt = g.alt
            twin.rgba = g.rgba
            twin.material = g.material
            twin.group = g.group
            twin.contype = 0
            twin.conaffinity = 0
            twin.density = 0.0
            n_added += 1
        g.group = 3
        n_hidden += 1

    print(f"[LOW_GFX] Sparsame Grafik: {n_hidden} Kollisions-Geom(s) ausgeblendet, "
          f"{n_added} Visual-Gegenstueck(e) ergaenzt, {n_swapped} Visual-Geom(s) "
          f"umgestellt; vereinfachte Meshes {stats['tris_in']:,} -> "
          f"{stats['tris_out']:,} Dreiecke (je Mesh-Datei).", flush=True)


def apply_to_model(mj_model):
    """Schatten + Spiegelung aus (reine Render-Felder, nach compile())."""
    mj_model.light_castshadow[:] = 0
    mj_model.mat_reflectance[:] = 0.0
    print("[LOW_GFX] Schatten und Bodenspiegelung aus.", flush=True)
