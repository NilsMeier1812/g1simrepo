#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lowpoly.py — vereinfachte Mesh-Kopien fuer RViz ("Sparsame Grafik", G1_LOW_GFX=1).

Reine Anzeige: RViz bekommt vereinfachte STL-Kopien (in /tmp, der Container
mountet die Originale read-only), alle anderen Nutzer der Meshes (IK, Nav-Karte,
Kollisionspruefung) bleiben bei den Originalen bzw. arbeiten ohnehin nur mit
Bounding-Boxen (siehe scene_markers.py).

Das Clustering ist identisch zu unitree_mujoco/simulate_python/low_gfx.py (die
beiden Container teilen keinen Python-Code).
"""
import hashlib
import os
import re
import struct

import numpy as np

#: Clustering-Raster [m] fuer den Roboter (feine Teile) und die Umgebung.
ROBOT_CELL_M = float(os.environ.get("G1_LOW_GFX_ROBOT_CELL", "0.003"))
ENV_CELL_M = float(os.environ.get("G1_LOW_GFX_ENV_CELL", "0.005"))

CACHE_DIR = os.environ.get("G1_LOW_GFX_CACHE", "/tmp/g1_lowpoly")

#: Lohnt sich die Vereinfachung nicht (Rest > dieser Anteil), Original behalten.
_MIN_GAIN = 0.9


def enabled() -> bool:
    return os.environ.get("G1_LOW_GFX", "0").strip().lower() in ("1", "true", "yes", "on")


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
    verwerfen. -> (verts (V,3), faces (F,3) int)."""
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


def write_binary_stl(path, verts, faces):
    tri = verts[faces].astype(np.float32)                      # (F, 3, 3)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, norm, out=np.zeros_like(n), where=norm > 0)
    rec = np.zeros(len(tri), dtype=np.dtype(
        [("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"] = n
    rec["v"] = tri
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(b"g1pilot lowpoly".ljust(80, b" "))
        f.write(struct.pack("<I", len(rec)))
        f.write(rec.tobytes())
    os.replace(tmp, path)


def lowpoly_file(src, cell_m, scale=1.0, subdir="mesh"):
    """Vereinfachte Kopie von `src` im Cache (wird bei Bedarf erzeugt).
    -> Pfad der Kopie, oder None (Original behalten: kein STL, unlesbar, oder
    die Vereinfachung lohnt nicht)."""
    if not str(src).lower().endswith(".stl"):
        return None
    try:
        st = os.stat(src)
    except OSError:
        return None
    tag = hashlib.sha1(f"{os.path.abspath(src)}|{st.st_mtime_ns}|{st.st_size}|"
                       f"{cell_m}|{scale}".encode()).hexdigest()[:12]
    out_dir = os.path.join(CACHE_DIR, subdir)
    out = os.path.join(out_dir, f"{os.path.splitext(os.path.basename(src))[0]}_{tag}.stl")
    if os.path.isfile(out):
        return out
    tris = read_stl(src)
    if tris is None or len(tris) == 0:
        return None
    scale = abs(float(scale)) or 1.0
    verts, faces = simplify_triangles(tris, cell_m / scale)
    if len(faces) < 4 or len(faces) > _MIN_GAIN * len(tris):
        return None
    os.makedirs(out_dir, exist_ok=True)
    write_binary_stl(out, verts, faces)
    return out


_VISUAL_RE = re.compile(r"<visual\b.*?</visual>", re.DOTALL)
_FILENAME_RE = re.compile(r'filename="package://([^/"]+)/([^"]+)"')


def lowpoly_urdf(urdf_xml, resolve_package, cell_m=None):
    """Mesh-Verweise NUR in <visual>-Bloecken auf vereinfachte Kopien umbiegen
    (file://...). <collision> bleibt unangetastet. `resolve_package(name)` ->
    share-Verzeichnis des Pakets. -> (neues XML, Anzahl umgestellter Meshes)."""
    cell_m = ROBOT_CELL_M if cell_m is None else cell_m
    count = [0]

    def fix_filename(m):
        try:
            src = os.path.join(resolve_package(m.group(1)), m.group(2))
        except Exception:
            return m.group(0)
        out = lowpoly_file(src, cell_m, subdir="robot")
        if out is None:
            return m.group(0)
        count[0] += 1
        return f'filename="file://{out}"'

    def fix_visual(block):
        return _FILENAME_RE.sub(fix_filename, block.group(0))

    return _VISUAL_RE.sub(fix_visual, urdf_xml), count[0]
