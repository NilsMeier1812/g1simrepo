"""Tests fuer g1pilot/utils/lowpoly.py (Sparsame Grafik, nur RViz-Anzeige)."""
import os

import numpy as np

from g1pilot.utils import lowpoly


def _sphere_tris(n=60, r=0.05):
    th = np.linspace(0, np.pi, n)
    ph = np.linspace(0, 2 * np.pi, 2 * n)
    p = np.array([[[r * np.sin(t) * np.cos(f), r * np.sin(t) * np.sin(f), r * np.cos(t)]
                   for f in ph] for t in th])
    tris = []
    for i in range(n - 1):
        for j in range(2 * n - 1):
            tris.append([p[i, j], p[i + 1, j], p[i + 1, j + 1]])
            tris.append([p[i, j], p[i + 1, j + 1], p[i, j + 1]])
    return np.array(tris)


def test_simplify_reduces_and_keeps_bounds():
    tris = _sphere_tris()
    verts, faces = lowpoly.simplify_triangles(tris, 0.01)
    assert 4 <= len(faces) < 0.5 * len(tris)
    assert faces.max() < len(verts)
    # Form bleibt erhalten: Ausdehnung weicht hoechstens um eine Zelle ab.
    assert np.allclose(verts.max(0) - verts.min(0), 0.1, atol=0.01)


def test_lowpoly_file_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setattr(lowpoly, "CACHE_DIR", str(tmp_path / "cache"))
    src = tmp_path / "ball.stl"
    v = _sphere_tris()
    lowpoly.write_binary_stl(str(src), v.reshape(-1, 3), np.arange(len(v) * 3).reshape(-1, 3))
    out = lowpoly.lowpoly_file(str(src), 0.01)
    assert out and os.path.isfile(out)
    assert 0 < len(lowpoly.read_stl(out)) < len(v)
    assert lowpoly.lowpoly_file(str(src), 0.01) == out      # Cache-Treffer


def test_urdf_only_visual_meshes_change(tmp_path, monkeypatch):
    monkeypatch.setattr(lowpoly, "CACHE_DIR", str(tmp_path / "cache"))
    v = _sphere_tris()
    (tmp_path / "meshes").mkdir()
    lowpoly.write_binary_stl(str(tmp_path / "meshes" / "ball.STL"), v.reshape(-1, 3),
                             np.arange(len(v) * 3).reshape(-1, 3))
    urdf = ('<robot><link name="a">'
            '<visual><geometry><mesh filename="package://pkg/meshes/ball.STL"/></geometry></visual>'
            '<collision><geometry><mesh filename="package://pkg/meshes/ball.STL"/></geometry></collision>'
            '</link></robot>')
    out, n = lowpoly.lowpoly_urdf(urdf, lambda pkg: str(tmp_path), cell_m=0.01)
    assert n == 1
    assert 'file://' in out.split("<collision>")[0]
    assert 'filename="package://pkg/meshes/ball.STL"' in out.split("<collision>")[1]
