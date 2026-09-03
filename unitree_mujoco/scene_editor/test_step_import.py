#!/usr/bin/env python3
"""Tests fuer step_import.py - Schwerpunkt STL-Pruefung/ASCII-Reparatur.

Die eigentliche STEP-Konvertierung braucht ein CAD-Backend (cadquery-ocp) und
wird hier uebersprungen, wenn keins installiert ist. Die Teile, an denen der
Editor bisher gescheitert ist (ASCII-STL, zu viele Dreiecke), laufen dagegen
ohne jede Abhaengigkeit.

    python3 test_step_import.py
"""
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import step_import as si  # noqa: E402


def binary_stl(n_faces: int) -> bytes:
    out = bytearray(b"\0" * 80) + struct.pack("<I", n_faces)
    for i in range(n_faces):
        out += struct.pack("<3f", 0.0, 0.0, 1.0)
        out += struct.pack("<3f", 0.0, 0.0, float(i))
        out += struct.pack("<3f", 1.0, 0.0, float(i))
        out += struct.pack("<3f", 0.0, 1.0, float(i))
        out += struct.pack("<H", 0)
    return bytes(out)


ASCII_STL = """solid teil
  facet normal 0.0 0.0 1.0
    outer loop
      vertex 0.0 0.0 0.0
      vertex 1.5 0.0 0.0
      vertex 0.0 2.5 0.0
    endloop
  endfacet
  facet normal 0.0 0.0 -1.0
    outer loop
      vertex 0.0 0.0 1.0
      vertex 1.0 0.0 1.0
      vertex 0.0 1.0 1.0
    endloop
  endfacet
endsolid teil
"""


class TestSTLPruefung(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp(prefix="stepimport_"))

    def _write(self, data, name="teil.stl") -> Path:
        p = self.dir / name
        p.write_bytes(data if isinstance(data, bytes) else data.encode())
        return p

    def test_binaere_stl_wird_erkannt(self):
        p = self._write(binary_stl(7))
        self.assertEqual(si.stl_face_count(p), 7)
        self.assertTrue(si.is_binary_stl(p))
        self.assertIsNone(si.stl_problem(p))

    def test_ascii_stl_ist_keine_binaere(self):
        p = self._write(ASCII_STL)
        self.assertIsNone(si.stl_face_count(p))
        self.assertIn("ASCII", si.stl_problem(p))

    def test_groesse_passt_nicht_zum_header(self):
        p = self._write(b"\0" * 80 + struct.pack("<I", 5) + b"\0" * 10)
        self.assertIsNone(si.stl_face_count(p))

    def test_ascii_wird_binaer_und_behaelt_die_geometrie(self):
        p = self._write(ASCII_STL)
        self.assertEqual(si.ascii_stl_to_binary(p), 2)
        self.assertEqual(si.stl_face_count(p), 2)
        data = p.read_bytes()
        # Erstes Dreieck: Normale + 3 Vertices wie in der ASCII-Quelle.
        vals = struct.unpack_from("<12f", data, 84)
        self.assertEqual(vals[:3], (0.0, 0.0, 1.0))
        self.assertEqual(vals[3:6], (0.0, 0.0, 0.0))
        self.assertEqual(vals[6:9], (1.5, 0.0, 0.0))
        self.assertEqual(vals[9:12], (0.0, 2.5, 0.0))

    def test_make_mujoco_ready_repariert_ascii(self):
        p = self._write(ASCII_STL)
        notes = []
        self.assertIsNone(si.make_mujoco_ready(p, notes))
        self.assertTrue(si.is_binary_stl(p))
        self.assertTrue(any("ASCII" in n for n in notes))

    def test_make_mujoco_ready_meldet_zu_grosses_netz(self):
        n = si.MJ_MAX_FACES + 1
        p = self._write(b"\0" * 80 + struct.pack("<I", n) + b"\0" * (n * 50),
                        name="riesig.stl")
        problem = si.make_mujoco_ready(p)
        self.assertIsNotNone(problem)
        self.assertIn("Dreiecke", problem)

    def test_leere_ascii_stl_meldet_klartext(self):
        p = self._write("solid leer\nendsolid leer\n")
        problem = si.make_mujoco_ready(p)
        self.assertIsNotNone(problem)
        self.assertIn("STL", problem)

    def test_obj_wird_durchgereicht(self):
        p = self._write(b"irgendwas", name="teil.obj")
        self.assertIsNone(si.make_mujoco_ready(p))

    def test_format_das_mujoco_nicht_kann_wird_abgelehnt(self):
        p = self._write(b"glTF-Dummy", name="teil.glb")
        problem = si.make_mujoco_ready(p)
        self.assertIsNotNone(problem)
        self.assertIn("Format", problem)

    def test_trimesh_liest_die_reparierte_datei(self):
        try:
            import trimesh
        except ImportError:
            self.skipTest("trimesh nicht installiert")
        p = self._write(ASCII_STL)
        si.ascii_stl_to_binary(p)
        mesh = trimesh.load_mesh(str(p), file_type="stl")
        self.assertEqual(len(mesh.faces), 2)


class TestKonvertierung(unittest.TestCase):
    """Nur mit installiertem CAD-Backend - sonst uebersprungen."""

    def setUp(self):
        if not si.available_backends():
            self.skipTest("kein STEP-Backend installiert")

    def test_beispiel_step_wird_ladbares_stl(self):
        src = si.MESHES_DIR / "sample_bracket.step"
        if not src.is_file():
            self.skipTest("sample_bracket.step fehlt")
        with tempfile.TemporaryDirectory() as tmp:
            out = si.convert_step_to_stl(src, Path(tmp) / "bracket.stl")
            self.assertIsNone(si.stl_problem(out))


if __name__ == "__main__":
    unittest.main(verbosity=2)
