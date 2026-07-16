# Meshes

Hier liegen eigene 3D-Objekte (STL/OBJ) fuer die Szenen.

- `sample_crate.stl` – 30 cm Wuerfel (Beispiel-Import)
- `sample_ramp.stl`  – kleine Rampe/Keil (Beispiel-Import)

## Eigene Meshes reinlegen
Datei einfach hier ablegen und in der Szene einbinden:

```xml
<asset>
  <mesh name="mein_objekt" file="../meshes/mein_objekt.stl"
        scale="0.001 0.001 0.001"/>   <!-- CAD-STL in mm -> Meter -->
</asset>
<worldbody>
  <geom type="mesh" mesh="mein_objekt" pos="1 0 0"/>
</worldbody>
```

Tipps:
- MuJoCo mag **binaeres** STL am liebsten. ASCII-STL geht meist auch,
  im Zweifel in einem CAD/Slicer als binaer exportieren.
- **scale** nicht vergessen: STL aus CAD ist oft in Millimetern, MuJoCo
  rechnet in Metern -> `scale="0.001 0.001 0.001"`.
- Fuer **Kollision** sollte das Mesh geschlossen (watertight) und moeglichst
  konvex sein. Komplexe Meshes kollidieren sonst nur mit ihrer konvexen
  Huelle. Reine Deko: `contype="0" conaffinity="0"` am geom setzen.
