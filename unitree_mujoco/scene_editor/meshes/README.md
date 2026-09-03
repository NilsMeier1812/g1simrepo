# Meshes

Hier liegen eigene 3D-Objekte (STL/OBJ) und CAD-Dateien (STEP/STP) fuer die
Szenen.

- `sample_crate.stl`     – 30 cm Wuerfel (Beispiel-Import)
- `sample_ramp.stl`      – kleine Rampe/Keil (Beispiel-Import)
- `sample_bracket.step`  – Winkel-Halterung in mm (Beispiel fuer den CAD-Import;
  wird beim Editor-Start automatisch zu `sample_bracket.stl`)

## Import im Editor
Zwei Wege:
- **„Eigene Datei hochladen" -> „STL/OBJ/STEP waehlen ..."** (oben im Editor):
  Datei-Dialog, beliebiger Ordner; die Datei wird hierher kopiert und direkt in
  die Szene eingefuegt.
- **„Add Assets from File" -> „Scan assets" -> „Add asset"**: durchsucht diesen
  `meshes/`-Ordner (STL hier ablegen).

## STEP/STP (CAD)
MuJoCo kann **kein STEP** lesen – es braucht ein Dreiecksnetz. STEP-Dateien
werden darum automatisch nach STL tesseliert:

- beim **Hochladen** im Editor,
- beim **Editor-Start** fuer alles, was hier im Ordner liegt,
- oder von Hand: `./launch.sh convert sample_bracket.step`.

Dabei wird per Default mit **0.001** skaliert (CAD in mm -> MuJoCo in m). Ist
die STEP schon in Metern: `--scale 1` bzw. im Editor-Ordner „STEP/CAD-Import"
umstellen. Details: `../step_import.py`.

## Eigene Meshes per XML einbinden
Alternativ direkt in einer Szene referenzieren:

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
