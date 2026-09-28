# Meshes

Hier liegen eigene 3D-Objekte (STL/OBJ) und CAD-Dateien (STEP/IGES) fuer die
Szenen.

- `sample_crate.stl`     – 30 cm Wuerfel (Beispiel-Import)
- `sample_ramp.stl`      – kleine Rampe/Keil (Beispiel-Import)
- `sample_bracket.step`  – Winkel-Halterung in mm (Beispiel fuer den CAD-Import)
- `cad/<name>/`          – Einzelteile je CAD-Import (automatisch erzeugt, nicht
  im Git; neu erzeugen mit `./launch.sh import <datei>`)
- `uploads/`             – im Editor hochgeladene CAD-Dateien (nicht im Git)

Nur dieser Ordner ist in den Docker-Containern gemountet (Sim: ueber
`/unitree_mujoco`, ROS/RViz: als `/scene_meshes`) – Meshes, die eine Umgebung
benutzt, muessen also hier (oder in Unterordnern) liegen. Der Editor kopiert
Meshes von ausserhalb beim Speichern automatisch nach `imported/`.

## Import im Editor
- **„Eigene Datei hochladen" -> „Datei waehlen (STL/OBJ/STEP/...)"** (oben im
  Editor): Datei-Dialog, beliebiger Ordner.
- **„Add Assets from File" -> „Scan assets" -> „Add asset"**: durchsucht diesen
  `meshes/`-Ordner (Einzelteile aus `cad/` werden dort ausgeblendet).
- **„CAD-Import" -> „Importieren & einfuegen"**: STEP/IGES/... aus diesem Ordner.

## CAD (STEP/IGES) und fremde Mesh-Formate
MuJoCo kann nur STL/OBJ/MSH. Alles andere zerlegt `../cad_import.py` in
Einzelteil-Meshes (mit Namen, Farben, Lage; Einheit automatisch in Meter) –
Details im Abschnitt „CAD-Import" in `../README.md`.

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
- MuJoCo laedt nur **binaeres** STL mit hoechstens 200000 Dreiecken.
  ASCII-STL repariert der Editor automatisch; `./launch.sh check-meshes`
  prueft den Bestand.
- **scale** nicht vergessen: STL aus CAD ist oft in Millimetern, MuJoCo
  rechnet in Metern -> `scale="0.001 0.001 0.001"`.
- Fuer **Kollision** sollte das Mesh geschlossen (watertight) und moeglichst
  konvex sein. Komplexe Meshes kollidieren sonst nur mit ihrer konvexen
  Huelle. Reine Deko: `contype="0" conaffinity="0"` am geom setzen.
