# Scene Editor – MuJoCo-Umgebungen einfach bauen

Kleiner Baukasten, um MuJoCo-Umgebungen fuer den G1 zu bauen: Objekte per
Maus platzieren, eigene **STL/OBJ-Meshes importieren** und das Ergebnis als
MuJoCo-XML exportieren. Grundlage ist der
[`mujoco-scene-editor`](https://github.com/markusgrotz/mujoco-scene-editor)
(browserbasierter Editor) plus zwei fertige Beispiel-Szenen und ein paar
Helfer-Skripte.

> Ergaenzt das vorhandene `terrain_tool/` (Terrain per Python-Skript). Der
> Scene Editor ist der **visuelle** Weg: Objekte/Meshes per Maus setzen.

---

## Was ist drin?

```
scene_editor/
├── setup.sh                    # einmaliges Setup (virtualenv + Installation)
├── launch.sh                   # Editor / Viewer starten
├── requirements.txt
├── meshes/
│   ├── sample_crate.stl        # Beispiel-STL zum Import-Testen
│   └── sample_ramp.stl
└── scenes/
    └── environment_starter.xml # Umgebung OHNE Roboter -> das bearbeitest du

../unitree_robots/g1/
    └── scene_g1_playground.xml # lauffaehig: G1 + Objekte + eigene STLs
```

---

## 1. Setup (einmalig)

```bash
cd unitree_mujoco/scene_editor
./setup.sh
```

Das legt ein eigenes `.venv/` an und installiert den Editor dort hinein
(die Systemumgebung wird nicht angefasst).

Warum ein eigenes venv? Der Editor zieht `GPUtil` mit, das sich mit dem
alten System-`setuptools` **nicht bauen** laesst. Im frischen venv mit
aktuellem `setuptools` klappt es. Ausserdem fehlt dem Editor-Paket die
Abhaengigkeit `yourdfpy` – die installiert `setup.sh` gleich mit.

> **Erster Start braucht Internet:** Der Editor laedt beim allerersten Mal
> einmalig den Objaverse-Objektkatalog (Online-3D-Bibliothek) herunter und
> cached ihn. Danach laeuft er offline.

---

## 2. Umgebung im Editor bauen

```bash
./launch.sh edit          # oeffnet scenes/environment_starter.xml
# oder leer anfangen:
./launch.sh new
```

Der Editor startet einen lokalen Webserver und oeffnet den Browser
(**http://127.0.0.1:8080**). Dort kannst du:

- **Shapes platzieren** – Box, Kugel, Zylinder ... per Maus setzen/verschieben
- **Meshes importieren** – `Add Asset` -> eigenes STL/OBJ vom Dateisystem
  (z.B. `meshes/sample_crate.stl`)
- **exportieren** – `Export` schreibt die Szene als MuJoCo-XML zurueck

Zum schnellen Ansehen ohne Roboter:

```bash
./launch.sh view          # environment_starter.xml im MuJoCo-Viewer
```

---

## 3. Mit dem G1 zusammen ansehen

```bash
./launch.sh view-g1       # G1 + Objekte (scene_g1_playground.xml)
```

Und im Unitree-Simulator nutzen – in `simulate_python/config.py`:

```python
ROBOT_SCENE = "../unitree_robots/g1/scene_g1_playground.xml"
```

(bzw. `robot_scene` in `simulate/config.yaml` fuer den C++-Sim).

---

## Wie haengt das zusammen? (wichtig)

Es gibt bewusst **zwei** Szenen-Dateien, weil MuJoCo bei Roboter-Meshes
zickt (siehe unten):

| Datei | Zweck | Roboter? |
|-------|-------|----------|
| `scenes/environment_starter.xml` | **die bearbeitest du im Editor** | nein |
| `../unitree_robots/g1/scene_g1_playground.xml` | **die startest du im Sim** | ja (G1) |

**Objekte aus dem Editor in die G1-Szene uebernehmen:** die `<geom>`- (und
bei Meshes die `<mesh>`-) Zeilen aus deiner exportierten Szene nach
`scene_g1_playground.xml` kopieren. Einziger Unterschied:

- **Primitive** (box/sphere/cylinder ...) 1:1 kopieren.
- **Eigene STLs**: das Pfad-Prefix anpassen
  - in `environment_starter.xml`:  `file="../meshes/xxx.stl"`
  - in `scene_g1_playground.xml`:  `file="../../../scene_editor/meshes/xxx.stl"`

### Warum zwei Dateien / warum die Pfad-Umstellung?

Das G1-Modell (`g1_29dof.xml`) setzt `<compiler meshdir="meshes">`. Dieses
`meshdir` gilt in MuJoCo **global fuer die ganze zusammengesetzte Szene** –
auch fuer deine eigenen Meshes. Eine Szene, die den Roboter per `<include>`
einbindet, muss deshalb im g1-Ordner liegen (sonst findet MuJoCo die
Roboter-Meshes nicht), und eigene Mesh-Pfade sind dann relativ zu
`unitree_robots/g1/meshes/` zu sehen – daher das `../../../`-Prefix.

Der Editor exportiert dagegen **eigenstaendige** Szenen (ohne Roboter). Genau
deshalb bearbeitest du die roboterfreie `environment_starter.xml` und legst
den G1 erst in `scene_g1_playground.xml` obendrauf.

---

## Eigene STLs – Kurzreferenz

```xml
<asset>
  <mesh name="tisch" file="../meshes/tisch.stl" scale="0.001 0.001 0.001"/>
</asset>
<worldbody>
  <geom type="mesh" mesh="tisch" pos="1 0 0"/>
</worldbody>
```

- MuJoCo mag **binaeres** STL; `scale` beachten (CAD ist oft in mm).
- Fuer Kollision: Mesh moeglichst geschlossen/konvex. Reine Deko:
  `contype="0" conaffinity="0"`.

Mehr dazu in `meshes/README.md`.

---

## launch.sh – Uebersicht

```bash
./launch.sh new             # leere Szene im Editor
./launch.sh edit [datei]    # Szene im Editor (Default: environment_starter.xml)
./launch.sh prompt "..."    # Szene per Text-Prompt generieren (braucht LLM-API-Key)
./launch.sh view [datei]    # Szene im MuJoCo-Viewer ansehen
./launch.sh view-g1         # G1 + Objekte ansehen
```

## Bekannte Stolpersteine

- **`ModuleNotFoundError: yourdfpy`** – `setup.sh` erneut laufen lassen
  (installiert es); oder `.venv/bin/pip install yourdfpy`.
- **Build-Fehler bei `GPUtil` / `install_layout`** – passiert nur bei
  Installation in die Systemumgebung. Immer das venv aus `setup.sh` nutzen.
- **Absturz beim Start mit `403 Forbidden` / objaverse** – kein/gesperrtes
  Internet beim ersten Start. Der Objaverse-Katalog wird beim ersten Lauf
  einmalig heruntergeladen; mit Internet einmal starten, danach offline ok.
- **Editor mangelt beim Import einer Roboter-Szene** – bekannt: der Editor
  verwirft beim XML-Import manche Tags (Joints/Aktuatoren ausserhalb der
  Roboterbeschreibung, Reibung). Deshalb im Editor nur die roboterfreie
  Umgebung bauen, nicht die volle G1-Szene.
