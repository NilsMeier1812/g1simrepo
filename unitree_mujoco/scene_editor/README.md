# Scene Editor – MuJoCo-Umgebungen einfach bauen

Kleiner Baukasten, um MuJoCo-Umgebungen fuer den G1 zu bauen: Objekte per
Maus platzieren, eigene **Meshes (STL/OBJ/PLY/GLB/...) und CAD-Dateien
(STEP/IGES) importieren** – auch ganze Anlagen/Roboterzellen – und das Ergebnis
als MuJoCo-XML exportieren. Grundlage ist der
[`mujoco-scene-editor`](https://github.com/markusgrotz/mujoco-scene-editor)
(browserbasierter Editor) plus zwei fertige Beispiel-Szenen und ein paar
Helfer-Skripte.

> Ergaenzt das vorhandene `terrain_tool/` (Terrain per Python-Skript). Der
> Scene Editor ist der **visuelle** Weg: Objekte/Meshes per Maus setzen.

> **Zentraler Umgebungs-Ordner:** `scene_editor/scenes/`. Alles, was hier als
> `*.xml` liegt, ist die Liste der Umgebungen – waehlbar **im Editor**
> (`./launch.sh`) UND **beim G1-Start** (`g1pilot/start.sh`, Sim). Eine neue
> Umgebung im Editor unter `scenes/` speichern reicht, damit sie an beiden
> Stellen auftaucht.

## Konzept: Basis + Umgebung

Das System trennt **Basis** (immer gleich) von **Umgebung** (wechselt):

| | kommt aus | Inhalt |
|---|---|---|
| **Basis** (immer automatisch) | `build_env_scene.py` | G1-Roboter, **Lichtquelle**, Boden, **Weld** (haelt den G1 anfangs fest), visual/statistic – die technischen Grunddinge |
| **Umgebung** (waehlbar) | `scenes/<name>.xml` | **nur Hindernisse / Objekte zum Interagieren/Greifen** |

Du baust in einer Umgebung also **nur die Objekte**. G1, Licht, Boden und Weld
werden beim Laden immer automatisch dazugefuegt – nie in die Umgebung schreiben.
(Legt der Editor doch mal einen eigenen Boden/eine Lichtquelle an, wirft der
Generator sie beim Kombinieren automatisch raus.)

### Hindernis vs. Greif-Objekt (fuer Nav/IK)

Objekte werden nach RViz uebertragen (siehe `g1pilot/docs/51_navigation_technik.md`) und dort
in zwei Klassen unterschieden — **Hindernis** (der Arm weicht aus) und
**Greif-Objekt** (die Hand darf ran, das Objekt bewegt sich beim Anfassen mit).
Klassifikation per **Namenskonvention**: benennst du ein Objekt (im Editor im
Ordner **„Objekt umbenennen"**) mit dem Praefix **`grasp_`** (Gross-/
Kleinschreibung egal, z.B. `grasp_apfel`), macht `build_env_scene.py` beim
Kombinieren automatisch einen **freien, beweglichen Koerper** daraus
(`<freejoint/>`) — nur so kann MuJoCo es beim Greifen/Anfassen bewegen. Alle
anderen Objekte werden zu statischen Hindernissen.

```
box_demo        -> Hindernis (statisch, der Arm weicht aus)
grasp_apfel      -> Greif-Objekt (beweglich, die Hand darf ran)
```

> **Der Name entscheidet, nicht der Editor-Export.** Der Editor haengt jedes per
> Maus gesetzte Objekt in einen eigenen Koerper mit freiem Gelenk — ohne Umbau
> wuerde die halbe Umgebung beim Start umfallen. `build_env_scene.py`
> normalisiert deshalb beim Kombinieren **alles** auf die kanonische Form
> (Hindernis = statisches `<geom>`, Greif-Objekt = `<body>` mit `<freejoint/>`),
> allein anhand des Namens. Du musst also nichts von Hand nachbauen — nur
> sinnvoll benennen.

---

## Was ist drin?

```
scene_editor/
├── setup.sh                    # einmaliges Setup (virtualenv + Installation)
├── launch.sh                   # Menue / Editor / Viewer starten
├── run_editor.py               # Editor-Start + Speichern-nur-mit-Name, Umbenennen, Import
├── cad_import.py               # CAD/Mesh -> Einzelteil-Meshes + Umgebung (auch als CLI)
├── mesh_utils.py               # STL pruefen/reparieren (nur Standardbibliothek)
├── build_env_scene.py          # kombiniert G1 + Umgebung (nutzt start.sh)
├── test_*.py                   # Tests (python3 test_<name>.py)
├── requirements.txt
├── meshes/
│   ├── sample_crate.stl        # Beispiel-STL zum Import-Testen
│   ├── sample_ramp.stl
│   ├── sample_bracket.step     # Beispiel-STEP zum CAD-Import-Testen
│   ├── cad/<name>/             # Einzelteile je CAD-Import (automatisch, nicht im Git)
│   └── uploads/                # im Editor hochgeladene CAD-Dateien (nicht im Git)
└── scenes/
    └── environment_starter.xml # Umgebung OHNE Roboter -> das bearbeitest du

../unitree_robots/g1/
    ├── scene_g1_playground.xml # statisches Beispiel: G1 + Objekte + STLs
    └── scene_env_<name>.xml     # auto-generiert bei Umgebungs-Auswahl (start.sh)
```

**Umgebung im G1-Sim laden:** `g1pilot/start.sh` (Sim) fragt jetzt „Welche
Umgebung laden?" und listet alle `scenes/*.xml` auf – der G1 wird unveraendert
hineingeladen (siehe Abschnitt 4).

---

## 1. Setup (einmalig)

```bash
cd unitree_mujoco/scene_editor
./setup.sh
```

Das legt ein eigenes `.venv/` an und installiert den Editor dort hinein
(die Systemumgebung wird nicht angefasst).

> **Der Editor laeuft bewusst auf dem Host, nicht im Docker-Stack.** Docker
> mountet von hier nur `meshes/` (read-only) und laedt die fertige Szenen-XML.
> Dieses `setup.sh` ist also das einzige Setup, das der Editor braucht – einmal,
> danach nie wieder.
>
> Fehlt in einem **aelteren** `.venv` ein spaeter dazugekommenes Paket (z.B.
> `cadquery-ocp`/`coacd` fuer den CAD-Import), installiert `launch.sh` es beim
> naechsten Start **automatisch nach** – `setup.sh` musst du dafuer nicht erneut
> aufrufen.

Warum ein eigenes venv? Der Editor zieht `GPUtil` mit, das sich mit dem
alten System-`setuptools` **nicht bauen** laesst. Im frischen venv mit
aktuellem `setuptools` klappt es. Ausserdem fehlt dem Editor-Paket die
Abhaengigkeit `yourdfpy` – die installiert `setup.sh` gleich mit.

> **Erster Start braucht Internet:** Der Editor laedt beim allerersten Mal
> einmalig den Objaverse-Objektkatalog (Online-3D-Bibliothek) herunter und
> cached ihn. Danach laeuft er offline.

---

## 2. Starten – interaktives Menue

Einfach ohne Argument starten:

```bash
./launch.sh
```

Das listet **alle Umgebungen nummeriert** auf (aus dem zentralen Ordner
`scene_editor/scenes/` – dieselben, die auch beim G1-Start waehlbar sind):

```
=================== MuJoCo Scene Editor ===================
Umgebungen in scene_editor/scenes/
(dieselben, die auch beim G1-Start via g1pilot/start.sh waehlbar sind)
----------------------------------------------------------
    1) environment_starter
    2) kueche
    n) neue leere Umgebung im Editor
    q) beenden
----------------------------------------------------------
Auswahl (Zahl / n / q): 1

Gewaehlt: environment_starter
Aktion:
   e) im Editor bearbeiten
   v) allein im Viewer ansehen (ohne Roboter)
   g) mit dem G1 im Viewer ansehen
Auswahl [e/v/g] (Default e):
```

- Zahl = Umgebung waehlen, dann:
  - **e** – im Editor bearbeiten
  - **v** – allein im Viewer ansehen (ohne Roboter)
  - **g** – **mit dem G1** im Viewer ansehen (kombiniert automatisch, ohne
    Docker-Stack)
- **n** = mit einer leeren Umgebung neu anfangen.

Neue Umgebungen aus `scenes/` tauchen automatisch in der Liste auf – hier
**und** beim G1-Start.

## 3. Umgebung im Editor bauen & speichern

Der Editor startet einen lokalen Webserver und oeffnet den Browser
(**http://127.0.0.1:8080**). Dort kannst du:

- **Nur Objekte bauen** – Hindernisse / Greifobjekte. Boden, Licht, G1 und
  Weld kommen automatisch aus der Basis (nicht selbst anlegen).
- **Shapes platzieren** – Box, Kugel, Zylinder ... per Maus setzen/verschieben
- **Eigene STLs / STEP-CAD-Dateien importieren** – siehe eigener Abschnitt unten
  (der Knopf steckt im zugeklappten Ordner **„Add Assets from File"**).
- **Objekte benennen** – Ordner **„Objekt umbenennen"**: Objekt oben unter
  „Elements" waehlen, neuen Namen eintippen, **`Umbenennen`**. Praefix
  **`grasp_`** = greifbares Objekt, alles andere = Hindernis (siehe oben).
- **Speichern** – Ordner **„Umgebung speichern"** ganz oben. Dort steht **nur
  ein Feld: der Name** (z.B. `kueche`) – kein Pfad. Klick auf **`Speichern`**
  schreibt `scenes/kueche.xml` (+ `.json` zum spaeteren Weiterbearbeiten), und
  die Umgebung ist sofort im Menue **und** beim G1-Start waehlbar.

Beim Speichern passiert ausserdem automatisch:

- **Meshes werden mitgenommen.** Der Editor merkt sich absolute Pfade
  (`/home/du/Downloads/kiste.stl`) – die gibt es im Docker-Container nicht.
  Dateien ausserhalb des Repos landen darum in `meshes/imported/` und werden im
  XML relativ (`../meshes/...`) referenziert.
- **Kontrolle:** die gespeicherte Umgebung wird sofort testweise mit MuJoCo
  geladen. Klappt das nicht, sagt die Meldung im Editor warum (gespeichert wird
  trotzdem – deine Arbeit geht nie verloren).
- **Kein Beifang mehr:** frueher legte der Export zusaetzlich eine Datei
  `MuJoCo Model.xml` in `scenes/` ab, die dann als Geister-Umgebung in jeder
  Auswahlliste stand. Die wird jetzt weggeraeumt.

> „Umgebung speichern", „Objekt umbenennen", der Upload-Knopf und „Mesh
> skalieren" kommen aus `run_editor.py`; `launch.sh` startet den Editor immer
> darueber. Der eingebaute Export mit Pfad-Eingabe ist ausgeblendet.

### Eigene Meshes / CAD-Dateien in den Editor importieren

Es gibt zwei Wege – nimm den, der dir lieber ist:

**A) Datei-Dialog (beliebiger Ordner) – am einfachsten**

Ganz oben im Editor gibt es den Ordner **„Eigene Datei hochladen"** mit dem
Knopf **„Datei waehlen (STL/OBJ/STEP/...)"**:

1. Knopf klicken -> es oeffnet sich der **Datei-Dialog deines Systems**.
2. Datei aus **irgendeinem Ordner** waehlen: STL, OBJ, **STEP/STP, IGES/IGS,
   BREP**, PLY, GLB/GLTF, 3MF, OFF, DAE.
3. Fertig:
   * **STL/OBJ**, die MuJoCo direkt laden kann, kommen wie bisher als ein
     Mesh-Objekt in die Szene (ASCII-STL wird dabei binaer neu geschrieben).
   * **Alles andere** (CAD, fremde Formate, STL ueber 200000 Dreiecke) laeuft
     durch den CAD-Import (naechster Abschnitt) und kommt als **Gruppe aus
     Einzelteilen** in die Szene – mit Namen, Farben und Lage aus der Datei.
     Die ganze Gruppe unter „Elements" waehlen, um sie zu verschieben.

> Dieser Knopf wird von `run_editor.py` ergaenzt (der eingebaute Editor hat
> nur den Ordner-Scan unten). `launch.sh` startet den Editor immer darueber.

**B) Ordner-Scan (der eingebaute Weg)**

Der Editor kann auch einen Ordner nach Mesh-Dateien (STL, OBJ, PLY, GLB/GLTF,
USD) durchsuchen. Dieser Ordner ist fest auf `scene_editor/meshes/` vorbelegt:

1. Mesh nach `scene_editor/meshes/` kopieren.
2. Ordner **„Add Assets from File"** aufklappen (standardmaessig zugeklappt).
3. **„Scan assets"** klicken -> Meshes erscheinen im Dropdown
   (`sample_crate`/`sample_ramp` sind schon da).
4. Auswaehlen -> **„Add asset"**. Zu grosse Netze und Formate, die MuJoCo nicht
   kann (PLY/GLB), werden dabei automatisch ueber den CAD-Import zerlegt.

> Der eingebaute Editor-Default `~/temp/ArmarXObjects` existiert nicht – darum
> war die Liste vorher leer und der Import schien zu fehlen.

**CAD-Dateien aus `meshes/`** (STEP & Co. kennt der Ordner-Scan nicht): Ordner
**„CAD-Import"** aufklappen -> **„Liste aktualisieren"** -> Datei waehlen ->
**„Importieren & einfuegen"**.

### CAD-Import: STEP/IGES, ganze Anlagen

**MuJoCo kann kein CAD laden.** STEP/IGES beschreiben exakte Flaechen
(BRep/NURBS), MuJoCo braucht Dreiecksnetze (STL/OBJ/MSH). `cad_import.py`
**tesseliert** die Datei deshalb – und zwar **Bauteil fuer Bauteil**, so wie
sie im CAD aufgebaut ist:

| Was | Warum |
|---|---|
| **Je Bauteil (Volumenkoerper) ein Mesh** | MuJoCo kollidiert ueber die konvexe Huelle je Mesh. Eine ganze Zelle als ein Mesh waere ein einziger Klotz – und haette Millionen Dreiecke (MuJoCo-Grenze: 200000 je STL). |
| **Mehrfach verbaute Teile nur einmal gespeichert** | Profile, Schrauben usw. teilen sich eine Datei. |
| **Namen, Farben, Lage aus dem CAD** | Die Szene sieht aus wie im CAD; Teile heissen wie dort (`Zellenwand`, `Itemprofil_40x40x2120_3`). |
| **Einheit automatisch** | mm/inch/m aus der Datei -> Meter. Nur bei Meshes ohne Einheit ggf. Skalierung setzen. |
| **Kleine Teile nur Optik** | Schrauben, Knoepfe (Diagonale < 3 cm) kollidieren nicht – spart Rechenzeit. |
| **Konkave Teile konvex zerlegt** | Eine L-foermige Wand oder ein hohles Gehaeuse wuerde als konvexe Huelle die halbe Zelle fuellen. Solche Teile bekommen unsichtbare Kollisions-Stuecke `<teil>__k<n>` (Gruppe 3, braucht `coacd`). |
| **Offene/flache Teile** | Bleche/Schilder ohne Volumen bekommen `inertia="shell"` (sonst: „mesh volume is too small"). |
| **Platzierung `auto`** | Boden (bzw. Oberkante einer Bodenplatte) auf z=0; bei begehbaren Modellen steht der G1 (immer im Ursprung) auf einem freien Platz moeglichst mittig, kleine Objekte kommen 1 m vor den G1. Steht der G1 doch in einem Teil, sagt der Import das. |

Ergebnis: `meshes/cad/<name>/` (Einzelteile + Manifest `cad_import.json`) und –
beim Import auf der Kommandozeile – direkt eine fertige Umgebung
`scenes/<name>.xml`. Wird dieselbe Datei mit denselben Einstellungen noch einmal
importiert, kommt das Ergebnis sofort aus dem Cache.

Beispiel (NX-Export einer Roboterzelle, 43 MB STEP): 698 Teile, 315
verschiedene Meshes, 4 konkave Teile zerlegt – ca. 20 s Einlesen/Vernetzen plus
ca. 2 min fuer die konvexe Zerlegung, Laden in MuJoCo 0,3 s.

**Einstellungen** (im Editor Ordner **„CAD-Import"**, auf der Kommandozeile als
Optionen):

- **Genauigkeit** `coarse` / `normal` (Default) / `fine` – wie fein Rundungen
  vernetzt werden (mehr Dreiecke = schoener, aber langsamer).
- **Skalierung** `0` = automatisch, sonst fester Faktor (z.B. `0.001`).
- **Platzierung** `auto` (Default) / `floor` / `center` / `cad`.
- **Nur Optik unter (m)** – Grenze fuer „kleine Teile".

Auf der Kommandozeile (ohne Editor) – das Ergebnis ist sofort beim G1-Start
waehlbar:

```bash
./launch.sh import ~/Downloads/zelle.stp                  # -> scenes/zelle.xml
./launch.sh import zelle.stp --name demo --quality coarse
./launch.sh import teil.step --place cad                  # CAD-Koordinaten behalten
./launch.sh import --help                                 # alle Optionen
```

Technisch steckt dahinter **OpenCascade** (`cadquery-ocp`) fuer CAD,
**trimesh** fuer Mesh-Formate und **CoACD** (`coacd`) fuer die konvexe
Zerlegung – alles reine pip-Pakete, `setup.sh`/`launch.sh` installieren sie.
Fehlt `coacd`, kollidieren konkave Teile als gefuellter Block (der Import sagt,
welche). **JT, Parasolid (x_t), SolidWorks, Inventor** kann keine freie
Bibliothek lesen – im CAD als STEP (AP214/AP242) exportieren.

### STLs bewegen & skalieren

**Auswaehlen:** oben im Dropdown **„Elements"** das Mesh waehlen (oder im
3D-Fenster direkt draufklicken – „Allow mouse selection" ist an).

**Bewegen / Drehen:** ist das Mesh gewaehlt, erscheint der Zieh-Gizmo
(Pfeile/Ringe). Steuert die Checkbox **„Interactive translation"** (standard-
maessig **an**). Alternativ im **„Transform"**-Feld **Position (m)** / **Angles
(deg)** eintippen und **„Set Transform"** klicken. Sieht es aus, als bewege sich
nichts: erst ein Element auswaehlen und ggf. „Interactive translation" pruefen.

**Skalieren:** der eingebaute Editor kann Meshes **nicht** skalieren (nur
Box/Zylinder/Kugel-Masse). Dafuer gibt es den ergaenzten Ordner
**„Mesh skalieren"**:

1. Mesh oben unter „Elements" auswaehlen (das Feld zeigt dann seinen aktuellen
   Faktor).
2. **Faktor** eingeben (z.B. `2` = doppelt so gross; **CAD-STL in mm -> `0.001`**).
3. **„Auf gewaehltes Mesh anwenden"**.

> Upload-Button und „Mesh skalieren" ergaenzt `run_editor.py`; `launch.sh`
> startet den Editor immer darueber.

---

## 4. Umgebung im G1-Sim laden (der Hauptweg)

Der G1 bleibt **immer gleich** – du waehlst beim Start nur die Umgebung, und
der Roboter wird unveraendert hineingeladen. Beim G1-Start (`g1pilot/start.sh`,
Sim-Modus) kommt jetzt die Frage:

```
2d) Welche Umgebung laden? (G1 wird unveraendert hineingeladen)
   * 1) Standard — aktuelles Terrain (scene.xml)
     2) environment_starter
     3) kueche
     ...
```

Jede `scene_editor/scenes/*.xml` taucht hier automatisch als Auswahl auf.
Waehlst du eine, passiert Folgendes automatisch:

1. `build_env_scene.py` baut auf dem Host eine kombinierte Szene
   `unitree_robots/g1/scene_env_<name>.xml` = **G1 + deine Umgebung**
   (mit passender Hand-Variante, Mesh-Pfade korrekt umgerechnet).
2. `start.sh` setzt `G1_ENV=<name>`; `config.py` laedt dann diese Szene.

Du musst also **nichts** von Hand zusammenkopieren – Umgebung im Editor bauen,
speichern, beim Start auswaehlen, fertig.

### Schnell ohne den G1-Stack ansehen

```bash
./launch.sh view-g1       # statisches Beispiel G1 + Objekte
```

Oder eine kombinierte Szene manuell erzeugen und im Viewer pruefen:

```bash
python3 build_env_scene.py --env scenes/kueche.xml --inspire 0
.venv/bin/python -m mujoco.viewer --mjcf=../unitree_robots/g1/scene_env_kueche.xml
```

---

## Wie haengt das zusammen?

| Datei | Zweck | Inhalt |
|-------|-------|----------|
| `scenes/*.xml` | **Umgebungen – die baust du im Editor** | nur Objekte |
| `build_env_scene.py` | baut **Basis** (G1+Licht+Boden+Weld) und mischt die Objekte ein | – |
| `scene_env_<name>.xml` (im g1-Ordner, auto-generiert) | **das laedt der Sim** | Basis + Objekte |

Der Generator schreibt die feste Basis (G1, Lichtquelle, Boden, Skybox und den
Weld `hold_base_weld`, der den G1 anfangs an `torso_link` festhaelt) selbst und
haengt nur die Objekte der Umgebung an. Eigene Boeden/Lichter/doppelte
Asset-Namen aus der Umgebung werden dabei automatisch aussortiert.

Warum ueberhaupt ein Generator und kein simples `<include>`? Das G1-Modell setzt
`<compiler meshdir="meshes">`, und dieses `meshdir` gilt in MuJoCo **global** –
auch fuer deine eigenen Meshes. Die kombinierte Szene muss im g1-Ordner liegen
(sonst fehlen die Roboter-Meshes), und eigene Mesh-Pfade muessen dazu passen.
`build_env_scene.py` erledigt das: kombinierte Szene in den g1-Ordner, Mesh-Pfade
der Umgebung relativ umgerechnet (gilt auch im read-only gemounteten
Docker-Container).

> `scene_g1_playground.xml` (im g1-Ordner) ist ein **statisches Beispiel** von
> Hand. Der eigentliche Weg fuer eigene Umgebungen ist der Generator oben.

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
./launch.sh                 # interaktives Menue (Umgebungen nummeriert waehlen)
./launch.sh list            # vorhandene Umgebungen auflisten
./launch.sh new             # leere Umgebung im Editor
./launch.sh edit [name]     # Umgebung im Editor (Default: environment_starter)
./launch.sh prompt "..."    # Umgebung per Text-Prompt generieren (braucht OPENAI_API_KEY)
./launch.sh view [name]     # Umgebung allein im MuJoCo-Viewer ansehen
./launch.sh with-g1 [name]  # Umgebung + G1 im MuJoCo-Viewer ansehen
./launch.sh view-g1         # statisches Beispiel scene_g1_playground.xml
./launch.sh import <datei>  # CAD/Mesh (STEP, IGES, STL, GLB, ...) -> scenes/<name>.xml
./launch.sh check-meshes    # alle STL in meshes/ auf MuJoCo-Tauglichkeit pruefen
```

`[name]` darf `kueche`, `kueche.xml`, `scenes/kueche.xml` oder ein absoluter
Pfad sein – es wird immer in `scenes/` nachgeschlagen; bei einem Tippfehler
listet `launch.sh` die vorhandenen Umgebungen auf. Anderer Port fuer den
Editor: `SCENE_EDITOR_PORT=8081 ./launch.sh edit kueche`.

Nach Aenderungen an der Normalisierung: `python3 test_build_env_scene.py`,
an der STL-Pruefung: `python3 test_mesh_utils.py` (beide nur Standardbibliothek),
am CAD-Import: `.venv/bin/python test_cad_import.py` (im venv laufen alle Tests,
mit blankem `python3` werden die Teile ohne numpy/trimesh/OpenCascade
uebersprungen).

## Bekannte Stolpersteine

- **`stl_decoder: number of faces should be between 1 and 200000 ...
  perhaps this is an ASCII file?`** – MuJoCo laedt nur **binaeres** STL, kein
  ASCII-STL (haeufig bei Downloads/Exporten aus Blender, Sketchfab,
  Objaverse). Wird automatisch behoben: beim Speichern im Editor prueft
  `run_editor.py` alle verwendeten STLs und schreibt ASCII-Dateien binaer neu
  (mit `trimesh`); `build_env_scene.py` macht dieselbe Reparatur beim
  Kombinieren nochmal als Sicherheitsnetz (z.B. fuer Umgebungen, die nicht
  ueber den Editor entstanden sind). Fehlt `trimesh` (nur beim Aufruf mit
  blankem System-`python3` moeglich) oder laesst sich die Datei nicht
  reparieren, wird nur DAS betroffene Objekt weggelassen (mit Warnung) statt
  die ganze Umgebung zu verwerfen – von Hand reparieren: Datei in
  Blender/MeshLab als binaeres STL neu exportieren.
- **`Speichern fehlgeschlagen: AttributeError: 'NoneType' object has no
  attribute 'joints'`** – Bug im Fremdpaket: neue Objekte bekommen als
  Eltern-Pfad das, was gerade oben unter „Elements" ausgewaehlt ist. War dabei
  zufaellig ein normales Objekt (Box/Mesh/...) statt eine echte Gruppe
  ausgewaehlt, haengt der Editor das neue Objekt intern darunter – der
  Exporter kennt aber nur Gruppen als Eltern und stuerzt sonst ab.
  `run_editor.py` faengt das seit kurzem ab: verwaiste Objekte werden vor dem
  Speichern automatisch auf die oberste Ebene gehoben (Hinweis erscheint in
  der Speichern-Meldung) – nichts geht verloren. Vorbeugen: vor „Create Box"
  o.ae. in „Elements" erst **„— none —"** waehlen, wenn du keine Gruppe
  meinst.
- **`ModuleNotFoundError: yourdfpy`** – `setup.sh` erneut laufen lassen
  (installiert es); oder `.venv/bin/pip install yourdfpy`.
- **Build-Fehler bei `GPUtil` / `install_layout`** – passiert nur bei
  Installation in die Systemumgebung. Immer das venv aus `setup.sh` nutzen.
- **CAD-Import inaktiv / „OpenCascade fehlt"** – `launch.sh` holt die Pakete
  beim naechsten Start automatisch nach (braucht einmalig Internet). Steht es
  danach immer noch da: **Editor neu starten** (der Ordner „CAD-Import" wird nur
  beim Start aufgebaut). Von Hand: `.venv/bin/pip install cadquery-ocp coacd`;
  pruefen mit `.venv/bin/python cad_import.py --check` (Exit 0 = alles da).
- **`stl_decoder: number of faces should be between 1 and 200000` /
  „perhaps this is an ASCII file?"** – MuJoCos zwei STL-Grenzen: es laedt **nur
  binaeres** STL und **hoechstens 200000 Dreiecke** je Datei. Beides faengt der
  Editor selbst ab:
  * ASCII-STL wird beim Import automatisch binaer neu geschrieben.
  * Zu grosse Netze (auch per „Add asset") werden ueber den CAD-Import in
    mehrere Meshes geteilt statt abgelehnt; CAD-Baugruppen werden ohnehin
    Bauteil fuer Bauteil vernetzt.

  Bestand pruefen: `./launch.sh check-meshes`.
- **`mesh volume is too small ... Try setting inertia to shell`** – ein
  offenes/flaches Mesh (Blech, Schild). Der CAD-Import setzt dafuer automatisch
  `inertia="shell"`, auch beim Speichern im Editor und beim Kombinieren. Bei
  eigenen Meshes: `<mesh ... inertia="shell"/>` von Hand setzen.
- **CAD-Teil ist 1000x zu gross/klein** – bei STEP/IGES kommt die Einheit aus
  der Datei; falsch ist sie praktisch nur bei Meshes (STL/OBJ haben keine
  Einheit) oder BREP (mm angenommen). Im Ordner „CAD-Import" die Skalierung
  setzen (z.B. `0.001`) und neu importieren, oder `--scale` auf der
  Kommandozeile.
- **Der G1 steht beim Start in einem Teil der Anlage** – der Import meldet das
  („ACHTUNG: Der G1 startet ... IN ..."). Mit Platzierung `auto` importieren
  (sucht einen freien Platz) oder die Gruppe im Editor verschieben; in der
  erzeugten XML ist es das `pos` des einen `<body>` direkt unter `<worldbody>`.
- **Roboter kommt nicht in die Zelle / alles blockiert** – ein konkaves Teil
  (L-Wand, hohles Gehaeuse) kollidiert als gefuellter Block, weil `coacd` fehlt.
  Der Import nennt die Teile; `.venv/bin/pip install coacd` und neu importieren.
- **Absturz beim Start mit `403 Forbidden` / objaverse** – kein/gesperrtes
  Internet beim ersten Start. Der Objaverse-Katalog wird beim ersten Lauf
  einmalig heruntergeladen; mit Internet einmal starten, danach offline ok.
- **`Port 8080 ist schon belegt`** – es laeuft noch ein Editor. In der GUI:
  *Laufende Prozesse* -> *Stoppen*. Oder anderen Port nehmen:
  `SCENE_EDITOR_PORT=8081 ./launch.sh edit kueche`.
- **Umgebung erscheint nicht in der Auswahl** – der Dateiname darf nur
  `A-Z a-z 0-9 _ -` enthalten (er wird als `G1_ENV` weitergereicht) und die
  Datei muss ein `<mujoco>`-XML sein. Die GUI zeigt aussortierte Dateien unter
  *Umgebungen bearbeiten* mit Begruendung an.
- **`Umgebung ... laesst sich nicht laden`** beim Start – die kombinierte Szene
  kompiliert nicht (meist ein fehlendes Mesh oder ein doppelter Name). Genaue
  Meldung: *Umgebungen bearbeiten* -> **„Auf Ladbarkeit pruefen"** bzw.
  `python3 build_env_scene.py --env <name>`.
- **Objekte fallen beim Start um** – das war der alte Zustand (der Editor
  exportiert alles mit freiem Gelenk). Heute wird normalisiert: beweglich ist
  nur noch, was `grasp_` im Namen hat.
- **Editor mangelt beim Import einer Roboter-Szene** – bekannt: der Editor
  verwirft beim XML-Import manche Tags (Joints/Aktuatoren ausserhalb der
  Roboterbeschreibung, Reibung). Deshalb im Editor nur die roboterfreie
  Umgebung bauen, nicht die volle G1-Szene.
