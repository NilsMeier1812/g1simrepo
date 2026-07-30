# Scene Editor – MuJoCo-Umgebungen einfach bauen

Kleiner Baukasten, um MuJoCo-Umgebungen fuer den G1 zu bauen: Objekte per
Maus platzieren, eigene **STL/OBJ-Meshes importieren** und das Ergebnis als
MuJoCo-XML exportieren. Grundlage ist der
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

Objekte werden nach RViz uebertragen (siehe `g1pilot/SCENE_BRIDGE.md`) und dort
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
├── run_editor.py               # Editor-Start + Speichern-nur-mit-Name, Umbenennen
├── build_env_scene.py          # kombiniert G1 + Umgebung (nutzt start.sh)
├── test_build_env_scene.py     # Tests der Normalisierung (python3 test_build_env_scene.py)
├── requirements.txt
├── meshes/
│   ├── sample_crate.stl        # Beispiel-STL zum Import-Testen
│   └── sample_ramp.stl
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
- **Eigene STLs importieren** – siehe eigener Abschnitt unten (der Knopf steckt
  im zugeklappten Ordner **„Add Assets from File"**).
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

### Eigene STLs in den Editor importieren

Es gibt zwei Wege – nimm den, der dir lieber ist:

**A) Datei-Dialog (beliebiger Ordner) – am einfachsten**

Ganz oben im Editor gibt es den Ordner **„Eigene Datei hochladen"** mit dem
Knopf **„STL/OBJ waehlen ..."**:

1. Knopf klicken -> es oeffnet sich der **Datei-Dialog deines Systems**.
2. Beliebige STL/OBJ/... aus **irgendeinem Ordner** auswaehlen.
3. Fertig: die Datei wird nach `scene_editor/meshes/` kopiert und **sofort in
   die Szene eingefuegt** (danach wie jedes Objekt per Maus platzierbar).

> Dieser Knopf wird von `run_editor.py` ergaenzt (der eingebaute Editor hat
> nur den Ordner-Scan unten). `launch.sh` startet den Editor immer darueber.

**B) Ordner-Scan (der eingebaute Weg)**

Der Editor kann auch einen Ordner nach Mesh-Dateien (STL, OBJ, PLY, GLB/GLTF,
USD) durchsuchen. Dieser Ordner ist fest auf `scene_editor/meshes/` vorbelegt:

1. STL nach `scene_editor/meshes/` kopieren.
2. Ordner **„Add Assets from File"** aufklappen (standardmaessig zugeklappt).
3. **„Scan assets"** klicken -> STLs erscheinen im Dropdown
   (`sample_crate`/`sample_ramp` sind schon da).
4. Auswaehlen -> **„Add asset"**.

> Der eingebaute Editor-Default `~/temp/ArmarXObjects` existiert nicht – darum
> war die Liste vorher leer und der Import schien zu fehlen.

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
```

`[name]` darf `kueche`, `kueche.xml`, `scenes/kueche.xml` oder ein absoluter
Pfad sein – es wird immer in `scenes/` nachgeschlagen; bei einem Tippfehler
listet `launch.sh` die vorhandenen Umgebungen auf. Anderer Port fuer den
Editor: `SCENE_EDITOR_PORT=8081 ./launch.sh edit kueche`.

Nach Aenderungen an der Normalisierung: `python3 test_build_env_scene.py`
(braucht nur die Standardbibliothek).

## Bekannte Stolpersteine

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
