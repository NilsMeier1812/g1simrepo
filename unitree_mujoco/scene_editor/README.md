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

---

## Was ist drin?

```
scene_editor/
├── setup.sh                    # einmaliges Setup (virtualenv + Installation)
├── launch.sh                   # Menue / Editor / Viewer starten
├── run_editor.py               # Editor-Start mit festem Export-Pfad (scenes/)
├── build_env_scene.py          # kombiniert G1 + Umgebung (nutzt start.sh)
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

- **Shapes platzieren** – Box, Kugel, Zylinder ... per Maus setzen/verschieben
- **Meshes importieren** – `Add Asset` -> eigenes STL/OBJ vom Dateisystem
  (z.B. `meshes/sample_crate.stl`)
- **Speichern** – im `Export`-Feld ist der Pfad schon fest auf den
  `scenes/`-Ordner vorbelegt. Du tippst nur noch **den Namen** (z.B.
  `scene.xml` -> `kueche.xml`) und klickst **`Export scene`**. Ergebnis:
  `scenes/kueche.xml` (+ `.json` zum spaeteren Weiterbearbeiten) – und die
  Szene erscheint beim naechsten `./launch.sh` direkt im Menue.

> Der feste Speicherpfad kommt aus `run_editor.py` (setzt den Export-Default
> auf `scenes/`). `launch.sh` startet den Editor immer darueber.

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

| Datei | Zweck | Roboter? |
|-------|-------|----------|
| `scenes/*.xml` | **Umgebungen – die baust du im Editor** | nein |
| `build_env_scene.py` | kombiniert G1 + Umgebung (macht `start.sh` automatisch) | – |
| `scene_env_<name>.xml` (im g1-Ordner, auto-generiert) | **das laedt der Sim** | ja (G1) |

Warum der Generator und kein simples `<include>`? Das G1-Modell setzt
`<compiler meshdir="meshes">`, und dieses `meshdir` gilt in MuJoCo **global**
– auch fuer deine eigenen Meshes. Eine Szene, die den Roboter einbindet, muss
im g1-Ordner liegen (sonst fehlen die Roboter-Meshes), und eigene Mesh-Pfade
muessen dazu passen. `build_env_scene.py` erledigt genau das: es legt die
kombinierte Szene in den g1-Ordner und rechnet die Mesh-Pfade der Umgebung
passend um (relativ, damit sie auch im read-only gemounteten Docker-Container
stimmen). Der Editor selbst exportiert bewusst **roboterfreie** Szenen – so
schneidet er nie am Robotermodell herum.

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
./launch.sh new             # leere Umgebung im Editor
./launch.sh edit [datei]    # Umgebung im Editor (Default: environment_starter.xml)
./launch.sh prompt "..."    # Umgebung per Text-Prompt generieren (braucht LLM-API-Key)
./launch.sh view [datei]    # Umgebung allein im MuJoCo-Viewer ansehen
./launch.sh with-g1 [datei] # Umgebung + G1 im MuJoCo-Viewer ansehen
./launch.sh view-g1         # statisches Beispiel scene_g1_playground.xml
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
