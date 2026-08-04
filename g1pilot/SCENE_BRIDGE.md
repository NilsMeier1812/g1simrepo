# SCENE_BRIDGE.md — MuJoCo-Umgebungen nach RViz/ROS (Nav + IK)

> **Status: Implementiert** (Stufen 1–3 der Roadmap in §12; LiDAR/§11 und
> Basis-Positionsspeicher/§10 bewusst **noch nicht** gebaut — siehe §15 für den
> genauen Stand, was getestet wurde und was offen bleibt). Diese Datei hält
> die Architektur-Entscheidung fest, wie Umgebungen (Hindernisse **und**
> greifbare Objekte) aus MuJoCo in die ROS-Welt kommen, damit **Navigation**
> (Wege drumherum planen) **und** **IK/Manipulation** (Arm um Hindernisse,
> Objekte greifen, Positionsspeicher) damit arbeiten.

---

## 0 · Das Problem in einem Satz

Umgebungen entstehen heute im **Scene-Editor** als MuJoCo-XML und werden vom
Sim geladen — aber sie existieren **nur in MuJoCo**. Der Nav-Planer und der
IK-Solver sehen sie nicht, können also nicht um Objekte herum planen. Wir
brauchen eine Brücke, die die Umgebung so nach ROS/RViz bringt, dass beide
Engines damit rechnen — inklusive **bewegbarer, greifbarer** Objekte und der
Unterscheidung **Hindernis (ausweichen)** vs. **Grasp-Objekt (Kollision OK)**.

---

## 1 · Ausgangslage im Repo (Fakten)

### 1.1 Die Umgebung ist bereits eine saubere, deklarative Quelle
- Der Scene-Editor schreibt `unitree_mujoco/scene_editor/scenes/<name>.xml` —
  **reine Hindernisse/Objekte** als MuJoCo-`<geom>` (box/cylinder/sphere/mesh).
- `scene_editor/build_env_scene.py` kombiniert **feste Basis (G1 + Licht +
  Boden + Weld)** + Umgebung → `unitree_robots/g1/scene_env_<name>.xml`.
- `G1_ENV` steuert, welche Szene der Sim lädt
  (`unitree_mujoco/simulate_python/config.py`).
- **Konsequenz:** Die Wahrheit über Hindernisse steht in einer XML, die
  Sim **und** ROS lesen können. Wir müssen nichts aus der Physik rekonstruieren.

### 1.2 Wie die Engines Hindernisse konsumieren (heute)

| Engine | Datei | Braucht | Stand heute |
|---|---|---|---|
| **Nav** (Dijkstra) | `navigation/dijkstra_planner.py` ← `navigation/create_map.py` | 2D `nav_msgs/OccupancyGrid` auf `/map` | **Dummy/leer** — Hindernisse hardcoded auskommentiert; Planer plant faktisch geradeaus |
| **IK** (Pinocchio) | `utils/ik_solver.py` | 3D-Kollisionsgeometrie | Nur **Selbstkollision** (Arm↔Körper, `_init_collision_gate`); Umgebung **unbekannt** |
| **RViz** | `config/nav.rviz` | `MarkerArray` / OccupancyGrid | Zeigt Grid, RobotModel, TF, Map, Pfad — **keine** Umgebungs-Objekte |

### 1.3 Zwei Glücksfälle, die viel vereinfachen
- **Frames sind identisch.** MuJoCo-Weltursprung = Startpose des G1 = Ursprung
  des `map`-Frames (`sim_localization` publiziert die Ground-Truth-Pose in
  `map`). Geom-Weltkoordinaten → `map` ist **1:1**, keine Kalibrierung.
  *(Kleiner Fix: `create_map.py` Frame-Default steht auf `odom`, Nav nutzt
  `map` — angleichen.)*
- **Der Kollisionscheck existiert schon.** `_init_collision_gate` prüft im
  250-Hz-Tick die **kommandierte** Konfiguration, bevor sie zum Roboter geht
  (~0,3 ms/Check, Convex-Hulls). Das ist bereits **reaktive** Kollisionsprüfung
  — heute nur binär und nur gegen den eigenen Körper.

---

## 2 · Leitende Erkenntnisse (warum die Architektur so aussieht)

1. **Semantik kommt aus dem Modell, nie aus der Perzeption.** „Hindernis vs.
   Grasp-Objekt" ist Absicht, nicht Geometrie. LiDAR misst nur Punkte und kann
   das prinzipiell nicht unterscheiden. Der Tag muss **im Editor autort** und
   über die **Objekt-Identität** getragen werden.
2. **Ein geteiltes Weltmodell, mehrere Konsumenten.** Nav, reaktive IK und
   geplante IK lesen **dieselbe** Szene. Kein Objekt wird doppelt modelliert.
3. **Bewegbar/greifbar ⇒ Live-State.** Ein gegriffenes Objekt ändert ständig
   seine Pose → das Weltmodell muss live sein (statische XML ist nur der
   eingefrorene Spezialfall).
4. **Kollisionsbewusst ≠ plan-then-execute.** Reaktives Ausweichen in Echtzeit
   ist eine eigene Technik (Servoing mit Geschwindigkeitsskalierung), getrennt
   von globaler Planung. Beide dürfen nebeneinander existieren.
5. **Repräsentation neutral halten.** Die Szene als `moveit_msgs`-artige
   `CollisionObject`/`PlanningScene` + `MarkerArray` ausdrücken → RViz-Anzeige
   gratis **und** jede spätere Komponente (MoveIt, MoveIt Servo) dockt ohne
   Umbau an. Die eigentliche Echtzeit-Mathematik bleibt in Pinocchio (habt ihr
   schon), ohne `move_group` nur zum Halten einer Szene mitzuschleppen.

---

## 3 · Ansätze, die geprüft wurden (und Entscheidung)

| Ansatz | Idee | Urteil |
|---|---|---|
| **A — Statischer XML-Parser** | Szene einmalig parsen → Marker/Grid/Collision | Zu wenig: keine bewegten Objekte. **Wird zum Spezialfall von B.** |
| **B — Live-State-Bridge** | Geom-Posen live aus `mjData` publizieren | **Rückgrat.** Deckt bewegbar/greifbar + statisch ab. |
| **C — Fertige MuJoCo↔ROS-Bridges** (`ubi-agni/mujoco_ros_pkgs`, GSoC-MoveIt) | Sim durch Standard-Bridge ersetzen | Großer Umbau, will Control-Layer übernehmen → kollidiert mit `loco_sim`. **Nur bei MoveIt-Vollmigration.** |
| **D — Sensor-/LiDAR-Sim** | Tiefen-/LiDAR-Punkte → Occupancy/OctoMap | Verliert Identität/Semantik. **Nur additive Ebene für Unbekanntes.** |

**Entscheidung: B als Rückgrat** (identitätsbasierte Live-Bridge mit
semantischem Objekt-Modell). A ist der eingefrorene Sonderfall. D kommt **später
additiv** obendrauf (siehe §8). C nur, falls perspektivisch volle
MoveIt-Migration gewünscht.

---

## 4 · Ziel-Architektur (Gesamtbild)

```
Scene-Editor (+ Klassen-Selektor: Hindernis / Grasp)
     │  scenes/<name>.xml   (Geoms + Klasse; Grasp-Objekt = FREIER Körper)
     ▼
build_env_scene.py ── scene_env_<name>.xml ──► MuJoCo-Sim
     │                                          (Physik: Greifen bewegt Objekt)
     │  Klassen-Info (name → Hindernis/Grasp)         │ mjData: Live-Posen
     └──────────────────────────────►  scene_bridge (NEU, Ansatz B)  ◄──────────┘
                                         hält Identität + Klasse + Live-Pose
             ┌───────────────────────────┼───────────────────────────────┐
             ▼                           ▼                               ▼
   CollisionObject / PlanningScene   OccupancyGrid /map            RViz-Anzeige
   ├ Hindernis → avoid (ACM an)      (2D-Footprint aller boden-     (MarkerArray /
   ├ Grasp → Hand-Kollision OK        berührenden Objekte, Nav)      MotionPlanning)
   └ gegriffen → attached object
             │
   ┌─────────┴───────────────────────────────────────────┐
   ▼ (Echtzeit, Teleop)                                   ▼ (plan-execute, Button)
 Interactive Marker ─► reaktives DLS-Servoing        Arm-Planer (steckbar)
 (pro Tick, um Hindernisse)                          q_start→q_goal, kollisionsfrei
   │                                                       │  Joint-Trajektorie
   └───────────────► Mode-Mux (wie joy_mux) ◄──────────────┘
                          │  Joint-Command-Stream
                          ▼
              arm_controller (250 Hz) ──► Unitree-DDS

   [SPÄTER additiv]  LiDAR (real/sim) ─► self-filtering (bekannte Objekte raus)
                                         └► nur UNBEKANNTE Hindernisse ergänzen
```

**Kernprinzip:** *ein* Weltmodell, *zwei* Bewegungs-Generatoren (reaktiv +
geplant), *eine* Ausführungs-Schnittstelle (Joint-Stream → `arm_controller` →
DDS). Der Mux arbitriert wie der bestehende `joy_mux` der Nav (manueller
Vorrang, gleicher Deadman/Not-Aus).

---

## 5 · Die `scene_bridge` (neuer Node)

**Aufgabe:** Quelle der Wahrheit über Umgebungsobjekte in ROS. Hält je Objekt
`{ Identität, Klasse, Live-Pose, Geometrie }` und publiziert daraus:

1. **RViz:** `visualization_msgs/MarkerArray` auf `/scene_markers`
   (box→CUBE, cylinder→CYLINDER, sphere→SPHERE, mesh→MESH_RESOURCE).
2. **Nav:** `nav_msgs/OccupancyGrid` auf `/map` — Footprints boden­berührender,
   **kollidierbarer** Objekte auf die XY-Ebene projizieren und rastern (ersetzt
   den Dummy in `create_map.py`; die vorhandene `add_obstacle`-Logik ist die
   halbe Miete, nur die Quelle wechselt).
3. **IK/Manipulation:** Kollisionsobjekte — als `CollisionObject`-Messages
   (Interop/Anzeige) **und** eingespeist ins Pinocchio-`collision_model` des
   Solvers (Echtzeit-Mathematik).

**Datenquellen (gestaffelt):**
- **Stufe 1 (statisch):** parst die aktive Szenen-XML (Pfad aus `G1_ENV`
  ableiten wie `config.py`).
- **Stufe 2 (live):** liest Geom-/Body-Posen aus `mjData` der laufenden Sim →
  bewegte/gegriffene Objekte wandern mit.

**Deko vs. Hindernis:** MuJoCo markiert reine Deko mit
`contype="0" conaffinity="0"` (siehe `scene_editor/README.md`). Die Bridge
respektiert das — sonst plant die Nav um Dinge herum, durch die man laufen darf.

**Mesh-Handling:** 2D-Occupancy → Bounding-Box/Convex-Hull-Footprint; IK →
Convex-Hull (der Solver baut ohnehin schon `buildConvexRepresentation`).

---

## 6 · Semantik: Hindernis vs. Grasp-Objekt (ACM)

Konzept = **Allowed Collision Matrix** (MoveIt-Begriff), 1:1 auf euren
Pinocchio-Gate abbildbar (ihr togglet `collisionPairs` bereits):

- **Hindernis:** Paare Arm↔Objekt **aktiv** → IK weicht aus; Nav rastert den
  Footprint.
- **Grasp-Zielobjekt:** Paare **Hand**↔Zielobjekt **deaktiviert** (Kollision
  erlaubt) → die Hand darf ran; für den restlichen Arm bleibt es aktiv.
- **Nach dem Greifen:** Objekt als **attached** an den Hand-Frame umgehängt →
  wandert mit, zählt jetzt gegen den restlichen Arm/Körper und die Welt.
- **Für die Nav (Basis):** ein greifbares Objekt am Boden ist bis zum Greifen
  **trotzdem** ein Hindernis. Die Grasp-Ausnahme gilt für den **Arm**, nicht für
  die **Basis**.

### Voraussetzungen beim Autoren der Szene
1. **Grasp-Objekte müssen freie Körper sein:**
   `<body><freejoint/><geom/><inertial/></body>` statt statischem `<geom>`.
   Nur so greift die MuJoCo-Physik und das Objekt bewegt sich beim Anfassen.
   *(Sie liegen im `<worldbody>`, `build_env_scene.py` reicht sie durch —
   Masse/Inertia müssen aber gesetzt werden.)*
2. **Klassen-Tag muss den Editor überleben.** Da wir den Scene-Editor ohnehin
   umbauen (siehe §7), bekommt jedes Objekt einen erstklassigen
   **Klassen-Selektor** (`Hindernis` / `Grasp-Objekt`), der deterministisch in
   den Export geschrieben wird — keine Krücke über Namenskonventionen nötig.
   Der Tag muss die Kette überleben:
   **Editor → `scenes/*.xml` → `build_env_scene.py` → `scene_bridge`.**

---

## 7 · Scene-Editor-Umbau (Tagging)

Ihr wrappt den Browser-Editor schon (`run_editor.py` ergänzt Upload-/Skalier-
Buttons). In derselben Manier:
- **Objektklasse pro Objekt** (Dropdown `Hindernis` / `Grasp-Objekt`, später
  erweiterbar), geschrieben in den XML-Export.
- Für Grasp-Objekte optional direkt als freien Körper mit Default-Inertial
  exportieren (oder `build_env_scene.py` wandelt getaggte Grasp-Geoms in freie
  Körper um — Umsetzungsdetail).

---

## 8 · Dual-Mode am Arm: reaktiv **und** plan-execute

Zwei Bewegungs-Generatoren, **ein** Weltmodell, **eine** Ausführung.

| Modus | Trigger | Generator | Wann |
|---|---|---|---|
| **Reaktiv** | Interactive Marker | Pinocchio-DLS-Servoing (pro Tick) | Teleop, freies Steuern in Echtzeit |
| **Geplant** | Button / gespeicherte Pose | Arm-Planer (einmal, kollisionsfreie Bahn) | bekannte Zielhaltung sauber anfahren |

**Warum beides:** reaktives Servoing ist gierig/lokal und bleibt an Hindernissen
hängen (euer Divergenz-Schutz in `ik_solver.solve()` zeigt genau das) — schlecht
für „guten Umweg finden". Umgekehrt ist plan-execute für freies Teleop
unpraktisch. Der Mux (Muster wie `joy_mux`) schaltet um: Marker anfassen bricht
einen laufenden Plan ab (manueller Vorrang), gleicher Deadman/Not-Aus.

**Reaktives Ausweichen als Upgrade des Gates:** statt binär anzuhalten, die
DLS-Schrittweite runterskalieren, je näher der Arm einem Hindernis kommt
(Repulsiv-Term im ohnehin genutzten Nullraum). Bleibt voll reaktiv, „gleitet"
statt einzufrieren — für Greifen wichtig. *(Das ist dasselbe Muster wie MoveIt
Servo: Echtzeit-Jogging mit kollisionsbedingter Geschwindigkeitsskalierung,
ohne Planung. Wir übernehmen das Muster, nicht das Paket.)*

---

## 9 · Positionsspeicher (Oberkörper/Arm) — plan-execute

**Priorität jetzt:** Oberkörper/Arm-Positionen. (Basis-Positionsspeicher später,
separat — reitet auf dem bestehenden Dijkstra-Plan-Execute, siehe §10.)

> **Umgesetzt (Stand aktuell):**
> - **Persistent statt flüchtig:** Die Pose-Datei liegt im **bind-gemounteten
>   Repo** (`<repo>/data/arm_poses.json`, in `.gitignore`) und **nicht** mehr
>   unter `$HOME` — `$HOME` liegt im Container-Overlay und war beim nächsten
>   `docker compose up` weg (alle Posen verloren). Ohne Mount (Betrieb direkt
>   auf dem Host) bleibt `~/.g1pilot/arm_poses.json`; Override per
>   `G1_POSE_STORE`. Eine vorhandene alte Datei wird beim ersten Zugriff
>   **automatisch übernommen** (Migration). `arm_controller` loggt den Pfad
>   beim Start.
> - **Kategorien („Ordner"):** Jede Pose hängt in genau einer Kategorie —
>   alles bleibt in **derselben** Datei, nur getrennt geführt. Pose-Namen sind
>   global eindeutig (der Name ist der Schlüssel für
>   `/g1pilot/pose_store/goto`), die Kategorie ist Metadatum. Leere Kategorien
>   werden mitgespeichert, damit ein angelegter „Ordner" auch ohne Pose darin
>   erhalten bleibt. Dateiformat **Version 2**
>   (`{"version","categories","poses"}`); Version 1 (flach) wird transparent
>   gelesen und beim nächsten Schreiben überführt.
> - **Speichern in EINEM Fenster:** `PoseSaveDialog` (`ui_interface.py`) fragt
>   **Name**, **Kategorie** (Dropdown + Button „Kategorie anlegen") und die
>   Komponenten-Häkchen **auf einmal** ab — vorher waren das zwei
>   aufeinanderfolgende Dialoge (erst Name, dann Komponenten). Häkchen sind die
>   **vier Komponenten flach nebeneinander**: **Linker Arm / Rechter Arm /
>   Linke Hand / Rechte Hand** (alle vorausgewählt, jede einzeln abwählbar) —
>   genau die Granularität, die der Store führt. Bewusst **kein** Sammel-
>   Häkchen „Handposition" mit Zusatz-Tickbox „Hände getrennt": das war ein
>   Überrest aus der Version mit *einer* Hand-Komponente und kostete nur
>   Klicks. Nur die gewählten Komponenten kommen in die Pose
>   (`pose_store.py` speichert komponentenweise: `left_arm`, `right_arm`,
>   `left_hand`, `right_hand`; Legacy-Einträge mit `left`/`right` werden
>   transparent gelesen). Die Auswahl geht als JSON
>   `{"name","category","components"}` an `arm_controller._on_pose_save`
>   (`components`: Komponenten-Schlüssel, `"hand"` = beide Hände).
> - **Laden nach Kategorie:** `PoseLoadDialog` zeigt die Posen als Baum
>   **gruppiert nach Kategorie** (inkl. leerer Ordner) und je Pose, *was* drin
>   ist („Rechter Arm, Rechte Hand") — man sieht vor dem Anfahren, ob die Hände
>   mitfahren. Kategorie-Zeilen sind Überschriften, nicht anfahrbar.
> - **Handposition:** Ein Hand-Häkchen speichert die aktuelle 6-DOF-Finger-
>   stellung der jeweiligen Inspire-Hand. Dazu veröffentlicht die inspire-Bridge
>   (`inspire_ftp/bridge.py`) den Ist-Zustand auf `/g1pilot/hand_state/{side}`
>   (arm_controller merkt ihn sich zum Speichern) und nimmt Zielwinkel auf
>   `/g1pilot/hand_goal/{side}` entgegen (zum Wiederherstellen, `set_all_angles`).
>   Läuft die Bridge nicht, wird die Handposition sauber übersprungen — Arme
>   speichern/fahren trotzdem.
> - **Gleichzeitige Ausführung:** Beim Anfahren fahren beide Arme **synchron**,
>   nicht mehr nacheinander. Sind beide Arme in der Pose, plant der Controller
>   sie **gemeinsam als 14-DOF-Problem** (`plan_arms_joint_path`) — dadurch ist
>   **Arm-gegen-Arm an jedem Zwischenzustand** geprüft — und fährt sie über
>   **eine** geteilte Wegpunktliste mit gemeinsamem Index ab. Die Finger fahren
>   beim Bewegungsstart mit los.

### Datenmodell
- Eintrag = `{ name, q_upper }` — **Gelenk-Konfiguration** des Oberkörpers
  (beide Arme, optional Waist je `use_waist`). Eindeutig, keine IK-Mehrdeutigkeit,
  reproduziert die Haltung exakt.
- **Nur den Endpunkt speichern, Bahn jedes Mal neu planen.** Grund: die
  **Startpose ist jedes Mal anders**, und Hindernisse/Grasp-Objekte können sich
  bewegt haben. Eine gespeicherte Bahn wäre brittle.
- Der Planer arbeitet im **Gelenkraum** (`q_start → q_goal`) → abweichende
  Startpose ist kein Problem.

### Aufnahme & Wiedergabe
- **Aufnehmen:** Arm per Marker in die Pose servoen → Button „Speichern" →
  aktuelle Joint-States abgreifen → in die DB. (Triviale Seite; Joint-States
  werden ohnehin gestreamt.)
- **Anfahren:** Button → `q_goal` laden → Planer plant gegen aktuelle
  PlanningScene → Trajektorie über den **gleichen** Joint-Stream ausführen.
- **UI:** Buttons am Streamdeck-GUI (`g1_gui.py`), analog zum `AUTO NAV`-Button.

### Speicherort
- **Lokal-first** (YAML/SQLite): offline-fähig, keine Netzabhängigkeit beim
  Pose-Recall am echten Roboter. **Nicht** in den Echtzeit-Pfad legen.
- Optional später geräteübergreifend (z. B. Supabase) für geteilte Bibliotheken.

### Der Arm-Planer ist eine **steckbare Box** → jetzt **OMPL**
Feste Schnittstelle: rein `(q_start, q_goal, IK-Solver mit synchronisierter
Umgebung)`, raus eine Wegpunktliste. Innenleben austauschbar — und genau diese
Steckbarkeit wurde jetzt genutzt, um das Backend auf **OMPL** umzustellen,
**ohne `arm_controller.py` anzufassen** (Rückgabe-Kontrakt identisch):

- **OMPL (RRTConnect) — jetzt das Standard-Backend.** `arm_planner.py` baut
  einen `RealVectorStateSpace` (7 DOF), setzt den **bestehenden ~0,3-ms-
  Pinocchio-Kollisionscheck als `StateValidityChecker`** (Selbstkollision +
  Umgebungs-ACM, exakt derselbe wie beim reaktiven Servoing) und lässt OMPL
  planen + vereinfachen (`PathSimplifier`). Reines Python-Wheel `ompl`
  (manylinux cp310, x86_64 **und** aarch64/Jetson), **keine** ROS-/MoveIt-/
  ros2_control-Abhängigkeit — passt sauber zur DDS-Steuerung.
- **Eingebauter RRT-Connect — jetzt Fallback.** Fehlt das `ompl`-Wheel im
  Container, fällt `plan_arm_joint_path` transparent auf den handgeschriebenen
  RRT-Connect zurück. Das Feature hängt also nicht an einer optionalen
  Abhängigkeit.
- **Voll-MoveIt2** (move_group/SRDF + `FollowJointTrajectory→DDS`-Adapter)
  wurde **bewusst nicht** gewählt: großer Umbau, der gegen die ros2_control-lose
  Steuerung arbeitet, und würde ohnehin nur den plan-execute-Pfad betreffen.

**Zwei Sicherheits-Feinheiten** (siehe `arm_planner.py`-Moduldoku):
1. OMPLs Motion-Validator-Auflösung ist ein **Bruchteil der Raum-Ausdehnung** →
   an den absoluten `substep` (0,05 rad) gekoppelt (`substep / Ausdehnung`),
   sonst würden dünne Hindernisse übersprungen.
2. Jeder OMPL-Pfad wird zusätzlich mit **exakt derselben Segment-Prüfung** wie
   der Fallback nachvalidiert; schlägt das fehl (Diskretisierungs-Rest), greift
   der Fallback. Die Sicherheit hängt damit **nicht** an OMPLs interner
   Diskretisierung.

---

## 10 · Navigation (später, separat)

Nicht Prio, aber die Bausteine sind schon da:
- Basis-Position anfahren = **bereits plan-execute** (Dijkstra plant um
  Hindernisse, `nav2point` folgt). Ein Basis-Positionsspeicher = gespeicherte
  `/g1pilot/goal`-Posen in der DB, per Button republishen. Fast geschenkt.
- Profitiert automatisch vom echten `/map` aus der `scene_bridge` (§5).

---

## 11 · LiDAR-Zukunft (additive Ebene)

Falls der Livox/MOLA-Pfad in die Sim nachgebaut oder real genutzt wird:
**zwei-Ebenen-Modell**, keine Konkurrenz zum Objekt-Modell.

| Ebene | Quelle | Trägt | Rolle |
|---|---|---|---|
| **Semantik** (bekannt) | `scene_bridge` (mjData) | Identität + Klasse + Live-Pose | Grasp/Ausweichen + Attach |
| **Perzeption** (unbekannt) | LiDAR | nur Punkte | fängt Unmodelliertes (Menschen, Clutter) |

**Fusion per self-filtering:** bekannte Objekt-Punkte aus der Wolke entfernen
(ihr wisst, wo eure Objekte stehen) → Rest = unbekanntes Hindernis, immer
„ausweichen". **LiDAR muss „Hindernis oder Grasp?" nie beantworten** — das
Modell tut es. Damit ist der LiDAR-Plan voll kompatibel.

---

## 12 · Roadmap (Stufen)

1. **Sichtbar + Nav (statisch).** `scene_bridge` Stufe 1: `MarkerArray` +
   `CollisionObject` + 2D-`OccupancyGrid` aus der Szene. Objekte in RViz,
   Dijkstra plant drumherum. `nav.rviz` um Marker-Display ergänzen. Frame-Fix
   in `create_map`. **Sofortiger sichtbarer Nutzen.**
2. **Live + Semantik + reaktives Ausweichen.** `mjData`-Live-Posen; Klassen-Tag
   aus dem umgebauten Editor; Weltobjekte ins Pinocchio-`collision_model`; ACM
   (Hindernis/Grasp) + Attach-beim-Greifen; Gate binär → abgestuft. Alles im
   250-Hz-Tick.
3. **Positionsspeicher am Arm (plan-execute).** Pose-DB (lokal), Aufnahme/Button,
   steckbarer C-Space-Planer, Mode-Mux, Ausführung über `arm_controller`.
4. **Opt-in mächtiger / Real.** MoveIt(-Servo) gegen dieselbe PlanningScene
   und/oder LiDAR-self-filtering additiv. Basis-Positionsspeicher.

---

## 13 · Betroffene Dateien (Impact-Karte, tatsächlicher Stand)

| Datei | Änderung |
|---|---|
| `unitree_mujoco/simulate_python/scene_objects.py` *(neu)* | Parst die kompilierte Szenen-XML (worldbody-Kinder) → Objekt-Specs; STL-AABB-Reader (pure Python) |
| `unitree_mujoco/simulate_python/scene_state_publisher.py` *(neu)* | Sendet Hindernisse (statisch) + Greif-Objekte (live aus `mj_data`) periodisch per UDP an `scene_bridge` |
| `unitree_mujoco/simulate_python/config.py` | `SCENE_UDP_PORT`/`SCENE_UDP_HOST`/`SCENE_PUBLISH_HZ`/`SCENE_ENABLE` |
| `unitree_mujoco/simulate_python/unitree_mujoco.py` | `SceneStatePublisher` in beide Sim-Loops (Realtime + Lockstep) eingehängt |
| `unitree_mujoco/scene_editor/build_env_scene.py` | `grasp_`-Namenskonvention: wrapt passende `<geom>` automatisch in `<body><freejoint/>…</body>` |
| `unitree_mujoco/scene_editor/README.md` | Hindernis-vs-Greif-Objekt-Konvention dokumentiert |
| `g1pilot/g1pilot/navigation/scene_markers.py` *(neu)* | Gemeinsames Wire-Format (`/scene_markers`-Kontrakt): ns/id/text-Codec, Marker↔MuJoCo-Skalierung, AABB/Footprint-Helfer |
| `g1pilot/g1pilot/navigation/scene_bridge.py` *(neu)* | ROS2-Node: UDP-Empfang → `visualization_msgs/MarkerArray` auf `/scene_markers` |
| `g1pilot/g1pilot/navigation/create_map.py` | Dummy → Footprints aus `/scene_markers`; Frame-Default `odom`→`map` |
| `g1pilot/g1pilot/utils/ik_solver.py` | `sync_environment()`, `environment_command_in_collision()` (Punkt-zu-OBB, ACM Hindernis/Grasp), `make_scratch_buffers()` (Thread-Sicherheit) |
| `g1pilot/g1pilot/manipulation/pose_store.py` *(neu, komponentenweise + Kategorien)* | JSON-Datei-Ablage: pro Pose eine Teilmenge aus `left_arm`/`right_arm`/`left_hand`/`right_hand` (Legacy `left`/`right` transparent gelesen) + `category`; **persistenter Default-Pfad** im Repo-Mount (`data/arm_poses.json`, Override `G1_POSE_STORE`, Migration von `~/.g1pilot/`); Format v2 mit `categories`/`poses`, v1 transparent gelesen |
| `g1pilot/g1pilot/manipulation/arm_command.py` *(neu)* | Wire-Format der Live-Pose-Schnittstelle: Parsen/Validieren von `{type,left,right,hands,...}` + Status-Zustaende. ROS-frei, von Controller UND HTTP-Bruecke benutzt (siehe ARM_API.md) |
| `g1pilot/g1pilot/manipulation/arm_api.py` *(neu)* | HTTP-JSON-Bruecke (Default `127.0.0.1:8770`) -> `/g1pilot/arm_command`; wartet optional bis zum Endzustand, liefert Ablehnungsgrund/IK-Restfehler zurueck |
| `g1pilot/g1pilot/utils/ik_solver.py` *(Erweiterung)* | `solve_pose()`: EINMAL auskonvergierte IK auf eigenen Buffern (thread-sicher, ruehrt `set_goal`/`self.data` des Servoings nicht an) -- Basis fuer kartesische Live-Ziele |
| `g1pilot/g1pilot/manipulation/arm_planner.py` *(Backend OMPL, Multi-Arm)* | `plan_arms_joint_path` plant EINEN oder BEIDE Arme (7/14 DOF) via OMPL RRTConnect gegen den bestehenden Pinocchio-Check als `StateValidityChecker`; eingebauter RRT-Connect als Fallback; Shortcut |
| `g1pilot/docker/Dockerfile.sim`, `g1pilot/docker/Dockerfile` | `pip install ompl` (Planer-Backend; reines manylinux-Wheel cp310, x86_64+aarch64, keine ROS-/MoveIt-Abhängigkeit) |
| `g1pilot/g1pilot/manipulation/arm_controller.py` | `/scene_markers`-Abo → `sync_environment`; Kollisions-Gate; Auswahl-Save (JSON inkl. `category`, Hände gemeinsam *oder* seitenweise); **gleichzeitige** 14-DOF-Planung + synchrone Wegpunkt-Zustandsmaschine; `hand_state`-Abo/`hand_goal`-Publish |
| `g1pilot/g1pilot/manipulation/inspire_ftp/bridge.py` | `/g1pilot/hand_state/{side}` (Ist-Fingerwinkel raus) + `/g1pilot/hand_goal/{side}` (Zielwinkel rein → `set_all_angles`) für den Positionsspeicher |
| `g1pilot/g1pilot/teleoperation/ui_interface.py` | Streamdeck-Buttons „POSE SPEICHERN/ANFAHREN/ABBRECHEN"; `PoseSaveDialog` (**ein** Fenster: Name + Kategorie inkl. „Kategorie anlegen" + vier flache Komponenten-Häkchen L/R-Arm, L/R-Hand); `PoseLoadDialog` (Baum **nach Kategorie**, zeigt enthaltene Komponenten) |
| `g1pilot/config/nav.rviz` | `MarkerArray`-Display auf `/scene_markers` |
| `g1pilot/launch/bringup_sim.launch.py` | `scene_bridge` **unconditional** (IK braucht es auch ohne Nav); `create_map` weiter an `G1_ENABLE_NAV` gekoppelt |
| `g1pilot/launch/navigation_launcher.launch.py` | `scene_bridge`-Node ergänzt (Real-Full-Profil) |
| `g1pilot/launch/manipulation_launcher.launch.py` | `environment_collision_gate`/`planned_motion_tolerance` als Launch-Argumente |
| `g1pilot/setup.py` | `scene_bridge`-Entry-Point |
| `g1pilot/docker-compose.yml` | `SIM_SCENE_PORT` in beiden Sim-Containern; `scene_editor/meshes` read-only nach `/scene_meshes` in `g1pilot-sim` (RViz-Mesh-Anzeige) |
| `g1pilot/NAVIGATION.md` | Dummy-Karten-Hinweise auf den echten `/scene_markers`-Stand aktualisiert |

---

## 14 · Getroffene Entscheidungen (waren offen, jetzt umgesetzt)

- **Arm-Planer:** **OMPL (RRTConnect)** als Standard-Backend, eingebauter
  RRT-Connect als Fallback (`arm_planner.py`). Nutzt exakt die bestehenden
  IK-Kollisionschecks als `StateValidityChecker` (kein zweiter
  Sicherheitsbegriff), Rückgabe-Kontrakt unverändert → `arm_controller.py`
  blieb unangetastet. **Voll-MoveIt2 wurde bewusst nicht** gewählt (großer
  Umbau gegen die ros2_control-lose DDS-Steuerung; würde ohnehin nur den
  plan-execute-Pfad betreffen). Reines `ompl`-Wheel, keine ROS-Abhängigkeit.
  *(Ursprünglich als schlanker eigener RRT-Connect gestartet; die feste,
  steckbare Schnittstelle `(q_start, q_goal, ik_solver) → Wegpunkte` machte den
  Backend-Wechsel auf OMPL ohne Controller-Änderung möglich.)*
- **Dual-Arm-Planung:** **gleichzeitig** über einen gemeinsamen 14-DOF-Planer
  (`plan_arms_joint_path` mit `sides=['left','right']`). Damit ist Arm-gegen-Arm
  an JEDEM Zwischenzustand geprüft, und beide Arme fahren synchron über eine
  geteilte Wegpunktliste (gemeinsamer Index) ab. *(Frühere Version: sequenziell
  rechts-dann-links mit dem jeweils anderen Arm als fest — auf Wunsch auf
  gleichzeitige Ausführung umgestellt; mit OMPL ist die 14-DOF-Planung günstig.)*
  Wird nur ein Arm gespeichert/angefahren, plant er als 7-DOF, der andere bleibt
  stehen (feste Konfiguration).
- **Umgebungs-Kollisionsgeometrie:** **kein** hppfcl-`GeometryObject` im
  IK-Solver (unverifizierbare Konstruktor-Signatur je Pinocchio-Version) —
  stattdessen einfache, per Hand nachgerechnete Punkt-zu-orientierter-Box-
  Distanz (Ellbogen + Hand-TCP je Arm). Mit dem echten G1-URDF getestet
  (siehe §15).
- **Klassen-Tag-Kanal:** **Namenskonvention** (`grasp_`-Präfix), nicht ein
  neues XML-Attribut oder eine Sidecar-Datei — übersteht den (nicht selbst
  veränderten) Browser-Editor garantiert, weil er nur den ohnehin editierbaren
  Namen nutzt.
- **Grasp-Freikörper-Erzeugung:** in `build_env_scene.py` (nicht im
  Editor-Export) — der Editor bleibt unangetastet (siehe unten).
- **Wire-Format `/scene_markers`:** ausschließlich `visualization_msgs/
  MarkerArray` (kein `moveit_msgs`, keine neue `.msg`-Schnittstelle/kein neues
  Interface-Package) — RViz UND alle Konsumenten (create_map, ik_solver)
  lesen denselben Standard-Topic. Klasse via `ns`, stabile Identität via
  `id` (Hash des Namens), Name (+ Mesh-AABB) via wiederverwendetes `text`-Feld.
- **Speicher-Backend Positionsspeicher:** lokale JSON-Datei
  (`<repo>/data/arm_poses.json`, `pose_store.py`) — kein Datenbank-Server. Der
  Ort ist bewusst das **bind-gemountete Repo** statt `$HOME`: nur so überlebt
  die Datei den Container (Overlay-FS). **Kategorien in derselben Datei**
  (Feld `category` je Pose + Liste `categories` für leere Ordner) statt eigener
  Dateien/Unterverzeichnisse pro Ordner — ein atomares Rewrite, kein
  Verzeichnis-Scan, und der Pose-Name bleibt global eindeutiger Schlüssel für
  `/g1pilot/pose_store/goto`.
- **DOF-Umfang der gespeicherten Pose:** 7 DOF je Arm (Schulter…Handgelenk),
  **ohne** Taille — konsistent mit der bestehenden Home-/Walk-Pose-Konvention
  in `arm_controller.py`.
- **Editor-Umbau:** **nicht** angefasst (kein Klassen-Dropdown im Browser-
  Editor) — die Namenskonvention macht das unnötig; Objekte werden im Editor
  ganz normal umbenannt (Feld existiert bereits).

---

## 14a · Live-Pose-Schnittstelle (fremde Projekte)

Eigene Dokumente: **[`ARM_API.md`](ARM_API.md)** (Referenz) und
**[`ARM_API_HOWTO.md`](ARM_API_HOWTO.md)** (Anleitung + CLI unter
`examples/arm_api/`). Kurz die Entscheidungen:

- **Kanonischer Eingang ist ein ROS-Topic** (`/g1pilot/arm_command`,
  `std_msgs/String` mit JSON) — **kein eigenes `.msg`-Interface-Paket**, gleiche
  Regel wie beim `/scene_markers`-Wire-Format (§14). Status zurück auf
  `/g1pilot/arm_command/status`.
- **HTTP-JSON-Brücke als EIGENER Node** (`arm_api.py`), nicht im
  `arm_controller`: der regelt mit 250 Hz, dort hat kein Webserver etwas zu
  suchen (dieselbe Trennung wie bei der `scene_bridge`). Der Aufrufer braucht so
  **kein ROS/DDS** — das ist der eigentliche Zweck, denn das fremde Projekt soll
  nicht den halben Container nachbauen müssen.
- **Kein neuer Ausführungspfad:** eingespielte Ziele laufen durch **dieselbe**
  Plan-Execute-Maschinerie und dieselben Gates wie „POSE ANFAHREN"
  (`_start_planned_motion`). Ein fremdes Projekt kann sich nicht an E-Stop,
  `arms_enabled`, Kollisions-Gate oder Gelenklimits vorbeischreiben.
- **Kartesische Ziele brauchten neue IK-Semantik:** `solve()` ist ein
  Servo-Schritt auf geteiltem Zustand; für ein einmalig auskonvergiertes Ziel
  gibt es jetzt `solve_pose()` (eigene `pin.Data`, kein `set_goal`) — läuft im
  Planungs-Thread, ohne das Marker-Servoing zu stören. Nicht erreichbare Ziele
  werden mit **Restfehler** abgelehnt statt näherungsweise angefahren.
- **Nichts wird gespeichert** — bewusst getrennt vom Positionsspeicher (§9).
- **Bind auf `127.0.0.1`** als Default (Token optional): der Container läuft mit
  `network_mode: host`, ein Host-Prozess erreicht die API also lokal, ohne dass
  sie im Netz hängt.

---

## 15 · Implementierungsstand

**Gebaut (Stufen 1–3 der Roadmap in §12), inklusive Positionsspeicher:**
Live-Szenen-Bruecke (MuJoCo → UDP → `scene_bridge` → `/scene_markers`),
`create_map` aus echten Objekten, Umgebungs-Kollision + Hindernis/Grasp-ACM im
IK-Solver, Dual-Mode Arm (reaktives Servoing bleibt unveraendert + neuer
geplanter Positionsspeicher-Modus mit **OMPL RRTConnect**, Fallback eingebauter
RRT-Connect), Streamdeck-Buttons.

**Nachträglich umgesetzt (Backend-Wechsel):** Der Arm-Planer nutzt jetzt
**OMPL (RRTConnect)** statt des eigenen RRT-Connect — über die bereits
vorhandene steckbare Schnittstelle, ohne `arm_controller.py` anzufassen. Der
eigene RRT-Connect bleibt als Fallback erhalten (falls das `ompl`-Wheel fehlt).

**Nachträglich umgesetzt (Positionsspeicher-Erweiterung):** (a) **Auswahl beim
Speichern** — Häkchen für Rechter Arm / Linker Arm / Handposition
(komponentenweiser `pose_store`); (b) **Handposition** speicher-/wiederherstellbar
über neue Bridge-Topics `hand_state`/`hand_goal` (6-DOF je Inspire-Hand); (c)
**gleichzeitige Ausführung** beider Arme via gemeinsamer 14-DOF-Planung statt
sequenziell rechts-dann-links.

**Nachträglich umgesetzt (Speichern/Laden überarbeitet):** (a) **Persistenz** —
Ablage im bind-gemounteten Repo statt `$HOME`, damit Posen das Schließen der
Sim überleben (inkl. automatischer Migration der alten Datei); (b)
**Kategorien** („Ordner") in derselben Datei, mit „Kategorie anlegen"; (c)
**ein** Speichern-Dialog für Name + Kategorie + Komponenten (statt zwei
hintereinander), mit den **vier Komponenten flach** (L/R-Arm, L/R-Hand) statt
Sammel-Häkchen + Trenn-Tickbox; (d) **Laden** zeigt die Posen nach Kategorie
gruppiert samt enthaltener Komponenten.

**Bewusst nicht gebaut** (mit „Zukunft" markiert bzw. vom Nutzer
zurückgestellt): LiDAR-Perzeptionsebene (§11), Basis-Positionsspeicher (§10,
Nav war explizit nicht Prio), **Voll-MoveIt2** (move_group/SRDF +
`FollowJointTrajectory→DDS`-Adapter — bewusst zugunsten des schlanken
OMPL-Backends verworfen).

**Wie geprüft wurde** (dieses Environment hat kein RViz/ROS2-Runtime, aber
MuJoCo und Pinocchio ließen sich per pip installieren — daher konnte ein
Großteil *wirklich ausgeführt* werden, nicht nur gelesen):
- `scene_objects.py` (XML-Parser, STL-AABB, Pose-Komposition inkl. Rotation)
  und `pose_store.py`: mit synthetischen Fixtures **ausgeführt und verifiziert**.
- `build_env_scene.py` + **echte MuJoCo-Physik**: eine `grasp_`-Szene erzeugt,
  mit `mujoco.MjModel` **kompiliert** und geprüft, dass das Greif-Objekt ein
  freier (beweglicher) Körper ist (fällt unter Schwerkraft), das Hindernis
  statisch bleibt, und die Live-Pose-Auslese (`mj_data.xpos/xquat`) korrekt ist.
- `scene_state_publisher.py` + **echter UDP-Transport**: mit kompiliertem
  Modell instanziiert, Snapshot über einen echten Loopback-Socket gesendet und
  empfangen; verifiziert, dass sich die Greif-Objekt-Pose im Payload live
  bewegt (z 0,9 → 0,71 nach 100 Physikschritten), Hindernis-Pose statisch bleibt.
- `scene_markers.py`: Marker↔MuJoCo-Umrechnung, Rotation, Footprint-AABB
  **ausgeführt** (u.a. 45°-Rotation gegen Handrechnung).
- **Vollständige Pipeline-Integration ausgeführt**: reale Szene → Payload →
  Marker → `create_map`-Footprint (Box + Mesh mit korrekten Größen) **und**
  `arm_controller._on_scene_markers` → `ik_solver.sync_environment` → ein
  Marker an der Handposition löst die Umgebungs-Kollision korrekt aus.
- `ik_solver.py`/`arm_planner.py`: **mit echtem G1-URDF und Pinocchio** —
  Umgebungs-ACM, Umwegplanung um ein echtes Hindernis (inkl. unabhängiger
  Segment-Nachprüfung), Mehr-Thread-Stresstest.
- **OMPL-Planer-Backend** (`arm_planner.py`, Umbau): `ompl`-Wheel real
  installiert und die Bindings-API **wirklich ausgeführt** (`SimpleSetup`,
  subklassierter `StateValidityChecker`, `RRTConnect`, `PathSimplifier`).
  Verifiziert: (a) OMPL findet einen kollisionsfreien **Umweg** und die
  unabhängige Segment-Nachprüfung akzeptiert ihn; (b) `direct`/
  `start_in_collision`/`goal_in_collision` liefern dieselben Reason-Codes wie
  zuvor; (c) **Fallback** ohne OMPL liefert einen gültigen Pfad; (d)
  `shortcut_path` kürzt und bleibt gültig. **Mit echtem G1-URDF + Pinocchio**:
  OMPL umplant um ein **echtes** Umgebungs-Hindernis (dünne Box auf der
  Hand-Bahn) herum — `ompl_rrtconnect`, Umweg unabhängig als kollisionsfrei
  bestätigt, direkter Weg nachweislich gesperrt. **Sicherheits-Feinheit
  geprüft:** zu grobe OMPL-Auflösung überspringt dünne Hindernisse → an
  `substep/Ausdehnung` gekoppelt; die zusätzliche Segment-Nachvalidierung fängt
  einen Rest-Diskretisierungsfall ab und löst dann den Fallback aus.
- **Multi-Arm-Planung** (`arm_planner.plan_arms_joint_path`, Umbau): **mit
  echtem G1-URDF + Pinocchio** ein 14-DOF-Umweg um ein echtes Hindernis auf der
  rechten Hand-Bahn — `ompl_rrtconnect`, 14-dimensionale Wegpunkte, unabhängig
  als kollisionsfrei (inkl. Arm-zu-Arm) bestätigt, Endpunkte korrekt; der
  Einzel-Arm-Wrapper liefert weiter 7-DOF.
- **Auswahl-Save + gleichzeitige Ausführung** (`arm_controller.py`,
  komponentenweiser Positionsspeicher): mit gestubbten ROS-/DDS-Abhängigkeiten
  **instanziiert und durchlaufen** — (1) Auswahl-Save speichert genau die
  gewählten Komponenten (nur rechter Arm / beide Arme+Hand / Legacy-Plainname /
  „Hand ohne Bridge-Zustand → übersprungen"); (2) Goto beider Arme erzeugt einen
  **14-DOF-Plan** und aktiviert die **synchrone** Wegpunkt-Zustandsmaschine, die
  beide Arme ans Ziel bringt; (3) Hand-Ziele werden beim Bewegungsstart
  gesendet; (4) reine Handposition fährt ohne Armplanung sofort.
- `arm_controller.py` (Sicherheits-Zustandsmaschine): **8 Szenarien** für den
  Positionsspeicher: normale Bewegung, E-Stop während der Planung, Planung
  fertig während DISABLED + Re-ENABLE, Marker-Griff bricht aktive/laufende
  Planung ab, zwei schnelle Anfahrten (nur die letzte gewinnt), POSE ABBRECHEN.
- **Nicht geprüft** (dieses Environment kann es nicht): RViz-Darstellung, das
  reale 250-Hz-Timing/DDS am Roboter, der Docker-Compose-Build. Vor dem ersten
  Einsatz am/mit dem echten Roboter: in der Sim mit E-Stop griffbereit
  gegenprüfen, insbesondere den Positionsspeicher-Modus.

**In diesem Review gefundene und behobene Bugs:**
1. **Mesh-Pfad-Auflösung** (`scene_objects.py`): Mesh-Dateien wurden relativ
   zum Szenen-Ordner statt zu `meshdir` (`g1/meshes/`) gesucht → alle Mesh-
   Objekte bekamen `aabb_half=None` und damit im Nav-Grid/in der IK-Kollision
   nur eine winzige 0,1³-Default-Box statt ihrer echten Größe (RViz war nicht
   betroffen). Fix: `meshdir`-bewusste Mehrfach-Basis-Auflösung.
2. **Überraschungsbewegung nach E-Stop/Disable** (`arm_controller.py`): Wurde
   eine Planung fertig, während die Arme per E-Stop/DISABLE gesperrt waren,
   aktivierte sie sich beim nächsten ENABLE unerwartet. Ebenso konnte ein
   Marker-Griff *während* laufender Planung deren späteres Ergebnis nicht
   entwerten. Fix: Generations-Zähler (`_plan_gen`) — jeder Abbruch/jede neue
   Anfahrt invalidiert in-flight-Ergebnisse race-frei; zusätzlich Abbruch bei
   E-Stop/ENABLE/DISABLE/HOMING/WALK/Marker/ABBRECHEN.

