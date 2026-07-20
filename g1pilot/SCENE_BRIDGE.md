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

### Der Arm-Planer ist eine **steckbare Box**
Feste Schnittstelle: rein `(q_start, q_goal, PlanningScene)`, raus eine
Joint-Trajektorie. Innenleben austauschbar:

- **Schlanker eigener C-Space-Planer** (RRT-Connect + Shortcutting +
  Zeit-Parametrisierung). Ihr habt **3 der 4 Teile schon**: Pinocchio-Modell,
  ~0,3 ms Kollisionscheck (als State-Validity-Checker), Joint-Streaming-Executor.
  Fehlt nur RRT-Connect obendrauf. Konsistent zu eurer Wahl „eigener Dijkstra
  statt Nav2" bei der Basis. **Leichte Empfehlung für den Start.**
- **MoveIt/OMPL** — Standard, Ökosystem, RViz-MotionPlanning. Kosten:
  MoveIt-Config (SRDF/Kinematik — Inputs großteils vorhanden) **plus**
  Execution-Brücke `FollowJointTrajectory → arm_controller → DDS`.

Weil die Box steckbar ist, ist die Wahl **aufschiebbar**: mit dem schlanken
Planer starten, später MoveIt einsetzen — ohne Weltmodell, Speicher oder Mux
anzufassen.

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
| `g1pilot/g1pilot/manipulation/pose_store.py` *(neu)* | JSON-Datei-Ablage gespeicherter Armposen |
| `g1pilot/g1pilot/manipulation/arm_planner.py` *(neu)* | RRT-Connect (7-DOF, ein Arm) + Shortcut-Glättung gegen dieselben IK-Kollisionschecks |
| `g1pilot/g1pilot/manipulation/arm_controller.py` | `/scene_markers`-Abo (TF map→pelvis) → `sync_environment`; kombiniertes Kollisions-Gate; Pose-Store-Topics; geplante-Bewegung-Zustandsmaschine |
| `g1pilot/g1pilot/teleoperation/ui_interface.py` | Streamdeck-Buttons „POSE SPEICHERN/ANFAHREN/ABBRECHEN" |
| `g1pilot/config/nav.rviz` | `MarkerArray`-Display auf `/scene_markers` |
| `g1pilot/launch/bringup_sim.launch.py` | `scene_bridge` **unconditional** (IK braucht es auch ohne Nav); `create_map` weiter an `G1_ENABLE_NAV` gekoppelt |
| `g1pilot/launch/navigation_launcher.launch.py` | `scene_bridge`-Node ergänzt (Real-Full-Profil) |
| `g1pilot/launch/manipulation_launcher.launch.py` | `environment_collision_gate`/`planned_motion_tolerance` als Launch-Argumente |
| `g1pilot/setup.py` | `scene_bridge`-Entry-Point |
| `g1pilot/docker-compose.yml` | `SIM_SCENE_PORT` in beiden Sim-Containern; `scene_editor/meshes` read-only nach `/scene_meshes` in `g1pilot-sim` (RViz-Mesh-Anzeige) |
| `g1pilot/NAVIGATION.md` | Dummy-Karten-Hinweise auf den echten `/scene_markers`-Stand aktualisiert |

---

## 14 · Getroffene Entscheidungen (waren offen, jetzt umgesetzt)

- **Arm-Planer:** schlanker eigener RRT-Connect (`arm_planner.py`), **nicht**
  MoveIt/OMPL — konsistent zum eigenen Dijkstra bei der Basis, nutzt exakt die
  bestehenden IK-Kollisionschecks (kein zweiter Sicherheitsbegriff). Bleibt
  austauschbar (feste Schnittstelle `(q_start, q_goal, ik_solver) → Wegpunkte`).
- **Dual-Arm-Planung:** **sequenziell** (erst rechts, dann links) statt
  gemeinsamem 14-DOF-Planer — der jeweils andere Arm gilt waehrend der Planung
  als fest; das Selbstkollisions-Gate deckt Arm-zu-Arm trotzdem ab. Ausfuehrung
  ist ebenfalls sequenziell (Konsistenz mit der Planungs-Annahme).
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
  (`~/.g1pilot/arm_poses.json`, `pose_store.py`) — kein Datenbank-Server.
- **DOF-Umfang der gespeicherten Pose:** 7 DOF je Arm (Schulter…Handgelenk),
  **ohne** Taille — konsistent mit der bestehenden Home-/Walk-Pose-Konvention
  in `arm_controller.py`.
- **Editor-Umbau:** **nicht** angefasst (kein Klassen-Dropdown im Browser-
  Editor) — die Namenskonvention macht das unnötig; Objekte werden im Editor
  ganz normal umbenannt (Feld existiert bereits).

---

## 15 · Implementierungsstand

**Gebaut (Stufen 1–3 der Roadmap in §12), inklusive Positionsspeicher:**
Live-Szenen-Bruecke (MuJoCo → UDP → `scene_bridge` → `/scene_markers`),
`create_map` aus echten Objekten, Umgebungs-Kollision + Hindernis/Grasp-ACM im
IK-Solver, Dual-Mode Arm (reaktives Servoing bleibt unveraendert + neuer
geplanter Positionsspeicher-Modus mit RRT-Connect), Streamdeck-Buttons.

**Bewusst nicht gebaut** (mit „Zukunft" markiert bzw. vom Nutzer
zurückgestellt): LiDAR-Perzeptionsebene (§11), Basis-Positionsspeicher (§10,
Nav war explizit nicht Prio), MoveIt/OMPL-Migration.

**Wie geprüft wurde** (dieses Environment hat kein MuJoCo-Display/ROS2-Runtime
— ehrliche Einordnung, keine Behauptung von End-to-End-Tests im echten Stack):
- `scene_objects.py` (XML-Parser, STL-AABB, Pose-Komposition inkl. Rotation)
  und `pose_store.py`: mit synthetischen Fixtures **ausgeführt und verifiziert**
  (reines Python, keine Fremdabhängigkeit).
- `build_env_scene.py`: die `grasp_`→Freikörper-Umwandlung **tatsächlich
  ausgeführt** (echtes Skript, echte G1-Robotermodelle) und die XML-Ausgabe
  geprüft.
- `scene_markers.py`: Marker↔MuJoCo-Umrechnung, Rotation, Footprint-AABB
  **mit gestubbten ROS-Messages ausgeführt und verifiziert** (u.a. 45°-Rotation
  gegen Handrechnung geprüft).
- `ik_solver.py`/`arm_planner.py`: **mit dem echten G1-URDF und echtem
  Pinocchio geladen und ausgeführt** (pip-Pakete `pin`/`coal` lassen sich hier
  installieren) — Umgebungs-ACM (Hindernis blockt, Grasp an der Hand erlaubt,
  an Ellbogen weiter blockt), RRT-Connect-Umwegplanung um ein echtes Hindernis
  (inkl. unabhängiger Nachprüfung jedes Wegpunkt-Segments) und ein gezielter
  Mehr-Thread-Stresstest (Planungs-Thread gleichzeitig mit simuliertem
  250-Hz-Regelkreis) liefen **ohne Fehlschlag**.
- `arm_controller.py`: mit umfangreich gestubbten ROS-/DDS-Abhängigkeiten
  **instanziiert und durchlaufen** — Pose speichern, Planung anstoßen, die
  komplette Wegpunkt-Zustandsmaschine bis zum Abschluss (realistische
  Taktrate simuliert) liefen fehlerfrei.
- **Nicht geprüft** (dieses Environment kann es nicht): echte MuJoCo-Physik
  (Greifen/Freikörper-Verhalten), echter UDP-Transport zwischen zwei
  Containern, RViz-Darstellung, das reale 250-Hz-Timing/DDS, Docker-Compose-
  Build. Vor dem ersten Einsatz am/mit dem echten Roboter: in der Sim mit
  E-Stop griffbereit gegenprüfen, insbesondere den Positionsspeicher-Modus.

