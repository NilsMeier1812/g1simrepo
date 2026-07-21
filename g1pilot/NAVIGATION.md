# NAVIGATION.md — Autonome Punkt-zu-Punkt-Navigation (g1pilot-Ansatz)

Wie du den (aus dem g1pilot-Original übernommenen) Navigations-Stack **startest,
initialisierst, bedienst** — und **worauf du achten musst**. Am Ende: eine
Zusammenfassung, **wie der komplette Nav-Stack aussieht und funktioniert**.

> Kurzfassung des Prinzips: Du gibst ein **Ziel** (Punkt in der Karte). Ein
> **globaler Planer** (Dijkstra) rechnet einen Pfad, ein **Follower** (nav2point)
> fährt ihn ab, indem er einen **virtuellen Joystick** erzeugt — denselben, den
> sonst du bewegst. Dadurch nutzt die Autonomie exakt den bestehenden Loco-Pfad.

---

## 1 · Was läuft wo (Sim vs. Real)

Es ist **derselbe Nav-Stack** in Sim und Real. Nur zwei Bausteine unterscheiden
sich (weil MuJoCo kein LiDAR hat und `loco_sim` keinen Joystick liest):

| Baustein | **Sim (MuJoCo)** | **Real (echter G1)** |
|---|---|---|
| Lokalisierung (Pose) | `sim_localization` (aus `rt/sportmodestate`) | **MOLA** (Livox-SLAM) + `mola_fixed` |
| Planer | `dijkstra_planner` | `dijkstra_planner` *(gleich)* |
| Follower | `nav2point` | `nav2point` *(gleich)* |
| Joy-Mux | `joy_mux` | `joy_mux` *(gleich)* |
| Loco-Anbindung | `joy_to_cmdvel` → `loco_sim` | Joy direkt → `loco_client` (Unitree-Onboard) |
| Karte | `create_map` aus `/scene_markers` (echte Umgebung, siehe `scene_bridge`) | `create_map` (Dummy/leer — kein MuJoCo/G1_ENV auf real, s.u.) |

Alles andere (Ziel-Topic, Pfad, auto_enable, Achsen) ist **identisch** — was du
in der Sim übst, gilt 1:1 real.

---

## 2 · Starten & Initialisieren — **Sim**

**1) Stack mit Navigation starten**

```bash
cd g1pilot && ./start.sh
#  2c) Navigation: JA   ← neuer Menüpunkt, startet den Nav-Stack
```

Nicht-interaktiv: `G1_ENABLE_NAV=1 ./start.sh --yes`.

> **Es gibt KEIN eigenes Nav-Fenster.** Die Navigation lebt in **RViz** (Karte,
> Pfad, Ziel-Werkzeug). Bei `G1_ENABLE_NAV=1` wird **RViz automatisch gestartet**
> mit der Nav-Ansicht `nav.rviz` (Fixed Frame `map`). Neben dem MuJoCo-Fenster
> geht also ein RViz-Fenster auf, in dem eine (anfangs leere) Karte liegt — das
> ist die Nav-Oberflaeche.

**2) Roboter ins Laufen-bereit bringen** *(Nav bewegt ihn nur, wenn er laufen darf)*

```bash
ros2 topic pub --once /g1pilot/start_balancing std_msgs/msg/Bool "{data: true}"  # -> STAND
ros2 topic pub --once /g1pilot/start_walking   std_msgs/msg/Bool "{data: true}"  # -> WALK
```

(Oder am Streamdeck: **START BALANCING**, dann **WALK**.) Erst jetzt reagiert der
Roboter auf Geschwindigkeits-Kommandos. Im reinen STAND bleibt er absichtlich
stehen.

---

## 3 · Bedienen (Sim **und** Real gleich)

**A) Ziel setzen** — Punkt in der Karte (`map`-Frame):

- **RViz:** oben in der Toolbar **„2D Goal Pose"** anklicken, dann in die Karte
  klicken-ziehen. In `nav.rviz` ist das Werkzeug bereits auf `/g1pilot/goal`
  konfiguriert und der Fixed Frame ist `map` — es funktioniert also direkt, ohne
  Einstellungen. In der Sim zeigt die Karte jetzt die Objekte der geladenen
  Umgebung (siehe `SCENE_BRIDGE.md`); ohne eigene `G1_ENV`-Umgebung (oder auf
  real) ist sie leer — geklickt werden kann trotzdem ueberall.
  > **Die Zieh-Richtung = die End-Ausrichtung.** Der Roboter faehrt zum Punkt und
  > dreht sich dort **auf der Stelle** auf genau diesen Yaw (kuerzester Weg). Nur
  > klicken (nicht ziehen) = Yaw 0. Tuning in `nav2point` (Live via `--ros-args -p`):
  > `yaw_tol_deg` (Toleranz, Default 8°), `align_yaw_kp` (Dreh-Tempo),
  > `yaw_hold_dist` (ab hier gerade reingleiten statt kreiseln, Default 0.5 m),
  > `final_align:=false` schaltet die Endausrichtung ganz ab.
- **CLI (immer):**
  ```bash
  ros2 topic pub --once /g1pilot/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}}}"
  ```

Der Planer publiziert daraufhin `/g1pilot/path` (in RViz als Linie sichtbar),
`nav2point` markiert Ziel (grün) und aktuellen Wegpunkt (orange).

**B) Autonomie scharfschalten** — erst jetzt fährt der Roboter los:

- **Streamdeck-GUI (empfohlen):** Button **AUTO NAV** klicken (er wird grün).
  Der Button erscheint automatisch, sobald der Nav-Stack läuft (Sim:
  `G1_ENABLE_NAV`, Real: `G1_ENABLE_LIDAR`).
- **CLI (Alternative):**
  ```bash
  ros2 topic pub --once /g1pilot/auto_enable std_msgs/msg/Bool "{data: true}"
  ```
- **Real zusätzlich:** der PS4-Auto-Button (`joystick.py` toggelt `auto_enable`).

Der Roboter folgt dem Pfad und **stoppt selbstständig am Ziel** (`goal_tolerance`).

**C) Autonomie abschalten / anhalten:**

**AUTO NAV** erneut klicken (Button wird grau) — oder CLI mit `{data: false}`.
Der Roboter stoppt dann sofort (`joy_mux` sendet beim Abschalten einen
Stop-Befehl). Sanftes Not-Anhalten wie immer: **START BALANCING**. Harter
Not-Aus: **EMERGENCY**.

---

## 4 · Starten & Bedienen — **Real** (kurz)

Voraussetzung: **Livox MID360 + großes Image** (der schlanke Real-Modus hat kein
MOLA). Nav zuschalten über `G1_ENABLE_LIDAR=1`:

```bash
G1_ENABLE_LIDAR=1 ./start.sh     # Real-Zweig; startet Livox + MOLA + Nav-Nodes
```

Danach **wie in Sim**: `START` → `START BALANCING` → Ziel setzen → `auto_enable`.
Der Unterschied ist nur die Pose-Quelle (MOLA) und dass `loco_client` den Joy
direkt fährt. **Erst gründlich in der Sim geübt haben** (siehe unten).

---

## 5 · Worauf du achten musst ⚠️

- **Hindernisvermeidung nur fuer die geladene Umgebung, nicht spontan.** In der
  Sim rastert `create_map` jetzt die Objekte der geladenen `G1_ENV`-Umgebung
  (siehe `SCENE_BRIDGE.md`) in die Karte — der Planer weicht ihnen also aus.
  **Spontane** Hindernisse (Menschen, verschobene Objekte ohne eigenes
  `G1_ENV`-Rebuild) sieht er **nicht** — es gibt keine Live-Perzeption (LiDAR/
  Kamera). Auf **real** ist die Karte weiterhin ein **leerer Dummy** (kein
  MuJoCo/G1_ENV). Nur in **freier, kontrollierter** Fläche einsetzen.
- **Roboter muss laufbereit sein.** In der Sim erst `START BALANCING` **und**
  `START WALKING`; real erst `START BALANCING`. Sonst passiert auf ein Ziel hin
  nichts (oder er steht nur).
- **Auto ODER manuell, nicht beides.** Bei aktivem `auto_enable` hat der Nav-Joy
  Vorrang. Fass den UI-/PS4-Joystick nicht gleichzeitig an — sonst kämpfen zwei
  Geschwindigkeits-Quellen um `loco_cmd_vel`.
- **Deadman gilt weiter.** Fällt der Nav-Stream aus, greift derselbe
  `cmd_vel_timeout`/Deadman wie beim manuellen Gehen — der Roboter stoppt.
- **Konservative Limits.** Geh-Limits (`vx/vy/vyaw`, real 0.4/0.3/0.4) gelten auch
  für die Autonomie. Für die ersten Versuche so lassen.
- **Freifläche + E-Stop erreichbar**, genau wie bei manuellem Gehen
  (`REAL_TESTING.md` / `PREFLIGHT.md`).

---

## 6 · Der komplette Nav-Stack — Aufbau & Funktion

**Datenfluss** (Sim-Variante; real ist identisch, nur die *kursiven* Kästen
tauschen):

```
   Ziel  /g1pilot/goal (PoseStamped, map)
     │
     ▼
 ┌───────────────────┐      /map (OccupancyGrid)        ┌──────────────────────┐
 │ dijkstra_planner  │◄──── create_map (Dummy) ─────────│  Pose                │
 │ Grid-Dijkstra +   │                                  │  /lidar_odometry/    │
 │ Inflation + LOS-  │◄─────────────────────────────────│  pose_fixed          │
 │ Shortcut + Glättung│                                 │                      │
 └─────────┬─────────┘                                  │ *sim_localization*   │
           │ /g1pilot/path                              │  (rt/sportmodestate) │
           ▼                                            │ real: *MOLA + fix*   │
 ┌───────────────────┐                                  └──────────┬───────────┘
 │ nav2point         │◄────────────── Pose ────────────────────────┘
 │ Pure-Pursuit-     │
 │ Waypoint-Follower │────► /g1pilot/auto_joy  (virtueller Joystick, Deadman-Btn 8)
 └───────────────────┘                    │
                                          ▼
                         ┌────────────────────────────┐   /g1pilot/auto_enable
                         │ joy_mux (Auto/Manuell-Mux)  │◄── (scharfschalten)
                         └──────────────┬──────────────┘
                                        │ /g1pilot/joy
                       ┌────────────────┴─────────────────┐
                       ▼ (SIM)                             ▼ (REAL)
             ┌───────────────────┐                ┌──────────────────┐
             │ joy_to_cmdvel     │                │ loco_client      │
             │ Joy→loco_cmd_vel  │                │ joystick_callback│
             └─────────┬─────────┘                │ → Move()         │
                       ▼                          └────────┬─────────┘
             ┌───────────────────┐                         ▼
             │ loco_sim (Policy) │                Unitree-Onboard-Loco
             └───────────────────┘                (Schreiten/Balance)
```

**Node für Node:**

1. **Lokalisierung** — liefert die Roboter-Pose auf `/lidar_odometry/pose_fixed`
   (Frame `map`). Real: **MOLA** (Livox-SLAM) + `mola_fixed` (Orientierungs-Fix).
   Sim: **`sim_localization`** liest die MuJoCo-Ground-Truth (`rt/sportmodestate`
   für x/y + IMU für Yaw) und tritt an dieselbe Stelle.
2. **`create_map`** — publiziert die Belegungskarte `/map`. In der Sim gerastert
   aus `/scene_markers` (den Objekten der geladenen `G1_ENV`-Umgebung, siehe
   `scene_bridge` und `SCENE_BRIDGE.md`); auf real (kein MuJoCo/G1_ENV) bleibt
   sie ein leerer Dummy.
3. **`dijkstra_planner`** — nimmt Karte + Pose + Ziel und rechnet einen kürzesten
   Pfad auf dem Raster: Hindernis-**Inflation** (Sicherheitsabstand),
   **Line-of-Sight-Shortcut**, **Catmull-Rom-Glättung**, **Turn-Cost** (bevorzugt
   gerade Wege). Ausgabe: `/g1pilot/path`. Ohne gültige Karte → gerade Linie.
4. **`nav2point`** — **Pure-Pursuit-Follower**: nimmt Pose + Pfad, zielt auf den
   nächsten Wegpunkt und erzeugt daraus eine **Joy-Nachricht** (`/g1pilot/auto_joy`)
   mit gedrücktem **Deadman-Button 8** und Geschwindigkeits-Achsen — als ob du den
   Joystick hieltest. Stoppt am Ziel (`goal_tolerance`).
5. **`joy_mux`** — mischt manuellen und Auto-Joystick. Nur wenn `auto_enable=true`
   ist, gibt er den Nav-Joy an `/g1pilot/joy` weiter (manuelle Eingaben haben ein
   kurzes Vorrangfenster). Das ist die **Scharfschalt-Schranke** der Autonomie.
6. **Loco-Anbindung** — Real: `loco_client` liest `/g1pilot/joy` direkt und ruft
   `Move()` am Unitree-Onboard-Loco. Sim: `joy_to_cmdvel` übersetzt den Joy in
   `/g1pilot/loco_cmd_vel`, das `loco_sim` (die Policy) fährt.

**Warum dieser Umweg über einen „Fake-Joystick"?** Weil die Autonomie so **exakt
denselben, erprobten Pfad** nimmt wie deine manuelle Teleoperation — inklusive
Deadman, Geschwindigkeits-Limits und Not-Aus. Es gibt keinen zweiten,
ungetesteten Steuerweg zum Roboter.

---

## 7 · Troubleshooting

| Symptom | Ursache / Fix |
|---|---|
| Ziel gesetzt, aber kein Pfad | Keine Pose? `ros2 topic echo /lidar_odometry/pose_fixed` (Sim: läuft `sim_localization`? Real: MOLA/Livox an?). |
| Pfad da, Roboter fährt nicht | `auto_enable` gesendet? Sim: schon in **WALK** (START BALANCING → START WALKING)? Real: **balanciert** (START BALANCING)? |
| Roboter ruckelt / zwei Tempo-Quellen | UI-/PS4-Joystick losgelassen? Bei Auto nicht gleichzeitig manuell fahren. |
| Fährt am Ziel vorbei / dreht komisch | `nav2point`-Limits/`pos_kp`/`yaw_kp` bzw. `waypoint_tolerance` prüfen. |
| RViz zeigt Roboter im Ursprung fest | Nur mit Nav bewegt sich das TF-Modell (`sim_localization` liefert die dyn. TF). Fixed Frame in RViz = `map`. |
| „läuft in Hindernis" | Sim: läuft `scene_bridge`? Objekt in `G1_ENV`? Ohne eigene Umgebung/auf real ist die Karte weiterhin leer (keine Live-Perzeption) — Fläche frei halten. |

Weiterführend: `REAL_TESTING.md` (Hardware-Runbook), `PREFLIGHT.md`
(Abnahme-Checkliste), `CHEATS.md` (Topic-Referenz).
