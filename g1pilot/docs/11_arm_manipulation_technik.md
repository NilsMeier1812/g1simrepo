# Arm-Manipulation — Technik

Richtet sich an: Entwickler, die an der Armregelung, IK oder dem
Positionsspeicher etwas ändern wollen. Für die Bedienung siehe
[10_arm_manipulation_anleitung.md](10_arm_manipulation_anleitung.md), für die
HTTP-Brücke siehe [21_arm_api_technik.md](21_arm_api_technik.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/manipulation/arm_controller.py` | Hauptregler-Node (250 Hz), IK-Anwendung, Sicherheitsgates, State-Machine |
| `g1pilot/utils/ik_solver.py` | `G1IKSolver`: Pinocchio-basierte IK, Kollisionsprüfung |
| `g1pilot/manipulation/arm_planner.py` | Gelenkraum-Wegplaner (OMPL RRTConnect + Fallback) für den Positionsspeicher |
| `g1pilot/manipulation/pose_store.py` | Dateibasierte Ablage gespeicherter Posen |
| `g1pilot/manipulation/arm_command.py` | Wire-Format der Live-Pose-Schnittstelle (gemeinsam mit `arm_api.py`) |
| `g1pilot/manipulation/interactive_marker.py` | RViz-Marker, Leader-Follower-Verhalten |
| `g1pilot/utils/joints_names.py` | Gelenk-Indizes, -Limits, -Namen (einzige Quelle der Wahrheit) |

## Node: `arm_controller`

Läuft mit fester Taktrate (`rate_hz`, Default 250 Hz) über
`self.timer = self.create_timer(1.0/rate_hz, self.main_loop)`. Schreibt bei
`use_robot=true` direkt auf `rt/arm_sdk` (DDS), sonst auf `/joint_states`
(ROS, für reine RViz-Anzeige ohne Roboter/Sim).

### Zustandsmaschine

Der Node hält mehrere unabhängige, aber interagierende Zustände:

- `arms_enabled` — Manipulation aktiv/inaktiv (`/g1pilot/arms/enabled`).
- `estop_active` — Notaus-Latch (`/g1pilot/emergency_stop`), blockiert
  `ENABLE`, bis `/g1pilot/start` quittiert.
- `_arm_slack` — Arme drehmomentfrei/gedämpft gehalten, ohne die Kontrolle
  an den Onboard-Regler zurückzugeben (siehe unten, „E-Stop").
- `homing_active` / `homing_reached` — Fahrt zur Home-Pose.
- `walk_mode` — während `WALK` (siehe [31_loco_technik.md](31_loco_technik.md))
  hält der Controller die Arme in einer definierten Lauf-Pose, damit die
  Lauf-Policy stabil bleibt.
- `_planned_motion_active` — eine geplante Bewegung (Positionsspeicher /
  Live-Kommando) fährt gerade eine vorab berechnete Wegpunktliste ab.

`_arms_not_ready_reason()` ist die **einzige** Stelle, die prüft, ob jetzt
eine neue Bewegung starten darf (E-Stop, Enable, Homing, Walk, laufende
Planung) — sowohl der Positionsspeicher als auch die Live-Pose-Schnittstelle
(`arm_command`) rufen sie auf, damit die Schranken nicht auseinanderlaufen.

### Reaktives Servoing (Marker-Pfad)

`_right_goal_callback` / `_left_goal_callback` (Subscriber auf
`/g1pilot/hand_goal/{left,right}`, `PoseStamped`):

1. Pose in den IK-World-Frame transformieren (TF, Default `pelvis`).
2. `_apply_offsets_and_filters`: statischer End-Effektor-Offset (Parameter
   `ee_offset_*`), optional automatische Kalibrierung
   (`_gate_auto_calibration`, greift, wenn das eingehende Ziel nahe an der
   aktuellen Handpose liegt — richtet Marker und Ist-Hand einmalig
   aufeinander aus), Tiefpassfilterung (`ik_goal_filter_alpha`),
   Orientierungs-Schrittbegrenzung (`ik_max_ori_step_rad`).
3. `ik_solver.set_goal(side, T_goal_use)` — das eigentliche Lösen passiert im
   `main_loop`.

Ein Marker-Griff bricht **immer** eine laufende geplante Bewegung ab
(`_abort_planned_motion("Marker bewegt")`) — manuelle Eingabe hat Vorrang.

### `main_loop()` — pro Tick

Reihenfolge (vereinfacht):

1. Workspace-Marker publizieren (rein visuell).
2. `_poll_plan_result()` — prüft billig (Lock + Identitätsvergleich), ob ein
   Hintergrund-Planungsthread fertig geworden ist.
3. Ist E-Stop/Slack aktiv und Manipulation nicht aktiv → `_publish_arm_slack()`
   und zurück (siehe unten).
4. Ist Manipulation deaktiviert → ggf. Gewichtsrampe abwärts fahren, sonst
   still sein.
5. Zielkonfiguration `q_target` je nach aktivem Modus bestimmen: `walk_mode`
   (Lauf-Pose) > `homing_active`/`homing_reached` (Home-Pose) >
   `_planned_motion_active` (nächster Wegpunkt der geplanten Bahn) > sonst
   IK-Ergebnis aus dem reaktiven Servoing (`ik_solver.get_joint_targets`).
6. Geschwindigkeitslimit (`arm_velocity_limit`) + exponentielle Glättung
   (`ik_alpha`).
7. Kartesisches Geschwindigkeitslimit (`_limit_cartesian_speed`, überwacht
   Hand-TCPs + Ellbogen).
8. Kollisions-Gate (`_apply_collision_gate`) — hält bei Kollision auf der
   vorherigen Position an.
9. Schreiben: bei `use_robot=true` `rt/arm_sdk` mit
   Schwerkraft-Feedforward-Drehmoment (`_arm_gravity_tau`,
   Parameter `gravity_comp`), sonst `/joint_states`.

### `rt/arm_sdk`-Gewichtsrampe

`motor_cmd[29].q` (der `kNotUsedJoint0`-Slot) trägt das Blend-Gewicht 0..1,
mit dem die Bridge (Sim) bzw. die G1-Firmware (Real) zwischen
Onboard-Steuerung und `arm_sdk`-Kommando mischt. Beim `ENABLE` rampt das
Gewicht über `arm_weight_ramp_up_s` von 0 auf 1, beim `DISABLE` symmetrisch
zurück (`_advance_arm_weight`, `_publish_weight_ramp_down`). Die Sim setzt
beide Rampenzeiten auf 0.0 (sofortiges Umschalten, im Bridge-Merge auf 0/1
getestet); der reale Roboter nutzt 2.0 s (weiche Übergabe, sonst reißt die
Steuerungsübernahme an den Armen).

### E-Stop-Semantik (`_on_emergency_stop` / `_publish_arm_slack`)

Kritisch: Beim E-Stop bleibt das `arm_sdk`-Gewicht auf **1** — würde es auf 0
gesetzt, übernähme der Onboard-Regler die Arme und führe sie *aktiv* mit
voller Geschwindigkeit in seine Default-Pose (ein Sprung). Stattdessen
schreibt `_publish_arm_slack()` weiterhin auf `rt/arm_sdk`, aber mit
`kp=0, tau=0, kd=estop_arm_kd` — die Arme werden drehmomentfrei und sacken
gedämpft, statt aktiv irgendwohin zu fahren. `/g1pilot/start` quittiert nur
den Latch (`estop_active`); die Arme bleiben schlaff, bis
`ENABLE MANIPULATION` die Kontrolle bewusst zurückholt.

### Positionsspeicher (plan-execute)

`_on_pose_save` / `_on_pose_goto` / `_on_pose_cancel` (Topics
`/g1pilot/pose_store/{save,goto,cancel}`).

- **Speichern:** schreibt die zuletzt *kommandierte* Zielkonfiguration
  (`_last_q_target`, nicht die verrauschte Messung) plus optional den
  zuletzt gemeldeten Fingerzustand (`_hand_state`, kommt von der
  Inspire-Bridge) in `PoseStore`. Rückkanal:
  `/g1pilot/pose_store/save/status` (JSON, siehe `arm_command.save_status_message`).
- **Anfahren:** `_start_planned_motion()` — gemeinsamer Einstiegspunkt für
  Positionsspeicher UND die Live-Pose-Schnittstelle. Bricht eine
  eventuell laufende ältere Bewegung sauber ab, erhöht die
  „Planungs-Generation" (`_plan_gen`) und startet `_plan_pose_worker` in
  einem **Hintergrund-Thread** (Planung kann Sekunden dauern, darf den
  250-Hz-Regelkreis nie blockieren).
- `_plan_pose_worker`: löst bei kartesischen Zielen zuerst die IK
  (`ik_solver.solve_pose`), plant dann mit `arm_planner.plan_arms_joint_path`
  einen kollisionsfreien Pfad und glättet ihn (`shortcut_path`). Ergebnis
  geht über `DataBuffer` (threadsicher) zurück.
- `_poll_plan_result()`: aktiviert nur Ergebnisse der **aktuellen**
  Generation — ein Abbruch während der Planung (E-Stop, neuer Marker-Griff,
  DISABLE) entwertet ein später eintreffendes Ergebnis race-frei.
- Ein/beide Arme werden **gemeinsam** als ein 7- bzw. 14-DOF-Problem
  geplant, damit Arm-gegen-Arm-Kollision an jedem Zwischenzustand geprüft
  ist und beide Arme später synchron über **eine geteilte Wegpunktliste**
  abgefahren werden (`_planned_sides`, `_planned_waypoints`,
  `_planned_wp_idx`, gemeinsamer Index im `main_loop`).

### `pose_store.py`

Dateiformat Version 2:

```json
{"version": 2,
 "categories": ["Allgemein", "Greifen"],
 "poses": {"<name>": {"category": "...", "left_arm": [...], "saved_at": 0.0}}}
```

Speicherbare Komponenten: `left_arm`, `right_arm`, `left_hand`, `right_hand`
(`COMPONENTS`-Tupel). Speicherpfad-Priorität: `G1_POSE_STORE`-Env-Variable
> `<repo>/data/arm_poses.json` (bind-gemountet, überlebt Container-Neustarts)
> `~/.g1pilot/` (Host-Betrieb ohne Mount). Schreiben ist atomar
(`write-tmp-then-rename`), Version-1-Dateien (flaches Format ohne
Kategorien) werden transparent migriert.

## `G1IKSolver` (`utils/ik_solver.py`)

Pinocchio-basierter, gedämpfter Pseudo-Inverse-Löser mit:

- adaptiver Dämpfung nahe Singularitäten (`adaptive_damping`,
  `sigma_min_thresh`),
- Nullraum-Regularisierung zu einer Ruhepose (`null_space_gain`,
  `set_rest_posture`) — hält den redundanten 7. Freiheitsgrad (Ellbogen-
  Swivel) natürlich, damit der Arm beim Marker-Ziehen nicht in den zum
  Körper geklappten IK-Zweig fällt,
- Selbstkollisions-Gate (Convex-Hulls, konfigurierbare Marge,
  `arm_command_in_collision`),
- Umgebungs-Kollisions-Gate gegen Objekte aus `/scene_markers`
  (`environment_command_in_collision`, `sync_environment` — siehe
  [51_navigation_technik.md](51_navigation_technik.md), Abschnitt
  Szenen-Brücke).
- `solve_pose(side, T_goal, current_all)` — Einzelaufruf-Variante (kein
  gemeinsamer Solver-Zustand), genutzt vom Planungs-Thread, damit dieser
  nicht mit dem reaktiven 250-Hz-Servoing um denselben mutablen
  Pinocchio-Zustand konkurriert.

## `arm_planner.py`

Plant im 7- oder 14-DOF-Gelenkraum. Primäres Backend: **OMPL RRTConnect**
(falls die Python-Bindings verfügbar sind), Fallback: ein
handgeschriebener RRT-Connect (`_plan_rrt_connect_builtin`) — beide nutzen
exakt dieselbe Kollisionsprüfung wie der reaktive Pfad
(`ik_solver.arm_command_in_collision` /
`environment_command_in_collision`), damit ein gefundener Pfad garantiert
dieselben Sicherheitsschranken erfüllt. Ein von OMPL gelieferter Pfad wird
zusätzlich mit derselben Segmentprüfung wie der Fallback nachvalidiert
(`_path_segments_valid`); schlägt das fehl, springt der Aufruf auf den
Fallback um. `shortcut_path()` glättet das Ergebnis danach zufällig
(Shortcut-Sampling), unabhängig vom verwendeten Backend.

Da der Planungsthread parallel zum reaktiven 250-Hz-Loop auf demselben
`ik_solver`-Objekt arbeitet, holt sich `_make_state_validity_fn` **eigene**
Scratch-Puffer (`ik_solver.make_scratch_buffers()`) statt der Live-Puffer
des Solvers.

## Konfiguration (Auszug, vollständige Liste in `manipulation_launcher.launch.py`)

| Parameter | Bedeutung |
|---|---|
| `rate_hz` | Regeltakt des `main_loop` (Default 250) |
| `ik_alpha`, `ik_goal_filter_alpha` | Glättung Servoing-Pfad |
| `ik_orientation_mode` | `full` / weitere Modi, siehe `set_orientation_mode` |
| `ik_null_space_gain` | Stärke der Nullraum-Regularisierung |
| `planned_motion_tolerance` | Toleranz [rad], ab der ein Wegpunkt als erreicht gilt |
| `arm_api_*` | Parameter der HTTP-Brücke, siehe [21_arm_api_technik.md](21_arm_api_technik.md) |

## Bekannte Einschränkungen

- Der IK-Solve kostet bei aktiv verfolgtem Marker-Ziel ~20-30 ms/Zyklus —
  der Regel-Loop läuft dann effektiv mit ~30-50 Hz statt 250 Hz. Alle
  Limits rechnen mit dem tatsächlichen `dt`, bleiben also korrekt; im
  Halte-Zustand läuft der Loop mit voller Rate.
- Die Taille (12-14) ist über `rt/arm_sdk` real ansteuerbar, wird von
  `arm_controller` aber bewusst nie beschrieben — ein Kommando dort würde
  gegen den Unitree-Loco-Controller arbeiten.
