# Architektur

Richtet sich an: Entwickler, die sich einen Überblick verschaffen wollen,
bevor sie an einer bestimmten Stelle etwas ändern. Für die Details der
einzelnen Subsysteme siehe die jeweiligen `*_technik.md`-Dokumente.

## Gesamtbild

G1Pilot ist ein ROS-2-Package (`g1pilot`), das über die Unitree-DDS-Schnittstelle
(`unitree_sdk2py`) sowohl mit dem echten Unitree G1 als auch mit einer
MuJoCo-Simulation spricht — **derselbe Code, dieselbe Nachrichten-API**, nur
die DDS-Domain/das Interface unterscheiden sich. Der Roboter selbst hat 29
Freiheitsgrade (Beine, Taille, zwei 7-DOF-Arme).

```
.
├── g1pilot/                     ROS-2-Package + Docker/Compose/Makefile (Hauptprojekt)
│   ├── g1pilot/
│   │   ├── state/                Roboterzustand -> /joint_states, /tf, IMU
│   │   ├── manipulation/         IK, Arm-Regelung, Positionsspeicher, Arm-API, Hände
│   │   ├── navigation/           Locomotion (Sim), Loco-Client (Real), autonome Navigation
│   │   ├── teleoperation/        Streamdeck-GUI, Joystick, Joy-Mux
│   │   └── utils/                IK-Solver, Joint-Tabellen, gemeinsame Hilfsfunktionen
│   ├── launch/                   ROS-2-Launchdateien (Sim-/Real-Bringup + Bausteine)
│   ├── docker/, docker-compose.yml   Container-Definitionen
│   ├── policies/g1_wholebody/    RL-Lauf-Policy (ONNX) + deploy.yaml
│   └── docs/                     diese Dokumentation
├── unitree_mujoco/               MuJoCo-Simulation (Unitree-Original, angepasst)
│   └── simulate_python/
│       ├── unitree_mujoco.py        Sim-Loop + Viewer
│       └── unitree_sdk2py_bridge.py DDS <-> MuJoCo, Befehls-Merge
├── unitree_ros2/                 Unitree-ROS-2-Abhängigkeiten
└── unitree_sdk2_python/          Unitree-SDK (Python-Bindings für die DDS-Nachrichten)
```

## Laufzeit-Topologie (Simulation)

Zwei Docker-Container, beide mit `network_mode: host`:

```
┌─ HOST (network_mode: host) ────────────────────────────────────────────┐
│                                                                        │
│  Container: g1_mujoco_sim              Container: g1pilot_sim         │
│  ┌──────────────────────────┐          ┌────────────────────────────┐│
│  │ MuJoCo-Physik (1 kHz)    │          │ ROS 2 Humble               ││
│  │ unitree_sdk2py_bridge     │          │  robot_state · arm_ctrl    ││
│  │   (DDS <-> MuJoCo)        │          │  interactive_marker        ││
│  └────────────┬─────────────┘          │  loco_sim · teleop · rviz  ││
│               │                        └─────────────┬──────────────┘│
│               │     DDS Domain 1 / lo (CycloneDDS)   │                │
│               └──────────────────────────────────────┘                │
└────────────────────────────────────────────────────────────────────────┘
```

Auf dem echten Roboter entfällt der `mujoco-sim`-Container; stattdessen
spricht der `g1pilot`-Container über das physische Netzwerkinterface direkt
mit dem G1-Onboard-Rechner.

## Zwei Kommunikationsebenen

- **Unitree-DDS (CycloneDDS)** — zwischen dem g1pilot-Code und dem Roboter
  (Sim: Bridge; Real: G1-Onboard-Rechner). Identische Nachrichtentypen wie am
  echten Roboter (`unitree_hg`-IDL). Wichtige Topics:

  | Topic | Typ | Richtung | Inhalt |
  |---|---|---|---|
  | `rt/lowstate` | `LowState_` | Roboter → Stack | Gelenk-q/dq/τ, IMU; (Sim) Fußkraft + Basis-v in `reserve[]` |
  | `rt/lowcmd` | `LowCmd_` | Stack → Roboter | Bein-/Taillen-Befehle (`loco_sim`/`loco_client`) |
  | `rt/arm_sdk` | `LowCmd_` | Stack → Roboter | Arm-Befehle (`arm_controller`), gewichtet gemischt mit dem Onboard-Regler |
  | `rt/sportmodestate` | `SportModeState_` | Roboter → Stack | Odometrie (Sim: Ground Truth) |
  | `rt/wirelesscontroller` | `WirelessController_` | Roboter → Stack | Gamepad-Zustand |
  | `rt/inspire/cmd`\|`state` | intern | beidseitig | Inspire-Hand-Finger (nur Sim-Backend `mujoco`) |

  ```
  LowCmd_.motor_cmd[i]:  q, dq, tau, kp, kd  ->  ctrl = tau + kp*(q-q_ist) + kd*(dq-dq_ist)
  LowState_.motor_state[i]: q, dq, tau_est
  LowState_.imu_state: quaternion[w,x,y,z], gyroscope[xyz], accelerometer[xyz]
  ```

- **ROS 2 (rclpy, CycloneDDS als RMW)** — innerhalb des g1pilot-Containers:
  `/joint_states`, `/tf`, Hand-Ziele, RViz, Teleoperation, der ganze
  g1pilot-eigene Node-Graph. Muss auf einer **anderen** DDS-Domain laufen als
  die Unitree-Schnittstelle, sonst kollidieren beide CycloneDDS-Instanzen im
  selben Prozess ("create domain error").

## Sim ↔ Real: Ein Schalter

Der zentrale Unterschied zwischen Simulation und echtem Roboter ist eine
einzige Umgebungsvariable, `G1_SIM_MODE`, ausgewertet in
`g1pilot/utils/common.py`:

```python
def resolve_dds(interface):
    if is_sim_mode():
        return SIM_DDS_DOMAIN, SIM_DDS_INTERFACE   # 1, "lo"
    return REAL_DDS_DOMAIN, interface               # 0, <physisches NIC>
```

| Aspekt | Simulation | Echter Roboter |
|---|---|---|
| `G1_SIM_MODE` | `true` | `false` |
| Unitree-DDS-Domain | `1` | `0` (Roboter-Firmware fest) |
| `ROS_DOMAIN_ID` | `0` | `1` |
| DDS-Interface | `lo` (Loopback) | physisches Ethernet-NIC |
| Locomotion | `loco_sim` (RL-Policy + PD-Balancer) | `loco_client` (Unitree-Onboard-High-Level) |
| Start | `make sim` / `./start.sh` → Simulation | `make real ROBOT_INTERFACE=<nic>` / `./start.sh` → Echter Roboter |
| Compose-Profil | `sim` | `real` / `real-full` |
| Launch-Datei | `bringup_sim.launch.py` | `bringup_real.launch.py` / `bringup_launcher.launch.py` |

Jeder Node, der die DDS-Verbindung öffnet, ruft dafür
`g1pilot.utils.common.init_dds(interface, logger)` auf — das ist die
**einzige** Stelle, an der Sim/Real unterschieden wird. Alle anderen Module
(Arm-IK, Positionsspeicher, Teleoperation, Navigation) sind
modus-unabhängig.

## Joint-Layout (29 DOF)

| Index | Gruppe | Gelenke |
|---|---|---|
| 0–5 | Bein links | hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll |
| 6–11 | Bein rechts | (analog) |
| 12–14 | Taille | waist_yaw, waist_roll, waist_pitch |
| 15–21 | Arm links | shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw |
| 22–28 | Arm rechts | (analog) |

Reine Drehmoment-Aktuatoren, jedes Gelenk drehmomentbegrenzt (z. B. Knöchel
±50 Nm). Die Zuständigkeit für die drei Gruppen ist zwischen den Nodes
strikt aufgeteilt:

- **Beine + Taille (0–14):** `loco_sim` (Sim) bzw. der Unitree-Onboard-Regler
  über `loco_client` (Real), via `rt/lowcmd`.
- **Arme (15–28):** ausschließlich `arm_controller`, via `rt/arm_sdk`.

Auf der Sim-Seite mischt `unitree_sdk2py_bridge.py` (in `unitree_mujoco`)
beide Befehlsquellen pro Motor: Beine/Taille aus `rt/lowcmd`, Arme gewichtet
aus `rt/arm_sdk` (`motor_cmd[29].q` ist das Blend-Gewicht 0..1). Pro
Physik-Schritt wird angewendet: `ctrl = tau + kp*(q-q_ist) + kd*(dq-dq_ist)`.
Auf dem echten Roboter übernimmt dieselbe Logik die G1-Firmware.

## Projektstruktur — Node-Übersicht

| Node (`ros2 run g1pilot ...`) | Datei | Aufgabe |
|---|---|---|
| `robot_state` | `state/robot_state.py` | `rt/lowstate` → `/joint_states`, IMU, TF, Motor-Telemetrie |
| `arm_controller` | `manipulation/arm_controller.py` | Arm-IK, Sicherheitsgates, Positionsspeicher, `rt/arm_sdk` |
| `arm_api` | `manipulation/arm_api.py` | HTTP/JSON-Brücke für externe Projekte |
| `interactive_marker` | `manipulation/interactive_marker.py` | RViz-Marker für Arm-Zielposen |
| `inspire_hand` | `manipulation/inspire_ftp/bridge.py` | Inspire-FTP-Hand-Bridge (GUIs + DDS/Modbus) |
| `loco_sim` | `navigation/loco_sim.py` | Sim-Stellvertreter für Stehen/Laufen (nur Sim) |
| `loco_client` | `navigation/loco_client.py` | Unitree-Onboard-Locomotion ansprechen (nur Real) |
| `dijkstra_planner`, `nav2point`, `create_map` | `navigation/*.py` | autonome Punkt-zu-Punkt-Navigation |
| `scene_bridge` | `navigation/scene_bridge.py` | Umgebungsobjekte (Sim) → `/scene_markers` |
| `sim_localization` | `navigation/sim_localization.py` | Sim-Ersatz für MOLA-Lokalisierung |
| `joystick`, `joy_mux` | `teleoperation/*.py` | Physischer Controller, Auto/Manuell-Mux |
| `ui_interface` | `teleoperation/ui_interface.py` | Streamdeck-GUI (Hauptbedienoberfläche) |

## Weiterführend

- [11_arm_manipulation_technik.md](11_arm_manipulation_technik.md) — IK, Planer, Sicherheitsgates
- [21_arm_api_technik.md](21_arm_api_technik.md) — HTTP-Brücke
- [31_loco_technik.md](31_loco_technik.md) — Stehen/Laufen in Sim und Real
- [41_teleoperation_technik.md](41_teleoperation_technik.md) — Streamdeck/Joystick
- [51_navigation_technik.md](51_navigation_technik.md) — autonome Navigation
- [61_inspire_haende_technik.md](61_inspire_haende_technik.md) — Hand-Bridge
