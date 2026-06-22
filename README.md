# G1Pilot · MuJoCo-Simulation für den Unitree G1

Eine **drag-and-drop Simulationsumgebung** für den Unitree G1 (29 DoF): der komplette
G1Pilot-ROS2-Stack — Arm-IK, Teleoperation, Locomotion, Visualisierung — läuft
**unverändert** gegen MuJoCo statt gegen die echte Hardware. Gesprochen wird über
dieselbe Unitree-DDS-Schnittstelle wie auf dem realen Roboter, sodass derselbe Code
in Sim und Real läuft.

| | |
|---|---|
| **Roboter** | Unitree G1, 29 DoF (Inspire-FTP-Hände) |
| **Simulator** | MuJoCo (Python-Bindings), 1 kHz Physik |
| **Middleware** | Unitree SDK2 (CycloneDDS, Domain 1) + ROS 2 Humble (Domain 0) |
| **Laufzeit** | zwei Docker-Container, `network_mode: host` |

---

## Was kann es?

- 🦾 **Arm-Manipulation** – kartesische Ziele per Interactive-Marker in RViz, gelöst
  über Pinocchio-IK, gesendet als `rt/arm_sdk`.
- 🚶 **Locomotion & Balance** *(`loco_sim`)* – ersetzt in der Sim das Unitree-Onboard-
  High-Level. Eine vortrainierte RL-Policy läuft, ein modellbasierter PD-Regler steht
  und balanciert. Zyklus **Stehen → Laufen → Stehen** über Joystick/Buttons.
- 🧍 **Freie Arme beim Balancieren** – die Arme werden *nicht* zum Balancieren
  gebraucht; sie bleiben für Manipulation frei (z. B. eine leichte Kiste tragen).
- 🕹️ **Teleoperation** – Bildschirm-Joystick + Buttons (Streamdeck-Node) und ein
  CLI-Interface über ROS-Topics.
- 🔁 **Sim ↔ Real ohne Code-Änderung** – Umschalten allein über Env-Variablen.

---

## Schnellstart

> Voraussetzungen: Docker + Docker Compose, ein X11-Display für die GUIs.

```bash
cd g1pilot
make sim          # baut beide Images (bei Bedarf) und startet den Stack
```

`make sim` baut das g1pilot- **und** das MuJoCo-Image (Letzteres enthält die
DDS-Bridge — Bridge-Änderungen werden also hier übernommen) und startet
`docker-compose.sim.yaml`.

| Befehl | Wirkung |
|---|---|
| `make sim` | Stack im Vordergrund starten (Ctrl-C stoppt) |
| `make sim-bg` | Stack im Hintergrund |
| `make stop` | Stack stoppen |
| `make logs` / `make status` | Logs folgen / Container-Status |
| `make shell-sim` / `make shell-mujoco` | Shell im jeweiligen Container |
| `make clean` | Container + Images entfernen |

Beim Start wird einmalig `xhost +local:docker` gesetzt, damit die Container auf das
Display kommen.

---

## Bedienung: der Stehen → Laufen → Stehen-Zyklus

`loco_sim` ist eine kleine FSM. Gesteuert wird sie über die Buttons/den
Bildschirm-Joystick der Teleop-GUI **oder** über ROS-Topics (siehe
[`g1pilot/CHEATS.md`](g1pilot/CHEATS.md)).

| Zustand | Was passiert | Auslöser |
|---|---|---|
| **HOLD** | Standby: steifer Stand, Basis gehalten | Start / `…/start` |
| **BALANCE** | Modellbasierter PD-Stand am Platz, Oberkörper frei | **START BALANCING** / `…/start_balancing` |
| **RUN** | RL-Walking-Policy; Geschwindigkeit per Joystick | **WALK** + `…/loco_cmd_vel` |
| **DAMP** | Not-Aus: alle Motoren weich | **EMERGENCY** / `…/emergency_stop` |

**Typischer Ablauf:**

1. **START BALANCING** → der Roboter steht frei und balanciert; die Arme lassen sich
   parallel über die Marker bewegen.
2. **WALK** → Joystick gibt Vorwärts/Seitwärts/Drehen vor.
3. **Joystick loslassen** → `loco_sim` bremst über die Policy ab und übergibt im
   Doppelstütz sanft zurück an den PD-Stand (Stepping-Stop).

CLI-Kurzform:

```bash
ros2 topic pub --once /g1pilot/start_balancing std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /g1pilot/start_walking   std_msgs/msg/Bool "{data: true}"
ros2 topic pub /g1pilot/loco_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"   # vorwärts
ros2 topic pub --once /g1pilot/loco_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"  # Stop
```

> **Hinweis:** `loco_sim` ist ein **Sim-only-Stellvertreter** für den Onboard-Loco-
> Regler des echten G1. Auf der echten Hardware übernimmt weiterhin Unitrees
> Onboard-High-Level (`loco_client`). Details & Tuning-Parameter in
> [`CHEATS.md`](g1pilot/CHEATS.md).

---

## Architektur

```
┌─ HOST (network_mode: host) ────────────────────────────────────────────┐
│                                                                        │
│  Container: g1_mujoco_sim              Container: g1pilot_sim           │
│  ┌──────────────────────────┐          ┌────────────────────────────┐  │
│  │ MuJoCo Physics (1 kHz)    │          │ ROS 2 Humble               │  │
│  │ unitree_sdk2py_bridge     │          │  robot_state · arm_ctrl    │  │
│  │   (DDS ↔ MuJoCo)          │          │  interactive_marker        │  │
│  └────────────┬─────────────┘          │  loco_sim · teleop · rviz   │  │
│               │                        └─────────────┬──────────────┘  │
│               │     DDS  Domain 1 / lo  (CycloneDDS) │                  │
│               └──────────────────────────────────────┘                  │
└────────────────────────────────────────────────────────────────────────┘
```

Zwei Kommunikationsebenen:

- **DDS (Unitree SDK2), Domain 1 über Loopback `lo`** — zwischen den Containern,
  identisch zum echten Roboter. Bridge → `rt/lowstate` (Gelenke, IMU); Stack →
  `rt/lowcmd` (Beine, von `loco_sim`) und `rt/arm_sdk` (Arme, von `arm_controller`).
- **ROS 2, Domain 0** — innerhalb des g1pilot-Containers (`/joint_states`, `/tf`,
  Hand-Targets, RViz, Teleop).

Die Bridge merged beide Befehls-Quellen pro Motor (Beine/Taille aus `rt/lowcmd`,
Arme gewichtet aus `rt/arm_sdk`) und wendet je Physik-Schritt
`ctrl = τ + kp·(q−q_ist) + kd·(dq−dq_ist)` an.

<details>
<summary>DDS-Topics &amp; Nachrichten-Layout</summary>

| Topic | Typ | Richtung | Inhalt |
|---|---|---|---|
| `rt/lowstate` | `LowState_` | MuJoCo → Stack | Gelenk q/dq/τ, IMU; (Sim) Fußkraft + Basis-v in `reserve[]` |
| `rt/lowcmd` | `LowCmd_` | Stack → MuJoCo | Bein-/Taillen-Befehle (`loco_sim`) |
| `rt/arm_sdk` | `LowCmd_` | Stack → MuJoCo | Arm-Befehle (`arm_controller`) |
| `rt/sportmodestate` | `SportModeState_` | MuJoCo → Stack | Odometrie |
| `rt/wirelesscontroller` | `WirelessController_` | MuJoCo → Stack | Gamepad-State |

```
LowCmd_.motor_cmd[i]:  q, dq, tau, kp, kd      → ctrl = tau + kp·(q−q_ist) + kd·(dq−dq_ist)
LowState_.motor_state[i]: q, dq, tau_est
LowState_.imu_state:   quaternion[w,x,y,z], gyroscope[xyz], accelerometer[xyz]
```
</details>

<details>
<summary>Joint-Layout (29 DoF)</summary>

| Index | Gruppe | Gelenke |
|---|---|---|
| 0–5 | Bein links | hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll |
| 6–11 | Bein rechts | (dito) |
| 12–14 | Taille | waist_yaw, waist_roll, waist_pitch |
| 15–21 | Arm links | shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw |
| 22–28 | Arm rechts | (dito) |

Reine Drehmoment-Aktuatoren, je Gelenk drehmomentbegrenzt (z. B. Knöchel ±50 Nm).
MuJoCo-`qpos` (nq=36): `[0:3]` Pelvis-Position, `[3:7]` Pelvis-Quaternion,
`[7:36]` die 29 Gelenkwinkel.
</details>

---

## Projektstruktur

```
.
├── g1pilot/                     ← ROS 2-Package + Docker/Compose/Makefile (Hauptprojekt)
│   ├── Makefile                 ← make sim | stop | build-* | shell-* | clean
│   ├── docker-compose.sim.yaml  ← Sim-Stack (zwei Container)
│   ├── docker/                  ← Dockerfile.sim · Dockerfile.mujoco · cyclonedds.xml
│   ├── launch/bringup_sim.launch.py   ← startet robot_state, arm, teleop, loco_sim
│   ├── policies/g1/             ← RL-Policy (motion.pt) + g1.yaml (Obs/Gains/Scales)
│   ├── g1pilot/
│   │   ├── state/robot_state.py        ← LowState_ → /joint_states + /tf
│   │   ├── manipulation/arm_controller.py · interactive_marker.py
│   │   ├── navigation/loco_sim.py       ← ★ RL-Walking + PD-Balance + Stepping-Stop
│   │   ├── teleoperation/ui_interface.py ← Buttons + Bildschirm-Joystick
│   │   └── utils/                       ← IK-Solver, Joint-Namen, common
│   ├── CHEATS.md                ← Befehls-Referenz (Topics, Tuning-Parameter)
│   └── TESTING_SIM.md           ← Test-Ablauf
└── unitree_mujoco/              ← MuJoCo-Sim (Unitree, modifiziert)
    └── simulate_python/
        ├── unitree_mujoco.py            ← Sim-Loop + Viewer
        ├── unitree_sdk2py_bridge.py     ← ★ DDS ↔ MuJoCo, Befehls-Merge
        └── config.py                    ← ROBOT, Domain, HOLD_BASE_MODE, dt
```

`unitree_ros2/` und `unitree_sdk2_python/` sind die Unitree-Abhängigkeiten.

---

## Konfiguration

<details>
<summary>Wichtige Schalter</summary>

**`unitree_mujoco/simulate_python/config.py`**

| Parameter | Default | Bedeutung |
|---|---|---|
| `DOMAIN_ID` / `INTERFACE` | `1` / `lo` | DDS-Domain & Interface (muss zum Stack passen) |
| `SIMULATE_DT` | `0.001` | Physik-Schritt (1 kHz); Loco-Regelrate 50 Hz (Decimation 20) |
| `HOLD_BASE_MODE` | `weld` (env) | `weld` = Basis fix (Arm-only); `off` = Basis frei (Loco regelt Beine) |
| `USE_JOYSTICK` | `0` | **muss 0 sein**, wenn kein Gamepad im Container hängt |
| `PUSH_UDP_PORT` | `47900` | Port für den Stoß-Test (`/g1pilot/push`) |

**Env-Variablen (`docker-compose.sim.yaml`)**

| Variable | Wert | Zweck |
|---|---|---|
| `G1_SIM_MODE` | `true` | aktiviert den Sim-Pfad (DDS Domain 1 / `lo`) |
| `ROS_DOMAIN_ID` / `UNITREE_DOMAIN_ID` | `0` / `1` | ROS-Graph bzw. Unitree-DDS getrennt |
| `USE_RVIZ` | `false` | RViz optional dazuschalten |
</details>

---

## Sim ↔ Real

Derselbe Code, anderes Verhalten — gesteuert über `G1_SIM_MODE`:

```python
sim = os.environ.get('G1_SIM_MODE', 'false').lower() == 'true'
domain, iface = (int(os.environ.get('UNITREE_DOMAIN_ID', 1)), 'lo') if sim \
                else (0, os.environ.get('INTERFACE', 'eth0'))
ChannelFactoryInitialize(domain, iface)
```

| Aspekt | Simulation | Echter Roboter |
|---|---|---|
| Start | `make sim` | `make real ROBOT_INTERFACE=<eth>` |
| Compose | `docker-compose.sim.yaml` | `docker-compose.real.yaml` |
| Launch | `bringup_sim.launch.py` | `bringup_launcher.launch.py` |
| DDS-Interface | `lo` | Ethernet zum Roboter |
| Locomotion | `loco_sim` (Sim-Stellvertreter) | Unitree-Onboard (`loco_client`, `use_robot=true`) |
| LiDAR / SLAM | aus | Livox + MOLA an |

> ⚠️ **Vor jedem Real-Start:** E-Stop erreichbar, ≥ 2 m Freifläche, erst in Sim
> validieren, Arm-Befehle klein anfangen, Drehmoment-Limits beachten.

---

## Bekannte Einschränkungen

- **`loco_sim` ist sim-only.** Es bildet das Verhalten des Onboard-Reglers nach (u. a.
  mithilfe sim-interner Größen wie Basis-Geschwindigkeit/Fußkontakt) und ist **nicht**
  für den Realeinsatz gedacht — dort läuft Unitrees Onboard-Loco.
- **Walking-Policy** ist die kanonische `unitree_rl_gym`-G1-Policy (velocity-getaktet,
  ohne explizites Stand-Verhalten); der driftfreie Stand kommt vom modellbasierten PD.
- **DX3-Hand-Controller** crasht beim Start (keine DX3 angeschlossen) — ignorierbar.
- **`USE_JOYSTICK=0`** zwingend ohne Gamepad, sonst stirbt der Sim-Thread.

---

## Weiterführende Doku

- [`g1pilot/CHEATS.md`](g1pilot/CHEATS.md) — alle Steuer-Topics + Live-Tuning-Parameter
- [`g1pilot/TESTING_SIM.md`](g1pilot/TESTING_SIM.md) — Test-Ablauf
