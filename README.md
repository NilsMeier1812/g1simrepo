# G1Pilot MuJoCo Simulation — Projekt-Dokumentation

## Überblick

Dieses Projekt ersetzt die echte Unitree G1 Hardware durch einen MuJoCo-Simulator. Der gesamte G1Pilot ROS2-Stack (Arm-IK, Teleoperation, Visualisierung) läuft unverändert — statt mit dem echten Roboter kommuniziert er über DDS mit MuJoCo.

**Roboter:** Unitree G1 (29 DoF) mit Inspire FTP Händen (kein DX3)
**ROS2:** Humble
**Simulator:** MuJoCo (Python bindings)
**Alles läuft in Docker-Containern.**

---

## Architektur

```
┌─────────────────────────────────────────────────────────────────────────┐
│  HOST (faps-pc)                                                         │
│                                                                         │
│  ┌──────────────────────────────┐   ┌────────────────────────────────┐  │
│  │  Container: g1_mujoco_sim    │   │  Container: g1pilot_sim        │  │
│  │  Image: g1pilot-mujoco:v1.0  │   │  Image: g1pilot-sim:v1.1.0    │  │
│  │                              │   │                                │  │
│  │  ┌────────────────────────┐  │   │  ┌──────────────────────────┐  │  │
│  │  │  MuJoCo Physics Engine │  │   │  │  ROS2 Humble             │  │  │
│  │  │  (mj_step @ 200 Hz)   │  │   │  │                          │  │  │
│  │  └───────────┬────────────┘  │   │  │  robot_state.py          │  │  │
│  │              │               │   │  │  arm_controller.py       │  │  │
│  │  ┌───────────┴────────────┐  │   │  │  interactive_marker.py   │  │  │
│  │  │  unitree_sdk2py_bridge │  │   │  │  loco_client.py          │  │  │
│  │  │  (DDS ↔ MuJoCo)       │  │   │  │  joystick.py             │  │  │
│  │  └───────────┬────────────┘  │   │  │  rviz2                   │  │  │
│  │              │               │   │  └──────────┬───────────────┘  │  │
│  └──────────────┼───────────────┘   └─────────────┼──────────────────┘  │
│                 │                                  │                     │
│                 │     DDS über Loopback (lo)       │                     │
│                 │     Domain 1, CycloneDDS         │                     │
│                 └──────────────────────────────────┘                     │
│                                                                         │
│  Beide Container: network_mode: host                                    │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Kommunikation im Detail

Es gibt **zwei getrennte Kommunikationsebenen**:

### 1. DDS (Unitree SDK2) — zwischen Containern

DDS ist das Protokoll, das Unitree für die Low-Level-Kommunikation mit dem echten Roboter verwendet. Im Sim-Modus läuft es über Loopback (`lo`) statt über Ethernet.

```
                    DDS Domain 1, Interface: lo
                    ──────────────────────────

  g1_mujoco_sim                              g1pilot_sim
  ─────────────                              ───────────
  Bridge PUBLIZIERT:                         robot_state.py SUBSCRIBED:
    rt/lowstate  (LowState_)  ──────────►     rt/lowstate
    rt/sportmodestate          ──────────►     (liest Joint-Positionen,
    rt/wirelesscontroller                       IMU, publiziert /joint_states)

  Bridge SUBSCRIBED:                         arm_controller.py PUBLIZIERT:
    rt/lowcmd    (LowCmd_)   ◄──────────      rt/arm_sdk
    rt/arm_sdk   (LowCmd_)   ◄──────────      (sendet Drehmomente/Positionen)
```

**DDS Topics:**

| Topic | Typ | Richtung | Inhalt |
|---|---|---|---|
| `rt/lowstate` | `LowState_` | MuJoCo → G1Pilot | Joint-Positionen (q), Geschwindigkeiten (dq), Drehmomente (tau), IMU |
| `rt/lowcmd` | `LowCmd_` | G1Pilot → MuJoCo | Motor-Befehle (Standard-Channel für Locomotion) |
| `rt/arm_sdk` | `LowCmd_` | G1Pilot → MuJoCo | Motor-Befehle (Arm-Channel, von arm_controller benutzt) |
| `rt/sportmodestate` | `SportModeState_` | MuJoCo → G1Pilot | Odometrie (Position, Velocity) |
| `rt/wirelesscontroller` | `WirelessController_` | MuJoCo → G1Pilot | Gamepad-State |

**LowCmd_ Nachricht (Motor-Befehl):**
```
motor_cmd[i].tau  = Drehmoment (Nm)
motor_cmd[i].kp   = Positionsregler-Gain
motor_cmd[i].q    = Ziel-Position (rad)
motor_cmd[i].kd   = Geschwindigkeitsregler-Gain
motor_cmd[i].dq   = Ziel-Geschwindigkeit (rad/s)

Effektives ctrl = tau + kp * (q - q_ist) + kd * (dq - dq_ist)
```

**LowState_ Nachricht (Sensor-Daten):**
```
motor_state[i].q       = aktuelle Position (rad)
motor_state[i].dq      = aktuelle Geschwindigkeit (rad/s)
motor_state[i].tau_est  = geschätztes Drehmoment (Nm)
imu_state.quaternion    = Orientierung [w, x, y, z]
imu_state.gyroscope     = Drehrate [x, y, z]
imu_state.accelerometer = Beschleunigung [x, y, z]
```

### 2. ROS2 — innerhalb des g1pilot_sim Containers

ROS2 läuft auf Domain 0 und wird für die High-Level-Kommunikation zwischen den G1Pilot-Nodes verwendet.

```
                    ROS2 Domain 0
                    ─────────────

  robot_state.py ──► /joint_states (JointState) ──► rviz2
                                                 ──► arm_controller.py
                 ──► /tf (pelvis → alle Links)   ──► rviz2
                                                 ──► interactive_marker.py

  interactive_marker.py ──► /right_hand_target (PoseStamped) ──► arm_controller.py
                        ──► /left_hand_target  (PoseStamped) ──► arm_controller.py

  arm_controller.py ──► (DDS rt/arm_sdk) ──► MuJoCo
                    ──► /arm_status (Marker) ──► rviz2
```

### Wie ein Arm-Befehl durch das System fließt

```
1. User bewegt Interactive Marker in RViz
        │
2. interactive_marker.py publiziert /right_hand_target (PoseStamped)
        │  [ROS2, Domain 0]
3. arm_controller.py empfängt Target
        │
4. IK-Solver (Pinocchio) berechnet Joint-Winkel
        │
5. arm_controller.py sendet LowCmd_ auf rt/arm_sdk
        │  [DDS, Domain 1, lo]
6. unitree_sdk2py_bridge.py empfängt LowCmd_ im DDS-Callback
        │
7. Bridge berechnet ctrl[i] = tau + kp*(q_soll - q_ist) + kd*(dq_soll - dq_ist)
        │
8. SimulationThread wendet ctrl[] an → mujoco.mj_step()
        │
9. MuJoCo berechnet neue Physik (Joint-Positionen, Kräfte)
        │
10. Bridge liest mj_data.sensordata → publiziert LowState_ auf rt/lowstate
        │  [DDS, Domain 1, lo]
11. robot_state.py empfängt LowState_, publiziert /joint_states + /tf
        │  [ROS2, Domain 0]
12. RViz zeigt neue Roboter-Pose → Feedback-Loop schließt sich
```

---

## Joint-Layout (29 DoF)

```
Index  Joint                      Actuator              Typ
─────  ─────                      ────────              ───
 0     left_hip_pitch_joint       left_hip_pitch        Bein
 1     left_hip_roll_joint        left_hip_roll         Bein
 2     left_hip_yaw_joint         left_hip_yaw          Bein
 3     left_knee_joint            left_knee             Bein
 4     left_ankle_pitch_joint     left_ankle_pitch      Bein
 5     left_ankle_roll_joint      left_ankle_roll       Bein
 6     right_hip_pitch_joint      right_hip_pitch       Bein
 7     right_hip_roll_joint       right_hip_roll        Bein
 8     right_hip_yaw_joint        right_hip_yaw         Bein
 9     right_knee_joint           right_knee            Bein
10     right_ankle_pitch_joint    right_ankle_pitch     Bein
11     right_ankle_roll_joint     right_ankle_roll      Bein
12     waist_yaw_joint            waist_yaw             Torso
13     waist_roll_joint           waist_roll            Torso
14     waist_pitch_joint          waist_pitch           Torso
15     left_shoulder_pitch_joint  left_shoulder_pitch   Arm L
16     left_shoulder_roll_joint   left_shoulder_roll    Arm L
17     left_shoulder_yaw_joint    left_shoulder_yaw     Arm L
18     left_elbow_joint           left_elbow            Arm L
19     left_wrist_roll_joint      left_wrist_roll       Arm L
20     left_wrist_pitch_joint     left_wrist_pitch      Arm L
21     left_wrist_yaw_joint       left_wrist_yaw        Arm L
22     right_shoulder_pitch_joint right_shoulder_pitch   Arm R
23     right_shoulder_roll_joint  right_shoulder_roll    Arm R
24     right_shoulder_yaw_joint   right_shoulder_yaw     Arm R
25     right_elbow_joint          right_elbow            Arm R
26     right_wrist_roll_joint     right_wrist_roll       Arm R
27     right_wrist_pitch_joint    right_wrist_pitch      Arm R
28     right_wrist_yaw_joint      right_wrist_yaw        Arm R
```

Alle Aktuatoren: `gaintype=0, biastype=0` → reine Drehmoment-Motoren, `ctrlrange=[-25, 25]` Nm.

MuJoCo qpos-Layout (nq=36): `qpos[0:3]` = Pelvis-Position, `qpos[3:7]` = Pelvis-Quaternion, `qpos[7:36]` = 29 Joint-Winkel (gleiche Reihenfolge wie oben).

---

## Verzeichnisstruktur

```
~/g1_pilot_sim/
├── g1pilot/                          ← G1Pilot ROS2 Package (Hauptprojekt)
│   ├── Makefile                      ← Build/Start-Befehle (make sim, make stop, ...)
│   ├── docker-compose.sim.yaml       ← Container-Definition für Simulation
│   ├── docker/
│   │   ├── Dockerfile.sim            ← G1Pilot Container (ROS2 Humble, kein Livox)
│   │   ├── Dockerfile.mujoco         ← MuJoCo Container (Ubuntu 22.04, mujoco, unitree_sdk2py)
│   │   └── cyclonedds.xml            ← DDS Config: Loopback, kein Multicast
│   ├── launch/
│   │   ├── bringup_sim.launch.py     ← ★ Haupt-Launchfile für Simulation
│   │   ├── bringup_launcher.launch.py ← Haupt-Launchfile für echten Roboter
│   │   ├── robot_state_launcher.launch.py
│   │   ├── manipulation_launcher.launch.py
│   │   ├── teleoperation_launcher.launch.py
│   │   ├── livox_launcher.launch.py   ← (nur real, nicht im Sim)
│   │   ├── mola_launcher.launch.py    ← (nur real)
│   │   └── navigation_launcher.launch.py
│   ├── config/
│   │   ├── config.yaml               ← ROS2 Parameter
│   │   └── 29dof.rviz                ← RViz Konfiguration
│   ├── g1pilot/                      ← Python Source-Code
│   │   ├── state/
│   │   │   └── robot_state.py        ← DDS→ROS2 Bridge: LowState_ → /joint_states + /tf
│   │   ├── manipulation/
│   │   │   ├── arm_controller.py     ← IK + DDS: /hand_target → LowCmd_ auf rt/arm_sdk
│   │   │   ├── interactive_marker.py ← RViz Marker → /hand_target
│   │   │   └── dx3_hand.py           ← DX3 Hand-Controller (nicht verwendet)
│   │   ├── navigation/
│   │   │   └── loco_client.py        ← Locomotion (use_robot=false im Sim)
│   │   ├── teleoperation/
│   │   │   ├── joystick.py           ← Gamepad-Input
│   │   │   └── joy_mux.py            ← Multiplex verschiedener Steuer-Inputs
│   │   └── utils/
│   │       ├── ik_solver.py          ← Pinocchio IK
│   │       ├── joints_names.py       ← Joint-Definitionen
│   │       └── common.py             ← Shared Konstanten
│   └── description_files/
│       └── meshes/                   ← STL Meshes für URDF/RViz
│
└── unitree_mujoco/                   ← MuJoCo Simulator (von Unitree, modifiziert)
    ├── simulate_python/
    │   ├── unitree_mujoco.py         ← ★ Hauptdatei: SimulationThread + PhysicsViewerThread
    │   ├── unitree_sdk2py_bridge.py  ← ★ DDS↔MuJoCo Bridge: LowCmd→ctrl, sensordata→LowState
    │   └── config.py                 ← Sim-Konfiguration (ROBOT, DOMAIN_ID, INTERFACE, HOLD_BASE)
    └── unitree_robots/
        └── g1/
            └── scene.xml             ← MuJoCo Scene-Definition (MJCF)
```

---

## Befehle

### Simulation starten

```bash
cd ~/g1_pilot_sim/g1pilot

# Simulation starten (Vordergrund — Ctrl+C zum Stoppen)
make sim

# Simulation im Hintergrund
make sim-bg

# Stoppen
make stop

# Logs anschauen
make logs

# Status
make status
```

**Wichtig:** Beim ersten Start nach Boot `xhost +local:` ausführen (wird von `make sim` automatisch gemacht), damit Container auf das X11-Display zugreifen können.

### Container einzeln starten/debuggen

```bash
# MuJoCo Container Shell
make shell-mujoco

# G1Pilot Container Shell
make shell-sim

# Innerhalb g1pilot_sim manuell ROS2 starten:
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch g1pilot bringup_sim.launch.py
```

### Images neu bauen

```bash
# Alles neu bauen
make build-sim

# Nur MuJoCo (nach Änderungen an unitree_mujoco/)
make build-mujoco

# Nur G1Pilot ROS2 (nach Änderungen an g1pilot/)
make build-g1pilot-sim

# Alles löschen und von vorn
make clean
```

### Debugging & Diagnostik

```bash
# MuJoCo Container Logs (DDS Bridge, Physics)
docker logs g1_mujoco_sim -f 2>&1 | grep -E "CMD|SIM|DIAG|ERR"

# ROS2 Topics im G1Pilot Container anschauen
docker exec -it g1pilot_sim bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
docker exec -it g1pilot_sim bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /joint_states --once"

# Joint-Befehl manuell senden (Test)
docker exec -i g1pilot_sim python3 << 'EOF'
import time
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
ChannelFactoryInitialize(1, 'lo')
msg = unitree_hg_msg_dds__LowCmd_()
msg.motor_cmd[22].tau = -10.0        # Joint 22 = right_shoulder_pitch
msg.crc = CRC().Crc(msg)
pub = ChannelPublisher('rt/arm_sdk', LowCmd_)
pub.Init()
time.sleep(3)
for _ in range(50):
    pub.Write(msg)
    time.sleep(0.033)
print("Fertig!")
EOF
```

---

## Konfiguration

### unitree_mujoco/simulate_python/config.py

| Parameter | Default | Beschreibung |
|---|---|---|
| `ROBOT` | `"g1"` | Roboter-Modell |
| `DOMAIN_ID` | `1` | DDS Domain (muss mit G1Pilot übereinstimmen) |
| `INTERFACE` | `"lo"` | Netzwerk-Interface für DDS |
| `USE_JOYSTICK` | `0` | Gamepad im MuJoCo-Container (0 = aus, **muss 0 sein** ohne Gamepad!) |
| `ENABLE_ELASTIC_BAND` | `False` | Virtuelles Gummiband (für H1/G1 Balance-Tests) |
| `HOLD_BASE` | `True` | Pelvis in der Luft fixieren (für Arm-Tests ohne Balance) |
| `SIMULATE_DT` | `0.005` | Physik-Timestep (200 Hz) |
| `VIEWER_DT` | `0.02` | Viewer-Refresh (50 FPS) |

### Environment-Variablen (docker-compose.sim.yaml)

| Variable | Container | Wert | Beschreibung |
|---|---|---|---|
| `G1_SIM_MODE` | g1pilot_sim | `true` | Aktiviert Sim-Modus in robot_state.py und arm_controller.py |
| `ROS_DOMAIN_ID` | g1pilot_sim | `0` | ROS2 Domain |
| `UNITREE_DOMAIN_ID` | g1pilot_sim | `1` | DDS Domain für Unitree SDK |
| `CYCLONEDDS_URI` | beide | `file:///etc/cyclonedds.xml` | CycloneDDS Konfiguration |
| `DISPLAY` | beide | `:0` | X11 Display für GUI |

### HOLD_BASE deaktivieren/entfernen

```bash
# Deaktivieren (Roboter fällt, Physik wirkt voll):
sed -i 's/^HOLD_BASE = True/HOLD_BASE = False/' \
  ~/g1_pilot_sim/unitree_mujoco/simulate_python/config.py
# → danach: make build-mujoco && make sim

# Komplett entfernen: Blöcke zwischen den Markern löschen:
#   # === HOLD_BASE ... START ===
#   # === HOLD_BASE ... END ===
# in config.py und unitree_mujoco.py
```

---

---

## Sim ↔ Real Wechsel

### Übersicht der Unterschiede

| Aspekt | SIMULATION | ECHTER ROBOTER |
|---|---|---|
| **Compose-File** | `docker-compose.sim.yaml` | `docker-compose.real.yaml` ⚠️ *muss noch erstellt werden* |
| **Image (g1pilot)** | `g1pilot-sim:v1.1.0` | `g1pilot-real:v1.1.0` |
| **Dockerfile** | `docker/Dockerfile.sim` | `docker/Dockerfile` (mit Livox/MOLA, ~45 min Build) |
| **MuJoCo Container** | gestartet (`g1_mujoco_sim`) | NICHT gestartet |
| **Launch-File** | `bringup_sim.launch.py` | `bringup_launcher.launch.py` |
| **DDS Interface** | `lo` (Loopback) | `enp0s31f6` (Ethernet zum Roboter) |
| **DDS Domain (Unitree)** | `1` | `0` (Unitree-Standard) |
| **ROS_DOMAIN_ID** | `0` | `0` |
| **`G1_SIM_MODE`** | `true` | nicht gesetzt (oder `false`) |
| **`loco_client.use_robot`** | `false` | `true` |
| **Livox LiDAR** | aus | an |
| **MOLA SLAM** | aus | an |
| **`HOLD_BASE`** | `True` (Pelvis halten) | irrelevant (MuJoCo läuft nicht) |
| **`USE_JOYSTICK`** | muss `0` sein (kein Gamepad im Container) | irrelevant |
| **Make-Befehl** | `make sim` | `make real ROBOT_INTERFACE=enp0s31f6` |

### Was bei Sim → Real beachten muss

**1. Hardware vorbereiten**

- ✓ G1 ist eingeschaltet und im "ready"-Modus
- ✓ E-Stop in Reichweite
- ✓ Freie Fläche um den Roboter
- ✓ Inspire FTP Hände montiert (kein DX3 — `dx3_controller` wird crashen, das ist normal)
- ✓ Ethernet-Kabel verbunden: Host → Roboter
- ✓ Host-Interface ist UP (prüfen mit `ip -br link`)

**2. Interface verifizieren**

```bash
# Prüfen welches Interface mit Roboter verbunden ist:
ip -br link show
# Erwartet: enp0s31f6 ist UP (nicht DOWN wie aktuell)

# Verbindung zum Roboter testen (Default-IP Unitree G1: 192.168.123.161):
ping 192.168.123.161
```

**3. `docker-compose.real.yaml` erstellen**

⚠️ Die Datei existiert **noch nicht** im Projekt. Vorlage basierend auf der Sim-Variante:

```yaml
# ~/g1_pilot_sim/g1pilot/docker-compose.real.yaml
services:
  g1pilot_real:
    image: g1pilot-real:v1.1.0
    container_name: g1pilot_real
    network_mode: host
    privileged: true                  # für Hardware-Zugriff
    environment:
      - DISPLAY=${DISPLAY:-:0}
      - ROS_DOMAIN_ID=0
      - UNITREE_DOMAIN_ID=0           # Domain 0 für echten Roboter
      - INTERFACE=${ROBOT_INTERFACE:-enp0s31f6}
      - CYCLONEDDS_URI=file:///etc/cyclonedds.xml
      # KEIN G1_SIM_MODE → Code läuft in Real-Modus
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./:/workspace/g1pilot
      - /dev:/dev                     # USB für Livox/Gamepad
    working_dir: /workspace/g1pilot
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        colcon build --symlink-install 2>&1 | tail -5 &&
        source install/setup.bash &&
        ros2 launch g1pilot bringup_launcher.launch.py
      "
    restart: unless-stopped
    tty: true
```

⚠️ Die exakten Werte sind **Vorlage, nicht verifiziert**. Vor dem ersten Real-Betrieb prüfen ob:
- `UNITREE_DOMAIN_ID=0` korrekt ist (Unitree default — sollte stimmen)
- Sonstige Real-spezifische ENV-Variablen aus dem alten Original-Setup gebraucht werden

**4. Build + Start**

```bash
cd ~/g1_pilot_sim/g1pilot

# Sim stoppen falls am laufen
make stop

# Real-Image bauen (~45 min, lädt Livox SDK)
make build-real

# Starten — Interface explizit setzen wenn nicht enp0s31f6
make real ROBOT_INTERFACE=enp0s31f6
```

**5. Erste Schritte mit dem Roboter**

1. Beim Start: `loco_client` schickt automatisch einen Damping-Befehl → alle Motoren sind als Dämpfer aktiv, Roboter steht stabil
2. Erst dann: Arm-Befehle via RViz/Interactive Marker
3. Bei Problemen: **E-Stop drücken**, dann `make stop`

### Was bei Real → Sim beachten muss

```bash
cd ~/g1_pilot_sim/g1pilot

# Real-Stack stoppen
make stop

# Sim starten
make sim
```

Die config.py Anpassungen (`HOLD_BASE`, `USE_JOYSTICK=0`) bleiben drin — sie haben keine Wirkung im Real-Modus (MuJoCo läuft nicht).

### Wie der Code zwischen Sim und Real unterscheidet

In `g1pilot/state/robot_state.py` und `g1pilot/manipulation/arm_controller.py`:

```python
sim_mode = os.environ.get('G1_SIM_MODE', 'false').lower() == 'true'
if sim_mode:
    domain_id = int(os.environ.get('UNITREE_DOMAIN_ID', 1))
    interface = 'lo'
else:
    domain_id = 0
    interface = os.environ.get('INTERFACE', 'eth0')
ChannelFactoryInitialize(domain_id, interface)
```

Das heißt: **gleicher Code, anderes Verhalten je nach `G1_SIM_MODE`-Env-Variable.**

### Sicherheits-Checkliste vor jedem Real-Start

- [ ] E-Stop funktioniert und ist erreichbar
- [ ] Mindestens 2 m Freifläche um den Roboter
- [ ] Roboter steht stabil (Beine auf festem Boden)
- [ ] Erste Tests immer in Sim — IK-Limits, Workspace, alles validiert
- [ ] Bei Arm-Befehlen klein anfangen (kleine Delta-Bewegungen)
- [ ] Bei `kp`-Werten vorsichtig — zu hoch = aggressive Bewegung
- [ ] Tau-Limits beachten: G1 ctrlrange ist ±25 Nm pro Aktuator

---

## Bekannte Einschränkungen

- **Locomotion nicht simuliert:** `loco_client` läuft mit `use_robot=false`. Balance/Walking funktioniert nur auf echter Hardware.
- **DX3 Hand-Controller crasht** beim Start (kein DX3 angeschlossen). Kann ignoriert werden.
- **Joystick-Meldung** "No joystick found!" ist normal im Container ohne Gamepad.
- **USE_JOYSTICK muss 0 sein** wenn kein Gamepad angeschlossen ist, sonst stirbt der SimulationThread.
- **Ohne HOLD_BASE** fällt der Roboter sofort um (kein Balance-Controller im Sim).
