# G1Pilot — Sim-Teststystem (MuJoCo Arm-Manipulation)

Vollständiger Runbook zum **schichtweisen** Testen der Arm-Steuerkette in der
MuJoCo-Simulation. Jede Stufe wird **isoliert** geprüft, damit ein Fehler genau
einer Schicht zugeordnet werden kann. Jeder Test erzeugt ein Log, das du
zurückschicken kannst.

> Branch: `dev`. Vor dem Testen immer `git pull` + Container neu bauen/starten.

---

## 1. Die Kette (jede Stufe)

```
[L0] MuJoCo-Physik  ── HOLD_BASE (Weld/Steifigkeit) hält Oberkörper, Arm-PD
        │                (sim_hold_test.py: KEIN DDS, reiner In-Process-PD)
        ▼
[L1] Bridge  ── unitree_sdk2py_bridge: ApplyLowCmd() rechnet PD je Sim-Schritt
        │        publiziert rt/lowstate, abonniert rt/lowcmd + rt/arm_sdk
        ▼
[L2] DDS  ── Domain 1 / lo  (sim_dds_check: kommt rt/lowstate an?)
        │
        ▼
[L3] Aktuierung  ── ein Gelenk steady @200 Hz fahren (sim_arm_wiggle)
        │
        ▼
[L4] arm_controller ── liest rt/lowstate, IK, schreibt rt/arm_sdk
        │              (sim_arm_monitor: Soll-q vs Ist-q)
        ▼
[L5] Ziel-Eingang ── /g1pilot/hand_goal/{left,right}  (sim_goal_probe)
        │
        ▼
[L6] RViz Interactive Marker ── Marker ziehen → Arm fährt
```

Wichtige Fakten (verifiziert):
- Sensor-Layout im Modell: `jointpos[0..28]`, `jointvel[0..28]`, `frc[...]` →
  Bridge nutzt `sensordata[i]` / `sensordata[i+29]`. **Korrekt.**
- Unitree-DDS-Domain = **1** auf `lo`; **ROS_DOMAIN_ID = 0** (müssen verschieden sein).
- Arm-Motorindizes: links 15–21, rechts 22–28. arm_sdk-Weight an Index 29.

---

## 2. Vorbereitung

```bash
cd ~/g1simrepo/g1pilot && git pull

# Kompletter Stack (MuJoCo + g1pilot bringup):
xhost +local:root
G1_SIM_MODE=true docker compose --profile sim up        # Terminal A (laufen lassen)

# Beenden:
docker compose --profile sim down --remove-orphans
```

Zweite/dritte Shell in die laufenden Container:
```bash
docker exec -it g1pilot-g1pilot-sim-1 bash      # g1pilot-Seite (ROS/DDS-Tools)
docker exec -it g1pilot-mujoco-sim-1 bash       # MuJoCo-Seite (Physik-Test)
# (Container-Namen ggf. mit `docker ps --format '{{.Names}}'` prüfen)
```

In jeder g1pilot-Shell zuerst:
```bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
```

Logs landen unter `/ros2_ws/src/g1pilot/logs/` (= `~/g1simrepo/g1pilot/logs/` auf dem Host).

---

## 3. Tests

### L0 — Reine Physik (DER Schlüsseltest fürs Zittern)
**Zweck:** Hält die MuJoCo-Physik die Arme ruhig, wenn ein **konstanter PD** sie
hält — ganz **ohne** DDS, Bridge oder arm_controller? Damit trennen wir Physik
von der DDS-/Controller-Schicht.

**Befehl** (MuJoCo-Container; den Haupt-Sim vorher in Terminal A stoppen, damit
nur ein Prozess läuft — oder eigenen Wegwerf-Container nutzen):
```bash
docker compose --profile sim run --rm --no-deps mujoco-sim bash -lc '
  cd /unitree_mujoco/simulate_python &&
  python3 sim_hold_test.py --mode weld     --seconds 8 ;
  python3 sim_hold_test.py --mode teleport --seconds 8
' 2>&1 | tee ~/g1simrepo/g1pilot/logs/L0_hold_test.log
```

**Erwartete Ausgabe (pro Zeile):**
```
[hold_test] t= 7.50s MESS osc_span_max=0.0007 rad (joint 25)  mean|err|=0.012
...
[hold_test] ERGEBNIS mode=weld: groesste Schwingungsweite nach Einschwingen = 0.0009 rad
[hold_test] -> STABIL. ...
```

**Interpretation:**
- `mode=weld` **STABIL** (`< 0.01 rad`), `mode=teleport` **zittert** →
  der alte Teleport war die Ursache. Im Sim `HOLD_BASE_MODE="weld"` benutzen (Default).
- **Beide zittern** → das Zittern ist reine Physik (Gains/Integration). Dann L0 mit
  höherem `--kd` wiederholen:
  ```bash
  python3 sim_hold_test.py --mode weld --kd 25 --seconds 8
  python3 sim_hold_test.py --mode weld --kd 40 --seconds 8
  ```
  Den `kd`-Wert, der stabil wird, übernehmen wir in den arm_controller.
- **Beide stabil** → Physik ist gesund; das Zittern entsteht erst in der
  DDS-/Controller-Schicht → weiter mit L3/L4.

> Zum Zuschauen statt messen: `--view` anhängen (braucht X / `xhost +`).

---

### L1/L2 — DDS-Verbindung
**Zweck:** Kommt `rt/lowstate` von MuJoCo bei g1pilot an?
```bash
# g1pilot-Shell:
ros2 run g1pilot sim_dds_check --seconds 10 2>&1 | tee /ros2_ws/src/g1pilot/logs/L2_dds.log
```
**Erwartet:** `rate≈50 Hz`, `mode_machine`, Arm-q. **Fehlt alles** → MuJoCo läuft nicht
/ falsche Domain.

---

### L3 — Einzelgelenk steady @200 Hz (Bridge-Aktuierung)
**Zweck:** Fährt **ein** Gelenk sauber, wenn mit **konstant hoher Rate** gesendet wird?
Hält die Bridge die übrigen Arm­gelenke dabei ruhig?
```bash
ros2 run g1pilot sim_arm_wiggle --joint 22 --amp 0.4 --seconds 12 \
  2>&1 | tee /ros2_ws/src/g1pilot/logs/L3_wiggle.log
```
**Erwartet:** Gelenk 22 schwingt sauber sinusförmig; die anderen Arm­gelenke bleiben
ruhig. **Wackeln die gehaltenen Gelenke hier** (200 Hz steady) → Physik/Bridge.
**Sind sie ruhig** → die Aktuierung ist sauber, Problem entsteht erst bei niedriger/
schwankender Rate (L4).

---

### L4 — arm_controller: Soll vs. Ist
**Zweck:** Was schickt der Controller (rt/arm_sdk) vs. was misst MuJoCo (rt/lowstate)?
```bash
# Shell 1: Monitor
ros2 run g1pilot sim_arm_monitor --window 0.25 --seconds 30 \
  2>&1 | tee /ros2_ws/src/g1pilot/logs/L4_monitor.log
# Shell 2: Arme an, dann Marker ziehen / Ziel senden
ros2 topic pub -1 /g1pilot/arms/enabled std_msgs/Bool "{data: true}"
```
**Lesart:**
- `cmd` konstant (`osc_span_max≈0`), aber `meas` schwingt → Physik/HOLD_BASE (→ L0).
- `cmd` schwingt → Controller/IK (Smoothing, Seed).
- `arm_sdk rx_rate` sehr niedrig/schwankend (z. B. 12–40 Hz) → IK bremst den Loop
  (Performance; mit ApplyLowCmd kein Stabilitätsproblem, aber träge).

---

### L5 — Ziel-Eingang ohne Marker
**Zweck:** Reagiert der arm_controller auf ein Ziel (umgeht RViz-Marker)?
```bash
ros2 run g1pilot sim_goal_probe --ros-args -p side:=right -p axis:=z -p amp:=0.08 \
  2>&1 | tee /ros2_ws/src/g1pilot/logs/L5_goal_probe.log
```
**Erwartet:** Arm schwingt ±8 cm in z; Log zeigt sich ändernde `elbow=`/`shoulder=`.

---

### L6 — Voller Marker-Weg (RViz)
1. Arme an: `ros2 topic pub -1 /g1pilot/arms/enabled std_msgs/Bool "{data: true}"`
2. In RViz den **grünen** (rechts) bzw. **blauen** (links) Würfel ziehen.
3. Erwartung: Arm fährt kontrolliert zum Ziel und **steht ruhig**.
   (Erstes Ziehen wird durch `ee_auto_calibrate` geschluckt — danach folgt der Arm.)

---

## 4. Entscheidungsbaum „Arm zittert"

```
L0 sim_hold_test --mode weld
 ├─ STABIL  ────────────────► Physik ok. Zittern aus DDS/Controller:
 │                              L3 wiggle ruhig?  ─ja→ L4: cmd schwingt? → IK/Smoothing
 │                                                 └nein→ Bridge/Rate
 └─ ZITTERT ────────────────► Physik. Vergleiche:
        L0 --mode teleport zittert, weld stabil → Teleport war schuld (weld nutzen)
        beide zittern → --kd erhöhen bis stabil → Wert in arm_controller übernehmen
```

---

## 5. Tuning-Knöpfe

**HOLD_BASE (MuJoCo, `unitree_mujoco/simulate_python/config.py`):**
```python
HOLD_BASE_MODE = "weld"   # "weld" | "teleport" | "off"
HOLD_BASE_STIFFNESS = 2000.0
HOLD_BASE_DAMPING   = 80.0
```

**Arm-PD-Gains (live, ohne Rebuild):**
```bash
ros2 run g1pilot arm_controller --ros-args \
  -p interface:=lo -p use_robot:=true \
  -p kd_low:=18.0 -p kp_low:=150.0 -p kd_wrist:=6.0
# (Werte als DOUBLE schreiben: 18.0, nicht 18!)
```

**Marker-Publish-Default** (grün/aktiv vs. grau/aus):
```
ros2 launch ... manipulation_launcher.launch.py marker_publish_default:=true
```

---

## 6. Logs schicken
Sammle die relevanten Logs aus `~/g1simrepo/g1pilot/logs/` (z. B. `L0_hold_test.log`,
`L4_monitor.log`) plus das MuJoCo-Startlog (Zeile `[HOLD_BASE] Modus=...`). Damit
lässt sich die fehlerhafte Schicht eindeutig bestimmen.

---

## 7. Loco-/Balance-Controller (freies Stehen, MuJoCo)

Die Arm-Manipulation oben läuft mit fixierter Basis (`HOLD_BASE_MODE=weld`). Der
**Loco-Controller** (`loco_sim`) lässt den Roboter **frei stehen und balancieren** —
ohne Weld. Eine vortrainierte RL-Policy (unitree_rl_gym G1, `policies/g1/motion.pt`)
regelt die Beine; Taille/Arme werden gehalten, bis der `arm_controller` die Arme via
`rt/arm_sdk` (Weight-Blend) übernimmt.

> Branch: `loco`. Erfordert das **neu gebaute Sim-Image** (PyTorch im
> `Dockerfile.sim`) — beim ersten Mal `docker compose ... build g1pilot-sim`.

### Zwei Unterkörper-Regler (FSM, immer nur EINER aktiv)
Der **primäre** Modus ist ein **modellbasierter Balance-Regler** (am Platz stehen +
Oberkörper-Störungen ausgleichen, spiegelt `LocoClient.BalanceStand`). Die
**Walking-RL-Policy** bleibt als umschaltbarer **Bonus** erhalten (Laufen via
`loco_cmd_vel`). Sie teilen sich die Beine über die FSM — nie beide gleichzeitig.

```
loco_sim   : rt/lowstate (IMU+Gelenke) → [BALANCE: Knöchel/Hüft-Regler  |  RUN: Policy(LSTM)]
             → rt/lowcmd (Beine 0–11; Taille/Arme gehalten)
             + Zustands-Code an die Bridge via motor_cmd[29].q (0=HOLD, 1=balancierend, 2=DAMP)
arm_ctrl   : rt/arm_sdk (Arme 15–28, Weight@29)   ← überschreibt Arme gewichtet
Bridge     : merged beide Quellen pro Motor → mj_data.ctrl (PD je Sim-Schritt)
             Managed-Weld: hält die Basis (Weld), bis loco_sim Code 1 meldet → dann stellt
             es den Roboter in eine saubere Stand-Pose und löst den Weld.
FSM        : HOLD → [START BALANCING] → BALANCE (modellbasiert, am Platz)
                  → [start_walking]    → RUN     (Walking-Policy, Bonus)
                  → [EMERGENCY]        → DAMP
```

> **Warum modellbasiert als Primär?** Die unitree_rl_gym-Policy ist eine *Lauf*-Policy
> (validiert auf `cmd=[0.5,0,0]`); bei `cmd=0` marschiert sie auf der Stelle und driftet,
> weil ihre Observation **keine Basis-Position** enthält. Fürs reine Am-Platz-Stehen +
> Ausgleichen ist ein Knöchel-/Hüft-Balancer auf IMU-Feedback der bessere Fit:
> station-keeping per Konstruktion, deterministisch, kompensiert Oberkörper-Störungen
> automatisch (reagiert auf die gemessene Neigung, egal woher).

> **Warum Managed-Weld?** Ein freistehender Zweibeiner lässt sich NICHT statisch aufrecht
> halten — nur ein aktiver Balance-Regler hält ihn. Auf langsamen Rechnern fällt der
> Roboter aber um, bevor `loco_sim` (nach `colcon build`+Launch) verbunden ist. Darum
> hält die Bridge die Basis fest, bis das Balancieren wirklich startet.

> **Langsamer PC / Realtime-Faktor:** `run_sim_loco.sh` setzt `SIM_REALTIME_FACTOR=0.65`,
> damit pro Regelschritt ~20 ms Physik vergehen (= Trainingsrate 50 Hz), auch wenn der
> Loop nur ~33 Hz Wall-Clock schafft. In der `[timing]`-Zeile soll **`sim-effektiv`**
> ~50 Hz zeigen; sonst `SIM_REALTIME_FACTOR` = (wall-Hz / 50) anpassen.

### Start (freie Basis)
```bash
cd ~/g1simrepo/g1pilot && git checkout loco && git pull
./run_sim_loco.sh        # setzt HOLD_BASE_MODE=off → Basis FREI
```
MuJoCo-Startlog muss `[HOLD_BASE] Modus=off (managed): Basis vorerst gehalten` und
`[BRIDGE] Managed-Weld aktiv ...` zeigen. Die Bridge hält die Basis (Weld), bis
`loco_sim` das Balancieren startet — der Roboter steht also beim Launch sicher und
fällt **nicht** um (egal wie langsam der PC ist). `loco_sim` meldet `Policy geladen:
... recurrent=True`. Im HOLD wird die Basis gehalten (noch kein aktives Balancieren).

### Ablauf
1. **START BALANCING** (`/g1pilot/start_balancing`) → Bridge stellt den Roboter auf,
   löst den Weld, `loco_sim` aktiviert den **modellbasierten Balance-Regler** (BALANCE)
   → Roboter steht **frei und hält die Position**:
   ```bash
   ros2 topic pub -1 /g1pilot/start_balancing std_msgs/Bool "{data: true}"
   ```
2. **Arme + Balance gleichzeitig:** Arme an + Marker ziehen (wie L6). Der Balancer
   gleicht die Oberkörper-Störung automatisch aus, Arme folgen — keiner geht limp:
   ```bash
   ros2 topic pub -1 /g1pilot/arms/enabled std_msgs/Bool "{data: true}"
   ```
3. **EMERGENCY STOP** (`/g1pilot/emergency_stop`) → DAMP (weich) **und** Arme aus:
   ```bash
   ros2 topic pub -1 /g1pilot/emergency_stop std_msgs/Bool "{data: true}"
   ```

### Balance-Regler tunen (live, ohne Rebuild)
Stellgröße ist **Feedforward-Drehmoment [Nm]** (mc.tau), bis ±50 Nm Knöchel-Limit.
Die `[state]`-Logzeile zeigt `grav` (aufrecht=`[0 0 -1]`; `gx>0`=vorn, `gy>0`=links)
und `|action|` = mittleres Stell-Drehmoment. Vorzeichen sind aus der Gelenk-Kinematik
abgeleitet; falls `grav` bei BALANCE **schneller** divergiert (Regler verstärkt) →
betroffenes `kp` negieren:
```bash
ros2 param set /loco_sim bal_ankle_kp_pitch 600.0   # stärker/früher gegen Vor/Zurück (Nm/Neigung)
ros2 param set /loco_sim bal_ankle_kp_roll  600.0   # stärker gegen Seitkippen
ros2 param set /loco_sim bal_ankle_kd_pitch 40.0    # mehr Dämpfung (gegen Schwingen, Nm/(rad/s))
ros2 param set /loco_sim bal_hip_kp_pitch   150.0   # Hüft-Sekundärstrategie zuschalten
ros2 param set /loco_sim bal_kp_scale       2.0     # Posture-Beine steifer (mehr passive Stabilität)
# Vorzeichen umdrehen, falls verstärkt statt abgefangen:
ros2 param set /loco_sim bal_ankle_kp_pitch -400.0
```

### Bonus: Laufen (Walking-Policy)
```bash
ros2 topic pub -1 /g1pilot/start_walking std_msgs/Bool "{data: true}"   # -> RUN
ros2 topic pub -1 /g1pilot/loco_cmd_vel geometry_msgs/Twist "{linear: {x: 0.4}}"  # vorwärts
ros2 topic pub -1 /g1pilot/loco_cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}}"  # stoppen
```

### Fehlersuche
- **Roboter sackt beim Launch zusammen** (vor START): Managed-Weld greift nicht —
  prüfe das MuJoCo-Startlog auf `[HOLD_BASE] Modus=off (managed)` und
  `[BRIDGE] Managed-Weld aktiv`. Fehlt der `off`-Modus, wurde nicht über
  `run_sim_loco.sh` gestartet (HOLD_BASE_MODE nicht gesetzt).
- **Beim START BALANCING „springt" der Roboter kurz** in die Stand-Pose: Das ist
  gewollt (Reset auf einen sauberen, aufrechten Start für die Policy).
- **Policy balanciert, kippt aber nach einigen Sekunden** weg: Sim-zu-Sim-Lücke
  (Policy in Isaac Gym trainiert, hier MuJoCo). Beobachten: nach wie vielen Sekunden
  und in welche Richtung (vor/zurück/seitlich) — danach Reibung/Gains/`LOCO_RESET_PELVIS_Z`
  nachziehen.
- **`Policy geladen` fehlt / ImportError torch** → Sim-Image nicht neu gebaut
  (`docker compose --profile sim build g1pilot-sim`).
- **Steht im HOLD, kippt aber nach START BALANCING** → Policy-Mapping/Sim-Realismus.
  Erst `recurrent=True` und `[HOLD_BASE] Modus=off` verifizieren; dann ggf.
  Reibung/Trägheit im Modell vs. Trainings-Setup prüfen.
- **Arme gehen beim Balancieren limp** → Merge/Weight: `arm_controller` muss
  `rt/arm_sdk` mit Weight@29=1 publishen (Arme aktiviert).
