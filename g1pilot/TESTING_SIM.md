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

### Architektur (kurz)
```
loco_sim   : rt/lowstate (IMU+Gelenke) → Policy(LSTM) → rt/lowcmd (Beine 0–11, Taille/Arme gehalten)
arm_ctrl   : rt/arm_sdk (Arme 15–28, Weight@29)   ← überschreibt Arme gewichtet
Bridge     : merged beide Quellen pro Motor → mj_data.ctrl (PD je Sim-Schritt)
FSM        : HOLD (steifer Stand) → [START BALANCING] → RUN (Policy) ; [EMERGENCY] → DAMP
```

### Start (freie Basis)
```bash
cd ~/g1simrepo/g1pilot && git checkout loco && git pull
./run_sim_loco.sh        # setzt HOLD_BASE_MODE=off → Basis FREI
```
MuJoCo-Startlog muss `[HOLD_BASE] Modus=off: Basis frei (Weld aus).` und
`[BRIDGE] Loco-Startup-Hold aktiv ...` zeigen. Die Bridge hält den Roboter im
Startfenster (während `colcon build`/Launch) in einer Standpose, bis `loco_sim`
verbunden ist — er fällt also **nicht** mehr beim Launch um. `loco_sim` meldet dann
`loco_sim bereit (HOLD = steifer Stand)` und `Policy geladen: ... recurrent=True`
und übernimmt nahtlos. Im HOLD steht der Roboter steif (balanciert noch nicht aktiv).

### Ablauf
1. **START BALANCING** (Streamdeck-Button, publisht `/g1pilot/start_balancing`) →
   `loco_sim` rampt in die Default-Pose und aktiviert die Policy → Roboter steht
   **frei und balanciert** (sollte ≥30 s stehen, auch bei kleinem Maus-Schubs).
   Manuell ohne Streamdeck:
   ```bash
   ros2 topic pub -1 /g1pilot/start_balancing std_msgs/Bool "{data: true}"
   ```
2. **Arme + Loco gleichzeitig:** Arme an + Marker ziehen (wie L6). Beine balancieren
   weiter, Arme folgen — keiner geht limp (beweist den Bridge-Merge):
   ```bash
   ros2 topic pub -1 /g1pilot/arms/enabled std_msgs/Bool "{data: true}"
   ```
3. **EMERGENCY STOP** (`/g1pilot/emergency_stop`) → `loco_sim` dampt (weich, sanftes
   Hinsetzen) **und** schaltet die Arme aus:
   ```bash
   ros2 topic pub -1 /g1pilot/emergency_stop std_msgs/Bool "{data: true}"
   ```

### Fehlersuche
- **Roboter sackt beim Launch zusammen** (vor START): Der Bridge-Startup-Hold greift
  nicht — prüfe das MuJoCo-Startlog auf `[HOLD_BASE] Modus=off` und
  `[BRIDGE] Loco-Startup-Hold aktiv`. Fehlt der `off`-Modus, wurde nicht über
  `run_sim_loco.sh` gestartet (HOLD_BASE_MODE nicht gesetzt). Danach muss `loco_sim`
  `rt/lowstate` bekommen und übernehmen (`ros2 node list` → `loco_sim`).
- **`Policy geladen` fehlt / ImportError torch** → Sim-Image nicht neu gebaut
  (`docker compose --profile sim build g1pilot-sim`).
- **Steht im HOLD, kippt aber nach START BALANCING** → Policy-Mapping/Sim-Realismus.
  Erst `recurrent=True` und `[HOLD_BASE] Modus=off` verifizieren; dann ggf.
  Reibung/Trägheit im Modell vs. Trainings-Setup prüfen.
- **Arme gehen beim Balancieren limp** → Merge/Weight: `arm_controller` muss
  `rt/arm_sdk` mit Weight@29=1 publishen (Arme aktiviert).
