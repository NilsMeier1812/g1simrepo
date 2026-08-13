# Locomotion — Technik

Richtet sich an: Entwickler, die den Stand-/Lauf-Regler ändern oder tunen
wollen. Für die Bedienung siehe [30_loco_anleitung.md](30_loco_anleitung.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/navigation/loco_sim.py` | Sim-Stellvertreter für Stehen/Laufen (nur Simulation) |
| `g1pilot/navigation/loco_client.py` | Ansteuerung des Unitree-Onboard-Reglers (nur echter Roboter) |
| `g1pilot/policies/g1_wholebody/policy.onnx` + `deploy.yaml` | RL-Lauf-Policy (unitree_rl_mjlab G1 Velocity, Apache-2.0) |
| `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py` | Mischt Bein-/Taillen-Kommandos (`rt/lowcmd`) mit Arm-Kommandos (`rt/arm_sdk`) |

## Warum zwei völlig verschiedene Implementierungen?

Auf dem **echten** G1 läuft Stehen/Balancieren/Gehen auf einem
Onboard-Controller von Unitree, den man nur über eine High-Level-API
(`LocoClient.BalanceStand`/`Move`/`StopMove`/`Damp`) anspricht. In **MuJoCo
gibt es diesen Onboard-Controller nicht** — die Simulation liefert nur die
rohe Low-Level-Schnittstelle (Gelenke + IMU rein, Motor-Befehle raus). Daher
gibt es einen eigenen Sim-Ersatzregler, `loco_sim`, der dieselben
Steuertopics wie `loco_client` bedient, intern aber komplett anders
funktioniert.

## `loco_sim` (Simulation)

### Zwei kombinierte Regler

- **STAND** (am Platz): modellbasierter Knöchel-/Hüft-PD-Regler. Hält die
  Füße geplant und richtet die per IMU gemessene Neigung aktiv auf — Arm-
  und Oberkörperstörungen werden ohne Schritte abgefangen.
- **WALK** (laufen): eine vortrainierte, velocity-konditionierte
  Ganzkörper-ONNX-Policy (`policies/g1_wholebody/policy.onnx`). Läuft
  omnidirektional; bei `cmd=0` steht sie einfach am Platz (`gait_phase=0`).

Grund für die Kombination: Die Policy läuft gut, aber unter dauerhafter
Arm-Bewegung im Stand driftet sie progressiv weg — sie „balanciert" jede
Störung per Schritt. Der PD-Regler hält dagegen die Füße wirklich fest.
Also: Policy fürs Laufen, PD fürs stationäre Stehen.

### Zustandsmaschine

```python
HOLD = "hold"    # Standby, Basis gehalten (Bridge: weld)
STAND = "stand"  # PD-Balancer, Füße geplant
WALK = "walk"    # ONNX-Policy
DAMP = "damp"    # Emergency / Sturz: alle Motoren weich
```

Übergänge **ausschließlich per Nutzer-Button/Topic**, kein automatisches
Umschalten zwischen STAND und WALK:

- `/g1pilot/start_balancing(True)` → `_enter_stand()`.
- `/g1pilot/start_walking(True)` → wartet zunächst weiter im STAND, bis der
  `arm_controller` die Arme in eine definierte Lauf-Pose gebracht hat
  (`/g1pilot/arms/walk_ready`, siehe
  [11_arm_manipulation_technik.md](11_arm_manipulation_technik.md)) oder ein
  Timeout (`walk_arm_timeout_s`, Default 4 s) abläuft — dann `_enter_walk()`.
  Grund: die Policy ist nur mit Armen nahe der Trainings-Pose stabil.
- `/g1pilot/emergency_stop(True)` → sofort `DAMP` + Arme deaktivieren.
- `/g1pilot/start(True)` → `HOLD`.
- Sturz-Erkennung (`_fallen`, IMU-Neigung über `fall_gz` für
  `fall_debounce_s`) → automatisch `DAMP`, unabhängig vom Auslöser.

### Zuständigkeit der Gelenke

`loco_sim` regelt **ausschließlich** Beine (0–11) und Taille (12–14),
niemals die Arme (15–28) — die gehören komplett dem `arm_controller`
(`rt/arm_sdk`). Das verhindert, dass ein Zustandswechsel im Loco-Regler
die Arme „teleportiert".

### Regelschleife (`_control_loop`)

Läuft in einem eigenen Thread, getaktet über `control_dt` aus
`deploy.yaml` (Policy-Rate, i. d. R. 20 ms = 50 Hz). Zwei Taktmodi:

- **Lockstep** (`SIM_LOCKSTEP=1`, Standard im Betrieb): wartet pro
  Iteration auf den nächsten `rt/lowstate`-Eingang, statt auf die
  Wall-Clock zu takten — deterministische 50-Hz-Regelrate unabhängig von
  der Rechnerlast.
- **Echtzeit** (`SIM_LOCKSTEP=0`): schläft die Restzeit von `control_dt`
  ab; `SIM_REALTIME_FACTOR` skaliert das Tempo.

`_send_hold` / `_send_damp` / `_send_balance_pd` / `_send_policy` schreiben
jeweils `rt/lowcmd` und (über `_write`) einen Zustandscode in
`motor_cmd[29].q` (`STATE_IDX`), den die Bridge zur Basis-Physik nutzt
(`weld` im HOLD, frei sonst).

### PD-Balancer (`_send_balance_pd`)

Feedforward-Drehmoment auf Knöchel (primär) und Hüfte (sekundär), berechnet
aus der IMU-Neigung (`get_gravity_orientation`) und Gyroskop-Rate:

```
t_ankle_pitch = kp*pitch_err + kd*pitch_rate + integral_trim
t_ankle_roll  = -(kp*roll_err + kd*roll_rate)
t_hip_pitch   = kp*pitch_err + kd*pitch_rate
t_hip_roll    = -(kp*roll_err + kd*roll_rate)
t_hip_yaw     = -kd*yaw_rate
```

Ein Integral-Trim auf den Knöchel-Pitch (`bal_ki_pitch`) gleicht statische
Schwerpunktversätze aus (z. B. schwerere Inspire-FTP-Hände verschieben den
Schwerpunkt nach vorn) — ohne ihn bliebe eine Dauerneigung stehen. Ein
sanfter Eintritts-Rampe (`bal_ramp_s`) blendet beim Wechsel aus WALK die
aktuelle Beinpose zur Standardpose, damit der steife PD die Beine nicht aus
der Lauf-Stellung reißt; aus HOLD ist die Rampe bewusst kurz (0.1 s), weil
eine längere Weich-Phase den (durch Hände kopflastigeren) Roboter
unaufholbar nach vorn kippen ließ.

### Policy (`_send_policy`, `_build_obs`)

98 Beobachtungswerte: Gyroskop (3), projizierte Gravitation (3),
Geschwindigkeitskommando (3), Gait-Phase sin/cos (2), Gelenkabweichung von
`default_joint_pos` (29), Gelenkgeschwindigkeit (29), letzte Aktion (29).
`deploy.yaml` liefert PD-Gains (`stiffness`/`damping`), `default_joint_pos`,
Aktions-Skalierung und die Geschwindigkeits-Ranges (`commands.base_velocity`).
Nur die ersten 15 Gelenke (Beine + Taille) werden aus dem Policy-Ausgang
aktuiert.

### Konfigurierbare Parameter (Auszug)

| Parameter | Bedeutung |
|---|---|
| `stand_eps` | Schwelle `‖cmd‖`, ab der die Policy als „stehend" gilt |
| `fall_gz`, `fall_debounce_s` | Sturz-Erkennungsschwelle/-Entprellung |
| `hold_kd_scale` | Dämpfungsfaktor der Beine im HOLD |
| `bal_kp_scale`, `bal_ramp_s` | Steifigkeit/Eintrittsrampe des PD-Balancers |
| `bal_ki_pitch`, `bal_i_limit` | Integral-Trim gegen statische Neigung |
| `bal_ankle_kp_pitch/roll`, `bal_hip_kp_pitch/roll`, `bal_yaw_kd` | PD-Gains je Achse |
| `walk_arm_wait`, `walk_arm_timeout_s` | Warten auf Arm-Aufräumen vor WALK |

Live änderbar via `ros2 param set /loco_sim <name> <wert>`.

### Zusatzfunktionen (nur Sim)

- **PUSH** (`/g1pilot/push`) — schickt einen UDP-Stoßimpuls an die Sim, um
  die Störunterdrückung zu testen.
- **GRASP BOX** (`/g1pilot/grasp_box`) — schaltet eine greifbare Testkugel
  in der Handfläche an/aus (Inspire-Hände, siehe
  [61_inspire_haende_technik.md](61_inspire_haende_technik.md)).

## `loco_client` (echter Roboter)

Dünner ROS-Node um `unitree_sdk2py.g1.loco.g1_loco_client.LocoClient`. Bildet
dieselben Streamdeck-Topics wie `loco_sim` auf die Unitree-High-Level-RPCs
ab:

| Topic | RPC |
|---|---|
| `/g1pilot/start` | `SetFsmId(4)` (Standby) |
| `/g1pilot/start_balancing` | `entering_balancing()` — Höhenrampe + `BalanceStand(1)` + `Start()` |
| `/g1pilot/start_walking` + `/g1pilot/loco_cmd_vel` | `Move(vx, vy, vyaw, continous_move=True)` in `_cmd_vel_tick` (20 Hz) |
| `/g1pilot/emergency_stop` | `Damp()`, sofort, eigene Callback-Gruppe |

Wichtige Sicherheitsmechanismen:

- **RPC-Serialisierung** (`_rpc_lock`): der E-Stop läuft in einer eigenen
  `MultiThreadedExecutor`-Callback-Gruppe, damit er auch während eines
  laufenden, blockierenden `entering_balancing()` sofort durchkommt.
- **Deadman/Timeout** (`_cmd_vel_tick`): `Move()` läuft nur, solange
  balanciert, nicht gestoppt, `WALK` aktiv und der `loco_cmd_vel`-Stream
  frisch ist (`cmd_vel_timeout`, Default 0.5 s). Fällt eine Bedingung weg,
  wird genau einmal `StopMove()` gesendet.
- **PS4-Controller-Pfad** (`joystick_callback`): physischer Deadman-Button
  (Index 8) hat Vorrang vor dem Streamdeck-`loco_cmd_vel`-Pfad, solange
  gedrückt.
- `damp_on_init=False`: der Node greift beim Start nicht von sich aus in
  den Roboterzustand ein — ein bereits stehender G1 würde bei `Damp()`
  sofort zusammensacken.

## Bridge-Merge (Sim)

`unitree_sdk2py_bridge.py` (in `unitree_mujoco`) empfängt sowohl
`rt/lowcmd` (Beine/Taille, von `loco_sim`) als auch `rt/arm_sdk`
(Arme, von `arm_controller`) und mischt sie pro Motor: Beine/Taille
übernehmen unverändert `rt/lowcmd`; die Arme werden mit dem in
`rt/arm_sdk`-`motor_cmd[29].q` transportierten Gewicht zwischen
`rt/lowcmd`-Fallback (0) und `rt/arm_sdk`-Kommando (1) überblendet. Details
zum DDS-Merge in [02_architektur.md](02_architektur.md).

## Bekannte Einschränkungen

- `loco_sim` ist ein reiner **Sim-Stellvertreter**. Die konkrete
  Balance-/Lauf-Strategie ist eigens für die Simulation gebaut und teils auf
  simulationsinterne Größen angewiesen (z. B. Basisgeschwindigkeit/Fußkraft
  in `reserve[]` der Bridge) — sie ist **nicht** für den Realeinsatz gedacht.
- Die Lauf-Policy ist die kanonische `unitree_rl_mjlab`-G1-Policy
  (velocity-getaktet); ein driftfreier Stand kommt ausschließlich vom
  modellbasierten PD.
- `USE_JOYSTICK=0` ist zwingend, solange kein Gamepad im Container hängt —
  sonst stirbt der Sim-Thread.
