# CHEATS:

## ENABLING / DISABLING COMMANDS

### START
```bash
ros2 topic pub --once /g1pilot/start std_msgs/msg/Bool "{data: true}"
```

### EMERGENCY STOP
```bash
ros2 topic pub --once /g1pilot/emergency_stop std_msgs/msg/Bool "{data: true}"
```

### START BALANCING
```bash
ros2 topic pub --once /g1pilot/start_balancing std_msgs/msg/Bool "{data: true}"
```

### WALK (Laufen) + Gehen->Stehen-Handoff
Aktiviert die Walking-Policy (RUN). Geschwindigkeit dann ueber `/g1pilot/loco_cmd_vel`
(normiert [-1,1]; vx=1.0 ~ 0.8 m/s). Im Streamdeck: Button "WALK" + Bildschirm-Joystick.
Die rekurrente Policy laeuft im RUN DURCHGEHEND (kein Command-Gating — das machte den
Gang instabil). Zentriert man das Kommando (cmd~0) nach echtem Laufen, BREMST loco_sim
aktiv ueber die Policy ab (STEPPING-STOP: Gegen-Kommando ~ -brake_gain * Basis-
Geschwindigkeit aus den Fuss-/v-Sensoren) und uebergibt erst im echten Stillstand
(|v| < brake_vstop UND beide Fuesse belastet = Doppelstuetz) sanft an den PD-Stand.
Das ist der robuste Weg, aus BELIEBIGER Richtung anzuhalten (headless 15-16/16, vorher
nur vorwaerts). Sehr schnelle Diagonal-Stopps bleiben grenzwertig. Fehlt die Sensorik
(reserve[] leer), Fallback auf den zeitbasierten Handoff (`walk_stop_settle_s`, 0.5 s).
Tuning live: `ros2 param set /loco_sim brake_gain 2.5` (auch brake_vstop, brake_ds_force,
brake_timeout). PD-Fang: `bal_vel_kv` (CoM-v-Daempfung), `bal_yaw_kd` (Yaw-Daempfung).
```bash
ros2 topic pub --once /g1pilot/start_walking std_msgs/msg/Bool "{data: true}"
ros2 topic pub /g1pilot/loco_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"  # vorwaerts
ros2 topic pub --once /g1pilot/loco_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"  # Stop -> Auto-Handoff zu PD
```
Ausroll-Zeit tunebar: `ros2 param set /loco_sim walk_stop_settle_s 0.5`.

### CATCH FALLS (Stuerze abfangen, Toggle — Default AN)
Schaltet bei drohendem Sturz automatisch vom PD-Balancer auf die steppfaehige
RL-Policy um (faengt ~80-150 N statt ~50 N) und kehrt nach dem Auffangen zum
PD-Stand zurueck. Auch als Streamdeck-Button "CATCH FALLS".
```bash
ros2 topic pub --once /g1pilot/catch_falls std_msgs/msg/Bool "{data: true}"   # an (Default)
ros2 topic pub --once /g1pilot/catch_falls std_msgs/msg/Bool "{data: false}"  # aus (reiner PD)
```
Schwelle tunebar: `ros2 param set /loco_sim catch_tilt 0.15` (hoeher = spaeter umschalten).

### PUSH ROBOT (Stoer-Test, nur Sim)
Schubst den Roboter mit einem kurzen Impuls in ZUFAELLIGER Richtung, um die
Stoerunterdrueckung des Balancers zu testen (auch als Streamdeck-Button "PUSH ROBOT").
```bash
ros2 topic pub --once /g1pilot/push std_msgs/msg/Bool "{data: true}"
```
Kraft/Dauer per Env an der Sim (Default 150 N, 120 ms): `SIM_PUSH_FORCE_N`,
`SIM_PUSH_DURATION_S`. Direkt ohne ROS: `echo -n 250 | nc -u -w0 127.0.0.1 47900`
(Payload = Kraft in N).

### GRASP BOX (greifbare Test-Kugel an/aus, nur Sim, nur Inspire-Haende)
Legt in jede Handflaeche eine kleine greifbare Kugel -> beim Schliessen der Hand
sofort echte Griffkraefte in der Hand-GUI. Als Streamdeck-Button "GRASP BOX" (Toggle)
oder:
```bash
ros2 topic pub --once /g1pilot/grasp_box std_msgs/msg/Bool "{data: true}"   # an
ros2 topic pub --once /g1pilot/grasp_box std_msgs/msg/Bool "{data: false}"  # aus
```
Direkt ohne ROS: `echo -n on | nc -u -w0 127.0.0.1 47901` (`on`/`off`/sonst = toggle).
Beim Sim-Start direkt AN: `G1_GRASP_TEST=1`. Ganz weglassen: `G1_GRASP_BOX=0`.
Die Kugel haengt starr in der Griffzone der Palme (faellt nicht) und ist im Aus-
Zustand unsichtbar und inert (~6 g, keine Kollision).

## PUBLISHING COMMANDS FOR NAVIGATION

###  PUBLISH GOAL
```bash
ros2 topic pub --once /g1pilot/goal geometry_msgs/PointStamped "{header: {frame_id: 'map'}, point: {x: 1.0, y: 0.0, z: 0.0}}"
```

### ENABLE AUTONOMOUS NAVIGATION
```bash
ros2 topic pub --once /g1pilot/auto_enable std_msgs/msg/Bool "{data: true}"
```

## PUBLISHING COMMANDS FOR MANIPULATION

### ENABLE MANIPULATION (way: 1)
```bash
ros2 topic pub --once /g1pilot/joy sensor_msgs/msg/Joy '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""}, axes: [0,0,0,0,0,0,0,0], buttons: [1,0,0,0,0,0,0,0,0,0,0,0]}'
```

### ENABLE MANIPULATION (way: 2)
```bash
ros2 topic pub --once /g1pilot/arms/enabled std_msgs/msg/Bool "{data: true}"
```

### HOMMING ARMS
```bash
ros2 topic pub --once /g1pilot/arms/home std_msgs/msg/Bool "{data: true}"
```

### PUBLISH POINT
```bash
ros2 topic pub -1 /g1pilot/hand_goal/left geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'pelvis'}, pose: {position: {x: 0.20, y: 0.17, z: 0.09}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### CONTROL INSPIRE HAND (for left and right hand)
```bash
ros2 topic pub --once /g1pilot/hand_action/right std_msgs/msg/String "{data: 'close'}"
```
```bash
ros2 topic pub --once /g1pilot/hand_action/right std_msgs/msg/String "{data: 'open'}"
```