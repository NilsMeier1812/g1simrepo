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

### CONTROL DX3 HAND (for left and right hand)
```bash
ros2 topic pub --once /g1pilot/dx3/hand_action/right std_msgs/msg/String "{data: 'close'}"
```
```bash
ros2 topic pub --once /g1pilot/dx3/hand_action/right std_msgs/msg/String "{data: 'open'}"
```