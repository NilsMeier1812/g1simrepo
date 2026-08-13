# Locomotion (Stehen/Laufen) — Anleitung

Richtet sich an: Anwender, die den G1 in der Simulation oder auf echter
Hardware stehen und laufen lassen wollen. Für die interne Funktionsweise
siehe [31_loco_technik.md](31_loco_technik.md).

## Überblick

Die Locomotion (Beine + Taille) ist eine kleine Zustandsmaschine mit vier
Zuständen, gesteuert über den Streamdeck oder ROS-Topics. In der Simulation
übernimmt der Node `loco_sim` die Rolle des Unitree-Onboard-Reglers (den es
in MuJoCo nicht gibt); auf dem echten Roboter läuft weiterhin Unitrees
eigener Onboard-Regler, angesprochen über `loco_client`.

| Zustand | Was passiert | Auslöser |
|---|---|---|
| **HOLD** | Standby: steifer Stand, Basis gehalten | Start / `/g1pilot/start` |
| **BALANCE** (Sim: STAND) | Modellbasierter Stand am Platz, Oberkörper frei bewegbar | **START BALANCING** / `/g1pilot/start_balancing` |
| **RUN** (Sim: WALK) | Lauf-Regler; Geschwindigkeit per Joystick | **WALK** + `/g1pilot/loco_cmd_vel` |
| **DAMP** | Notaus: alle Motoren weich | **EMERGENCY** / `/g1pilot/emergency_stop` |

## Bedienung

**Typischer Ablauf:**

1. **START BALANCING** → der Roboter steht frei und balanciert; die Arme
   lassen sich parallel über die Marker bewegen.
2. **WALK** → der Bildschirm-Joystick (bzw. echter Controller auf real) gibt
   Vorwärts/Seitwärts/Drehen vor.
3. Joystick loslassen → der Roboter bremst ab und geht zurück in den
   stationären Stand.

Über die Kommandozeile:

```bash
ros2 topic pub --once /g1pilot/start_balancing std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /g1pilot/start_walking   std_msgs/msg/Bool "{data: true}"
ros2 topic pub /g1pilot/loco_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"   # vorwärts
ros2 topic pub --once /g1pilot/loco_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"  # Stop
```

`loco_cmd_vel` ist normiert `[-1, 1]`; `linear.x` = vorwärts/rückwärts,
`linear.y` = seitwärts, `angular.z` = drehen. Die tatsächliche
Geschwindigkeit ergibt sich aus den konfigurierten Limits (siehe unten).

## Sim ↔ Real

| Aspekt | Simulation | Echter Roboter |
|---|---|---|
| Regler | `loco_sim` (RL-Lauf-Policy + modellbasierter Stand-Regler) | Unitrees Onboard-High-Level, über `loco_client` |
| Node-Start | automatisch mit `bringup_sim.launch.py` | automatisch mit `bringup_real.launch.py` |
| Steuertopics | identisch | identisch |

Der Streamdeck verhält sich in beiden Modi identisch — dieselben Buttons,
dieselben Topics.

**Wichtig:** `loco_sim` ist ein **reiner Sim-Stellvertreter**. Auf dem
echten G1 übernimmt ausschließlich Unitrees Onboard-Regler das
Stehen/Laufen; für Details zur echten Hardware siehe
[70_echtroboter_anleitung.md](70_echtroboter_anleitung.md).

## Notaus

**EMERGENCY STOP** versetzt die Beine sofort in `Damp()` — alle Motoren
werden weich, der Roboter sackt kontrolliert zusammen. Für sanftes Anhalten
im laufenden Betrieb stattdessen **START BALANCING** drücken (der Roboter
bleibt stehen). Details und Sicherheitsablauf für echte Hardware:
[70_echtroboter_anleitung.md](70_echtroboter_anleitung.md).

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| WALK-Klick ohne Wirkung | Erst START BALANCING, dann WALK — WALK wird sonst mit Warnung ignoriert. |
| Roboter fällt beim Eintritt in BALANCE | (Sim) Prüfen, ob `USE_JOYSTICK=0` gesetzt ist, sofern kein Gamepad im Container hängt. |
| Roboter reagiert nicht auf den Joystick | Ist WALK aktiv (nicht nur BALANCING)? Kommt der `loco_cmd_vel`-Stream regelmäßig an (Deadman-Timeout)? |
| Arme hängen seltsam beim Loslaufen | Normal für kurze Zeit: die Arme werden erst in eine definierte Lauf-Pose gebracht, bevor der Lauf-Regler startet. |
