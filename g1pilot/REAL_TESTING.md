# REAL_TESTING.md — Erster Hardware-Test am echten G1

Schritt-fuer-Schritt-Runbook fuer die erste Inbetriebnahme dieses Stacks am
echten G1 (Arme + Inspire-Haende + eingebauter Unitree-Loco-Controller).
**Die Stufen bauen aufeinander auf — in dieser Reihenfolge testen und erst
weitergehen, wenn die Erwartung der Stufe erfuellt ist.**

---

## 0. Sicherheits-Checkliste (VOR dem ersten Start)

- [ ] **Roboter gesichert**: Gantry/Traggurt fuer die ersten Balance-/Geh-Tests,
      oder mindestens 2 m freie Sturzflaeche in alle Richtungen.
- [ ] **Zweite Person** anwesend, die die Unitree-Fernbedienung haelt
      (die Fernbedienung ist IMMER der uebergeordnete Not-Aus).
- [ ] **E-Stop-Semantik verstanden**: Der Streamdeck-EMERGENCY-STOP stoppt
      ALLES sofort: `Damp()` an den Loco-Controller, der arm_controller
      sendet EINE Schlaff-Nachricht (arm_sdk-Weight=0 **und** kp/kd/tau=0)
      und verstummt, die Inspire-Haende werden disabled (halten Position).
      → **Der Roboter sackt zusammen** (haengend: Seile fangen ihn).
      Das ist die letzte Instanz, nicht der "Anhalten"-Knopf. Sanftes
      Anhalten beim Gehen = **START BALANCING** klicken (StopMove, Roboter
      bleibt stehen und balanciert).
      **Quittieren nach E-Stop:** **START** klicken (hebt den Latch in
      loco_client UND arm_controller auf), danach bei Bedarf ENABLE
      MANIPULATION neu druecken.
- [ ] **Speedlimits aktiv**: Armbewegungen sind auf **0.25 m/s** an Hand-TCP
      und Ellbogen begrenzt (ISO 10218-1 / ISO TS 15066 "reduced speed")
      plus 1.5 rad/s je Gelenk. Auch wenn der IK-Solver eine wirre
      Konfiguration vorschlaegt, faehrt der Arm sie nur mit diesem Tempo an.
      Anpassbar via `G1_ARM_VEL_LIMIT` / `G1_EE_VEL_LIMIT` (Env vor start.sh).
- [ ] **Selbstkollisions-Gate aktiv**: jede kommandierte Armpose wird vor dem
      Senden gegen Selbstkollision geprueft (3 cm Marge, Convex-Hulls,
      Arm↔Torso/Kopf/Hueften + Arm↔Arm). Bei Verletzung haelt der Arm an und
      loggt eine Warnung -- Marker einfach zurueckziehen.
- [ ] **Kein Auto-Start**: Im Real-Modus aktiviert der Streamdeck NICHTS von
      selbst. Der Roboter bewegt sich erst nach euren Klicks.
- [ ] Akku geladen, Umgebung frei von Hindernissen/Kabeln in Fussnaehe.
- [ ] Erste Arm-Tests am haengenden/knienden Roboter, nicht am stehenden.

---

## 1. Netzwerk (Ersteinrichtung — auch ohne G1-Vorerfahrung)

**Vorweg: "Ports" muss man NICHT konfigurieren.** Der G1 spannt sein eigenes
LAN auf (`192.168.123.0/24`); DDS laeuft als UDP direkt auf dem Interface
(bindet das Unitree-SDK selbst, sobald start.sh das Interface kennt), Modbus
zu den Haenden ist ausgehend (TCP 6000), die Hand-GUIs (8765-8767) sind rein
lokal. Einzige Pflicht: dem PC-Ethernet-Port eine **statische IP** im
Roboter-Subnetz geben — im Roboter-LAN gibt es keinen DHCP-Server.

| Adresse            | Wer                                          |
|--------------------|----------------------------------------------|
| `192.168.123.161`  | G1 Onboard-PC (DDS-Quelle)                   |
| `192.168.123.210/.211` | Inspire-Hand links/rechts (Modbus TCP 6000) |
| `192.168.123.222`  | dein PC (selbst vergeben, Konvention)        |

**Voraussetzung:** nativer Linux-PC mit freiem Ethernet-Port. KEIN WSL2 fuer
den echten Roboter — das SDK braucht die physische NIC direkt, das geht
durch die WSL2-NAT nicht zuverlaessig (Sim unter WSL2 ist ok).

1. G1 einschalten und per **Ethernet-Kabel** mit dem PC verbinden.
2. Interface-Namen finden (wechselt beim Einstecken auf UP):
   ```bash
   ip -br link        # z.B. enp3s0, eth0, enx... (lo/wlp... ignorieren)
   ```
3. Statische IP vergeben (fluechtig, nach Reboot weg):
   ```bash
   sudo ip addr add 192.168.123.222/24 dev <NIC>
   sudo ip link set <NIC> up
   ```
   Dauerhaft via NetworkManager (KEIN Gateway/DNS eintragen, sonst will der
   PC uebers Roboter-LAN ins Internet):
   ```bash
   sudo nmcli con add type ethernet ifname <NIC> con-name g1-robot \
        ipv4.method manual ipv4.addresses 192.168.123.222/24
   sudo nmcli con up g1-robot
   ```
4. Erreichbarkeit pruefen:
   ```bash
   ping -c3 192.168.123.161     # G1 Onboard-PC (Standard-Adresse)
   # Mit Inspire-Haenden zusaetzlich:
   ping -c3 192.168.123.210     # linke Hand
   ping -c3 192.168.123.211     # rechte Hand
   nc -zv 192.168.123.210 6000  # Modbus-Port offen?
   ```
   **Abbruch-Kriterium:** Ohne Ping zum Roboter hat nichts Weiteres Sinn —
   Kabel/IP/Interface pruefen.
5. Firewall beachten: `sudo ufw status` — falls aktiv, blockt sie den
   DDS-UDP-Traffic → `sudo ufw allow in on <NIC>` (oder deaktivieren).

WLAN darf parallel anbleiben (anderes Subnetz, stoert nicht). Die
Interface-Auswahl in start.sh markiert die NIC mit der 192.168.123.x-IP
automatisch als Default — Schritt 3 ist also die Voraussetzung dafuer,
dass dort einfach ENTER reicht.

---

## 2. Stack starten (Real-Modus)

```bash
cd g1pilot && ./start.sh
# -> Frage 0: "ECHTER ROBOTER" waehlen
# -> Interface auswaehlen (das mit der 192.168.123.x-IP ist vormarkiert)
# -> Haende ja/nein + IPs bestaetigen, RViz an, ggf. --build
# -> Sicherheitsabfrage: "REAL" tippen
```

Beim ersten Mal Rebuild waehlen (das schlanke Real-Image `g1pilot-real:v1.2.0`
wird gebaut). Erwartung: Container startet, colcon-Build laeuft durch, RViz
und der Streamdeck (Titel **"— REAL"**) gehen auf. Der Roboter tut NICHTS.

---

## 3. DDS-Verbindung pruefen (bevor irgendetwas kommandiert wird)

Im laufenden Container:
```bash
docker exec -it g1pilot-g1pilot-real-1 bash
ros2 run g1pilot dds_check --interface $ROBOT_INTERFACE
```
**Erwartung:** `rt/lowstate` kommt mit **~500 Hz**, `mode_machine` != 0,
die Arm-Gelenkwinkel aendern sich plausibel, wenn man die Arme von Hand bewegt.

ROS-Topics anschauen (im Container ist alles gesetzt; von einem Host-Terminal
aus braucht es `ROS_DOMAIN_ID=1` + CycloneDDS):
```bash
ros2 topic hz /joint_states       # ~ Publisher-Rate von robot_state
ros2 topic echo /g1pilot/imu --once
```
**Abbruch-Kriterium:** Kein lowstate → falsches Interface oder Roboter nicht
im Netz. NICHT weitermachen.

---

## 4. Haende-Test (Roboter passiv!)

Roboter bleibt in Damp/haengend, Arme schlaff.

1. Hand-GUIs oeffnen (Auto-Open oder `http://localhost:8767/hand_controller_viewer.html?autoconnect=1`).
2. **Erwartung GUI:** Status "VERBUNDEN", Host zeigt die echten IPs, Ist-Winkel
   folgen, wenn man Finger vorsichtig von Hand bewegt.
3. Hauptschalter (enabled) der linken Hand an → **OPEN/CLOSE LEFT HAND** am
   Streamdeck bzw. Slider in der GUI. Finger bewegen sich; RViz spiegelt sie.
4. Viewer-GUI: Taktil-Zonen reagieren auf Druck auf die Sensorflaechen,
   Kraefte (Gramm) plausibel.

**Troubleshooting:** "NICHT VERBUNDEN" → IP/Port pruefen (`nc -zv <ip> 6000`),
Log der Bridge lesen (`Backend=modbus ... left=...`).

---

## 5. Arme passiv beobachten

Arme von Hand bewegen (Manipulation NICHT enabled). **Erwartung:** RViz-Modell
folgt exakt; die IK-Marker folgen den Haenden (marker_follow_ee).

---

## 6. Arm-Manipulation (Roboter haengend oder kniend)

1. Roboter per Unitree-Fernbedienung in den normalen Stand-/Damp-Zustand
   bringen (fuer den ersten Test: haengend).
2. Streamdeck **ENABLE MANIPULATION** → das arm_sdk-Gewicht rampt **2 s**
   von 0 auf 1. **Erwartung:** Arme uebernehmen die aktuelle Pose OHNE Ruck
   und halten sie (Schwerkraft-Feedforward aktiv). Beim Enable werden alte
   IK-Ziele/Homing-Reste verworfen -- der Arm HAELT immer erst die
   aktuelle Stellung.
3. In RViz einen Marker **wenige cm** ziehen. Erwartung: Hand folgt ruhig
   (max. 0.25 m/s), haelt die Zielposition stabil.
4. **Gate-Test:** Marker absichtlich IN den Torso ziehen. Erwartung: Arm
   stoppt vor dem Koerper, Log zeigt "Selbstkollisions-Gate: ... Arm haelt
   an". Marker zurueckziehen → Arm folgt wieder.
5. **HOMING ARMS** testen → definierte Home-Pose (geht nur bei aktiver
   Manipulation).
6. **E-Stop-Test (haengend!):** EMERGENCY STOP druecken → Arme werden
   SOFORT schlaff (keine 2-s-Rampe). Danach START → ENABLE MANIPULATION:
   Arme uebernehmen wieder ruckfrei die aktuelle Pose.
7. **ENABLE MANIPULATION** wieder aus → Gewicht rampt 2 s auf 0, die
   Roboter-eigene Armsteuerung uebernimmt weich.

**Abbruch:** Ruckt oder schwingt etwas → sofort disable; kp/kd im
arm_controller pruefen, bevor es weitergeht (Real-Bringup setzt die
Unitree-Beispielwerte kp=60/kd=1.5; die Sim faehrt bewusst andere Gains).

**Hinweis Regelrate:** Waehrend ein Marker-Ziel AKTIV verfolgt wird, kostet
der IK-Solve ~20-30 ms/Zyklus -> der Regel-Loop laeuft dann effektiv mit
~30-50 Hz statt 250 Hz. Alle Limits rechnen mit dem echten dt und bleiben
korrekt; im Halten-Zustand laeuft der Loop mit voller Rate.

---

## 7. Balance (Unitree-Loco-Controller)

Roboter steht gesichert am Boden (Gantry!).

1. Streamdeck **START** → FSM 4 (Standby). Log von loco_client beobachten.
2. **START BALANCING** → StandHeight-Rampe + `BalanceStand(1)` + `Start()`.
   **Erwartung:** Roboter steht selbststaendig und balanciert.
3. Verhalten pruefen: leichtes Anstupsen wird ausgeglichen.

**Hinweis:** Die FSM-Sequenz stammt aus dem originalen g1pilot — falls eine
neuere G1-Firmware andere FSM-IDs nutzt, hier abbrechen und die IDs in
`loco_client.py` gegen die aktuelle Unitree-Doku pruefen.

---

## 8. Gehen

Erst **PS4-Pfad** (physischer Deadman!), dann Streamdeck.

1. **PS4:** Controller verbinden (Bluetooth/USB; Gerätename muss zu
   `JOYSTICK_NAME` passen, Default "Wireless Controller"; Log des joystick-
   Nodes pruefen: "Joystick found"). Button 8 (Deadman) HALTEN + linker
   Stick → Roboter geht; loslassen → StopMove.
2. **Streamdeck:** **WALK** klicken (geht nur nach START BALANCING), dann
   den virtuellen Joystick vorsichtig ziehen. Limits sind konservativ
   (0.4/0.3/0.4 — via `G1_MAX_VX/VY/VYAW` anpassbar).
3. **Deadman-Test (wichtig!):** Waehrend der Roboter geht, das Streamdeck-
   Fenster schliessen. **Erwartung:** Roboter stoppt von selbst nach
   ≤ 0.5 s (`cmd_vel_timeout`) und bleibt balanciert stehen.
4. Anhalten im Normalfall: Joystick zentrieren ODER **START BALANCING**.

---

## 9. Troubleshooting

| Symptom | Wahrscheinliche Ursache / Fix |
|---------|-------------------------------|
| `dds_check`: kein lowstate | Falsches `ROBOT_INTERFACE`; PC nicht im 192.168.123.x-Netz; Kabel. |
| "create domain error" im Log | `ROS_DOMAIN_ID` == Unitree-Domain (muss auf real 1 vs. 0 sein — compose setzt das; eigene Terminals ebenfalls auf `ROS_DOMAIN_ID=1`). |
| Hand "NICHT VERBUNDEN" | IP/Port pruefen (`nc -zv <ip> 6000`); Hand am G1 mit Strom versorgt? |
| "No joystick found" | `/dev/input` nicht gemountet (compose: privileged + Mount vorhanden); Geraetename weicht ab → `JOYSTICK_NAME` setzen (USB-PS4 heisst teils "Sony Interactive ... Controller"). Geraete anzeigen: `python3 -c "import evdev;[print(p, evdev.InputDevice(p).name) for p in evdev.list_devices()]"` |
| WALK-Klick ohne Wirkung | Erst START → START BALANCING (WALK wird sonst mit Warnung ignoriert). |
| Arme ruckeln beim Enable | Rampe verkuerzt/0? `arm_weight_ramp_up_s` pruefen (Default 2.0 auf real). |
| Roboter geht nach GUI-Absturz weiter | Darf nicht passieren (Deadman) — Log von loco_client pruefen (`StopMove (Deadman-Timeout)`), Issue melden. |
| RViz leer | `USE_RVIZ=true`? X11-Freigabe (`xhost +local:root`, macht start.sh). |

## Bekannte Punkte fuer spaeter

- `G1_ENABLE_LIDAR=1` (+ Profil `real-full`) schaltet Livox/Nav zu — braucht
  das grosse Image und den MID360.
- arm_sdk parallel zum Gehen: funktioniert konstruktionsbedingt (Weight-Blend),
  aber fuer die ersten Geh-Tests die Arme disabled lassen (natuerlicher
  Armschwung der Unitree-Steuerung).
