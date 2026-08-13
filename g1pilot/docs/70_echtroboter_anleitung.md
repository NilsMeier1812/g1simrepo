# Echter Roboter — Sicherheit & Ablauf

Richtet sich an: jeden, der den Stack zum ersten Mal (oder nach einer
Software-Änderung) gegen den echten Unitree G1 startet. Dieses Dokument ist
sicherheitsrelevant — vor dem ersten Realbetrieb vollständig lesen.

Architektur in Kürze: Unterkörper = Unitrees Onboard-Loco-Controller
(`loco_client`), Oberkörper/Arme/Hände = dieser Stack. Details in
[02_architektur.md](02_architektur.md),
[31_loco_technik.md](31_loco_technik.md) und
[11_arm_manipulation_technik.md](11_arm_manipulation_technik.md).

## A · Vor dem Einschalten

- [ ] Gantry/Traggurt für die ersten Balance-/Geh-Tests, oder mindestens
      2 m freie Sturzfläche in alle Richtungen.
- [ ] Eine zweite Person hält die **Unitree-Fernbedienung** bereit — das ist
      immer der übergeordnete Notaus, über allem anderen.
- [ ] E-Stop-Semantik verstanden (siehe Abschnitt „Notaus" unten).
- [ ] Akku geladen, keine Kabel/Hindernisse in Fußnähe.
- [ ] Erste Arm-Tests am hängenden/knienden Roboter, nicht am stehenden.

## B · Netzwerk (Ersteinrichtung)

Voraussetzung: nativer Linux-PC mit freiem Ethernet-Port. **Kein WSL2** für
den echten Roboter — das SDK braucht die physische Netzwerkkarte direkt, das
funktioniert durch die WSL2-NAT nicht zuverlässig (die Simulation unter
WSL2 ist hiervon nicht betroffen).

| Adresse | Wer |
|---|---|
| `192.168.123.161` | G1-Onboard-PC (DDS-Quelle) |
| `192.168.123.210` / `.211` | Inspire-Hand links/rechts (Modbus TCP 6000) |
| `192.168.123.222` | eigener PC (selbst vergeben, Konvention) |

1. G1 einschalten, per Ethernet-Kabel mit dem PC verbinden.
2. Interface-Namen finden: `ip -br link` (z. B. `enp3s0`, `eth0`; `lo`/WLAN
   ignorieren).
3. Statische IP vergeben:
   ```bash
   sudo ip addr add 192.168.123.222/24 dev <NIC>
   sudo ip link set <NIC> up
   ```
   Dauerhaft über NetworkManager (kein Gateway/DNS eintragen, sonst
   versucht der PC, über das Roboter-LAN ins Internet zu gehen):
   ```bash
   sudo nmcli con add type ethernet ifname <NIC> con-name g1-robot \
        ipv4.method manual ipv4.addresses 192.168.123.222/24
   sudo nmcli con up g1-robot
   ```
4. Erreichbarkeit prüfen:
   ```bash
   ping -c3 192.168.123.161
   ping -c3 192.168.123.210   # nur mit Inspire-Händen
   nc -zv 192.168.123.210 6000
   ```
   Ohne Ping zum Roboter hat nichts Weiteres Sinn — Kabel/IP/Interface
   prüfen, bevor es weitergeht.
5. Firewall beachten: `sudo ufw status` — falls aktiv, den DDS-UDP-Verkehr
   erlauben (`sudo ufw allow in on <NIC>`) oder deaktivieren.

Die Interface-Auswahl im Startmenü markiert die NIC mit
`192.168.123.x`-Adresse automatisch als Vorschlag.

## C · Stack starten

```bash
cd g1pilot && ./start.sh
# -> "ECHTER ROBOTER" wählen
# -> Interface bestätigen, Hände ja/nein + IPs, RViz an, ggf. neu bauen
# -> Sicherheitsabfrage "REAL" eintippen
```

Erwartung: Container startet, Build läuft durch, RViz und der Streamdeck
(Titel „— REAL") öffnen sich. **Der Roboter tut nichts von selbst.**

## D · DDS-Verbindung prüfen (bevor irgendetwas kommandiert wird)

```bash
ros2 run g1pilot dds_check --interface $ROBOT_INTERFACE
```

Erwartung: `rt/lowstate` kommt mit ~500 Hz, `mode_machine` ≠ 0, die
Arm-Gelenkwinkel ändern sich plausibel, wenn die Arme von Hand bewegt
werden. Ohne `lowstate`: falsches Interface, Roboter nicht im
192.168.123.x-Netz, oder Kabelproblem — nicht weitermachen.

## E · Guarantees, die bereits im Code stehen

- **Kein Auto-Damp beim Start:** `loco_client` läuft mit `damp_on_init=False`
  — der Node greift beim Hochfahren nicht in den Roboterzustand ein.
- **Kein Auto-Start:** Der Streamdeck publiziert im Real-Modus von sich aus
  nichts — der Roboter bewegt sich erst nach Klicks (siehe
  [41_teleoperation_technik.md](41_teleoperation_technik.md), Abschnitt
  Auto-Start).
- **WALK ist gesperrt, bis balanciert:** wird sonst mit Warnung ignoriert.
- **Weiche Arm-Übergabe:** das `arm_sdk`-Gewicht rampt beim `ENABLE` 0→1 und
  beim `DISABLE` 1→0 über 2,0 s.

## F · Ablauf der ersten Tests

**1. Hände (Roboter bleibt passiv, hängend/schlaff)**

Hand-GUIs öffnen, Hauptschalter der linken Hand aktivieren, per
Streamdeck/GUI öffnen/schließen. Erwartung: RViz spiegelt die Finger, die
Viewer-GUI zeigt plausible Kräfte bei Druck.

**2. Arme passiv beobachten**

Arme von Hand bewegen (Manipulation nicht aktiviert). RViz-Modell folgt
exakt; die Marker folgen der Hand (Leader-Follower).

**3. Arm-Manipulation (hängend oder kniend)**

1. **ENABLE MANIPULATION** — Gewicht rampt 2 s von 0 auf 1; die Arme
   übernehmen die aktuelle Pose ohne Ruck.
2. In RViz einen Marker wenige Zentimeter ziehen — Hand folgt ruhig,
   max. 0,25 m/s.
3. **Gate-Test:** Marker absichtlich in den Torso ziehen. Erwartung: Arm
   stoppt vor dem Körper (Log: „Selbstkollisions-Gate ..."). Marker
   zurückziehen — Arm folgt wieder.
4. **HOMING ARMS** testen (nur bei aktiver Manipulation möglich).
5. **E-Stop-Test (hängend!):** EMERGENCY STOP drücken. Erwartung: Arme
   werden **sofort** drehmomentfrei und sacken gedämpft — kein Sprung in
   eine Home-Pose. Danach START (quittiert den Latch, Arme bleiben
   schlaff) → ENABLE MANIPULATION übernimmt wieder ruckfrei die aktuelle
   Pose. Fährt der Arm beim E-Stop stattdessen aktiv irgendwohin, sofort
   die Fernbedienungs-Notabschaltung nutzen und den Vorfall melden.
6. **ENABLE MANIPULATION** wieder aus — Gewicht rampt 2 s auf 0, die
   Roboter-eigene Armsteuerung übernimmt weich.

Regelrate-Hinweis: Während ein Marker-Ziel aktiv verfolgt wird, kostet der
IK-Solve ~20-30 ms/Zyklus — der Regel-Loop läuft dann effektiv mit
~30-50 Hz statt 250 Hz. Alle Limits rechnen mit dem echten Zeitschritt und
bleiben korrekt.

**4. Balance**

Roboter gesichert am Boden (Gantry). **START** → FSM 4 (Standby). **START
BALANCING** → Höhenrampe + Balance-Start. Erwartung: Roboter steht
selbstständig und balanciert; leichtes Anstupsen wird ausgeglichen.

Firmware-Vorbehalt: Die FSM-ID-Sequenz stammt aus dem ursprünglichen
g1pilot-Projekt. Bei neuerer G1-Firmware ggf. andere FSM-IDs — dann hier
abbrechen und die IDs in `loco_client.py` gegen die aktuelle Unitree-Doku
prüfen.

**5. Gehen**

Zuerst **PS4-Pfad** (physischer Deadman), dann Streamdeck:

1. Controller verbinden (Gerätename passt zu `JOYSTICK_NAME`, Log „Joystick
   found"). Deadman (Button 8) halten + Stick bewegt den Roboter; loslassen
   stoppt.
2. Streamdeck **WALK** klicken (nur nach START BALANCING möglich), dann den
   virtuellen Joystick vorsichtig ziehen.
3. **Deadman-Test (Pflicht):** Während der Roboter geht, das
   Streamdeck-Fenster schließen. Erwartung: Roboter stoppt von selbst
   innerhalb ≤ 0,5 s und bleibt balanciert stehen. Läuft er weiter, ist das
   ein Fehler — sofort melden.
4. Geh-Limits konservativ lassen (Default vx=0.4/vy=0.3/vyaw=0.4 m·rad/s,
   über `G1_MAX_VX/VY/VYAW` anpassbar).

## Notaus-Reflex (auswendig lernen)

| Situation | Richtige Reaktion |
|---|---|
| Sanft anhalten beim Gehen | **START BALANCING** (Roboter bleibt stehen) |
| Ruckt/schwingt an den Armen | **ENABLE MANIPULATION aus** (Gewicht rampt 2 s runter) |
| Echte Gefahr / Sturz droht | **Unitree-Fernbedienung** (übergeordnet) — Streamdeck-EMERGENCY nur, wenn Zusammensacken vertretbar ist |

Was EMERGENCY STOP konkret bewirkt: Beine gehen in `Damp()` (Roboter sackt
kontrolliert zusammen); Arme werden drehmomentfrei und sacken gedämpft, ohne
in eine Home-Pose zu springen (dazu bleibt das `arm_sdk`-Gewicht bewusst auf
1 — würde es auf 0 gesetzt, übernähme der Onboard-Regler die Arme und führe
sie aktiv in seine Default-Pose); Hände werden deaktiviert und halten ihre
Position. Quittieren: **START**. Die Arme bleiben danach gedämpft schlaff,
bis **ENABLE MANIPULATION** die Kontrolle bewusst zurückholt.

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| `dds_check`: kein `lowstate` | Falsches `ROBOT_INTERFACE`; PC nicht im 192.168.123.x-Netz; Kabel. |
| „create domain error" im Log | `ROS_DOMAIN_ID` == Unitree-Domain (muss auf real 1 vs. 0 sein — Compose setzt das automatisch; eigene Terminals ebenfalls auf `ROS_DOMAIN_ID=1` setzen). |
| Hand „NICHT VERBUNDEN" | IP/Port prüfen (`nc -zv <ip> 6000`); Hand mit Strom versorgt? |
| „No joystick found" | `/dev/input` gemountet (Compose-Profil `real` braucht `privileged: true`)? Gerätename weicht ab → `JOYSTICK_NAME` setzen. Geräte anzeigen: `python3 -c "import evdev;[print(p, evdev.InputDevice(p).name) for p in evdev.list_devices()]"` |
| WALK-Klick ohne Wirkung | Erst START → START BALANCING. |
| Arme ruckeln beim Enable | Rampe verkürzt/0? `arm_weight_ramp_up_s` prüfen (Default 2.0 auf real). |
| Roboter geht nach GUI-Absturz weiter | Darf nicht passieren (Deadman) — Log von `loco_client` prüfen, melden. |
| RViz leer | `USE_RVIZ=true`? X11-Freigabe (`xhost +local:root`) gelaufen? |

## Weiterführend

`G1_ENABLE_LIDAR=1` (+ Compose-Profil `real-full`) schaltet Livox/Navigation
zu — braucht das große Image und den MID360; siehe
[50_navigation_anleitung.md](50_navigation_anleitung.md). `arm_sdk` parallel
zum Gehen funktioniert konstruktionsbedingt (Gewichts-Überblendung), für die
ersten Geh-Tests aber die Arme deaktiviert lassen — der natürliche Armschwung
der Unitree-Steuerung ist dafür besser geeignet.
