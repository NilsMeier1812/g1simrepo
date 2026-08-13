# Teleoperation — Anleitung

Richtet sich an: Anwender, die den G1 über die grafische Bedienoberfläche
(„Streamdeck") oder einen physischen Controller steuern. Für die interne
Funktionsweise siehe [41_teleoperation_technik.md](41_teleoperation_technik.md).

## Überblick

Die Hauptbedienoberfläche ist ein virtuelles Tastenfeld (Streamdeck-Fenster),
das mit dem Start des Stacks automatisch aufgeht. Zusätzlich lässt sich der
Roboter über einen physischen Gamepad (PS4-kompatibel, per Bluetooth/USB)
bedienen — hauptsächlich für Gehen und Notaus.

## Der Streamdeck

Der Streamdeck ist ein 5×5-Tastenraster. Aktive Buttons leuchten grün.

| Bereich | Buttons | Wirkung |
|---|---|---|
| Locomotion | START, START BALANCING, WALK | siehe [30_loco_anleitung.md](30_loco_anleitung.md) |
| Arme | ENABLE MANIPULATION, HOMING ARMS | siehe [10_arm_manipulation_anleitung.md](10_arm_manipulation_anleitung.md) |
| Hände | OPEN/CLOSE LEFT/RIGHT HAND, INSPIRE FTP GUIs | siehe [60_inspire_haende_anleitung.md](60_inspire_haende_anleitung.md) |
| Positionsspeicher | POSE SPEICHERN, POSE ANFAHREN, POSE ABBRECHEN | siehe [10_arm_manipulation_anleitung.md](10_arm_manipulation_anleitung.md) |
| Sonstiges | MARKER FOLLOW, AUTO NAV (nur mit Navigation), PUSH ROBOT/GRASP BOX (nur Sim) | — |
| Notaus | EMERGENCY STOP (unten rechts, rot) | sofortiger Stopp, siehe unten |

Unterhalb des Tastenrasters befindet sich ein Bildschirm-Joystick (Ziehen mit
der Maus = vorwärts/seitwärts) und ein Dreh-Regler (Yaw). Beide wirken nur im
Zustand WALK; Loslassen zentriert automatisch auf 0 (Stopp).

**MARKER FOLLOW** (Standard: an) lässt die RViz-Zielmarker der Arme der Hand
folgen, solange sie nicht gerade gezogen werden — sie „springen" also nach
externen Bewegungen (Homing, Positionsspeicher) nicht, sondern gleiten
unauffällig hinterher.

**In der Simulation startet der Stack automatisch**: nach kurzer Anlaufzeit
werden die Arme aktiviert und der Roboter geht in den balancierten Stand.
**Auf dem echten Roboter passiert das absichtlich nicht** — dort bewegt sich
nichts, bevor der Bediener klickt.

## Physischer Controller (PS4-kompatibel)

Nur auf dem echten Roboter aktiv (Bluetooth/USB, Gerätename konfigurierbar
über `JOYSTICK_NAME`, Default „Wireless Controller"). Wichtige Zuordnungen:

| Eingabe | Wirkung |
|---|---|
| Button 8 (Deadman, halten) | Steuert; loslassen stoppt sofort |
| Linker Stick (bei gehaltenem Deadman) | vx/vy |
| Rechter Stick X (bei gehaltenem Deadman) | Drehen |
| Button 6 | START BALANCING |
| Button 5 | EMERGENCY STOP |
| Button 0 | Arm-Steuerung ein/aus |
| Button 1 | Arme homen |
| Button 3 + Achse 4 (rechts/links) | Greifer öffnen/schließen |
| Dreieck (Triangle) | Autonome Navigation umschalten |

Der Deadman-Button ist Pflicht: Wird er losgelassen, stoppt der Roboter
innerhalb von rund 0,5 Sekunden von selbst, auch wenn die Software gerade
abstürzt oder das Fenster geschlossen wird.

## Notaus

**EMERGENCY STOP** stoppt alles sofort: Beine gehen in `Damp()` (weiche
Motoren, Roboter sackt zusammen), Arme werden drehmomentfrei und sacken
gedämpft (kein aktiver Sprung in eine Home-Pose), Hände werden deaktiviert
(halten ihre Position). Quittieren mit **START**; Arme bleiben danach
schlaff, bis **ENABLE MANIPULATION** die Kontrolle bewusst zurückholt.

Details und der vollständige Sicherheitsablauf für echte Hardware:
[70_echtroboter_anleitung.md](70_echtroboter_anleitung.md).

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| Kein Streamdeck-Fenster | X11-Display verfügbar? (`xhost`, siehe [01_installation.md](01_installation.md)) |
| Joystick reagiert nicht | Gerätename passt zu `JOYSTICK_NAME`? Log prüfen: „Joystick found". USB-Controller heißen teils anders als Bluetooth-Controller. |
| Roboter läuft nach GUI-Absturz weiter | Darf nicht passieren (Deadman) — melden, falls beobachtet. |
| AUTO NAV Button fehlt | Erscheint nur, wenn der Navigations-Stack läuft (`G1_ENABLE_NAV`/`G1_ENABLE_LIDAR`). |
