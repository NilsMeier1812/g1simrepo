# Inspire-FTP-Hände — Anleitung

Richtet sich an: Anwender, die die Inspire-RH56DFTP-2-Hände in Simulation
oder auf echter Hardware nutzen wollen. Für die interne Funktionsweise siehe
[61_inspire_haende_technik.md](61_inspire_haende_technik.md).

## Überblick

Jede Hand hat 6 steuerbare Freiheitsgrade (Kleiner, Ring-, Mittel-,
Zeigefinger, Daumen-Beugung, Daumen-Rotation) und liefert Kraft- sowie
Tastsinn-Rückmeldung. Zwei HTML-Oberflächen dienen der Bedienung:

- **Controller** (Port 8766) — Winkel/Kraft/Geschwindigkeit je Finger
  einstellen, Hand öffnen/schließen.
- **Viewer** (Port 8765) — Kraft- und Tastsinn-Heatmap anzeigen.

Beide GUIs sind in Simulation und auf echter Hardware identisch.

## Aktivieren

Beim Start über das grafische Startmenü: Häkchen „Inspire-FTP-Hände" setzen
(sonst wird das einfache Rubber-Hand-Modell ohne bewegliche Finger geladen).
Nicht-interaktiv:

```bash
G1_INSPIRE_HANDS=1 ./start.sh --yes
```

Auf echter Hardware zusätzlich die Modbus-IP-Adressen der Hände angeben
(Default `192.168.123.210`/`.211`, Port 6000).

## Bedienung

**Schnellzugriff über den Streamdeck:** `OPEN LEFT/RIGHT HAND`,
`CLOSE LEFT/RIGHT HAND`, `INSPIRE FTP GUIs` (öffnet beide Oberflächen im
Browser).

**GUIs manuell öffnen:**

```
http://localhost:8767/hand_controller_viewer.html?autoconnect=1
http://localhost:8767/inspire_hand_viewer.html?autoconnect=1
```

Beim Sim-/Real-Start kann „Hand-GUIs automatisch öffnen" aktiviert werden —
dann öffnen sich beide Seiten von selbst, sobald die Bridge bereit ist.

**Per Kommandozeile:**

```bash
ros2 topic pub --once /g1pilot/hand_action/right std_msgs/msg/String "{data: 'open'}"
ros2 topic pub --once /g1pilot/hand_action/right std_msgs/msg/String "{data: 'close'}"
```

**Handstellungen speichern:** Über den Positionsspeicher — siehe
[10_arm_manipulation_anleitung.md](10_arm_manipulation_anleitung.md).

## In der Simulation: greifbare Testkugel

Der Streamdeck-Button **GRASP BOX** legt eine kleine, fest mit der
Handfläche verbundene greifbare Kugel in die Griffzone. Beim Schließen der
Hand (etwa 70 %) greifen alle Finger und die Handfläche zu — sofort sichtbare
Griffkräfte in der Viewer-GUI. Standardmäßig aus; direkt beim Start aktiv
mit `G1_GRASP_TEST=1`.

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| GUI zeigt „NICHT VERBUNDEN" (real) | IP/Port prüfen (`nc -zv <ip> 6000`); Hand am G1 mit Strom versorgt? |
| GUI öffnet sich nicht automatisch | Läuft die Bridge schon (Port 8766 erreichbar)? Läuft `start.sh` auf dem Host (nicht in einem verschachtelten Container)? |
| Finger bewegen sich nicht in RViz | Ist die Inspire-Bridge aktiv (`G1_INSPIRE_HANDS=1`)? `robot_state` muss dann `publish_hand_joints:=false` laufen (passiert automatisch über die Launch-Datei). |
| Kraft/Taktil bleiben 0 (Simulation) | Backend `sim` (Stufe 1) hat keine Kontaktphysik — Default ist Stufe 2 (`mujoco`), die echte Kräfte liefert. |
