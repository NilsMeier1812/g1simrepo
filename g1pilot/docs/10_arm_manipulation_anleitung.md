# Arm-Manipulation — Anleitung

Richtet sich an: Anwender, die die Arme des G1 über RViz oder den Streamdeck
bedienen wollen. Für Entwickler siehe
[11_arm_manipulation_technik.md](11_arm_manipulation_technik.md). Für die
HTTP-Schnittstelle für externe Projekte siehe
[20_arm_api_anleitung.md](20_arm_api_anleitung.md).

## Überblick

Die Arme des G1 (je 7 Freiheitsgrade: Schulter Pitch/Roll/Yaw, Ellbogen,
Handgelenk Roll/Pitch/Yaw) werden unabhängig von den Beinen gesteuert. Die
Zielpose kommt entweder von einem interaktiven Marker in RViz oder aus dem
Positionsspeicher; ein inverser Kinematik-Löser (IK) rechnet daraus
Gelenkwinkel, ein Regler fährt sie sicher an.

## Voraussetzungen

- Stack läuft (Simulation oder echter Roboter, siehe
  [01_installation.md](01_installation.md)).
- RViz ist gestartet (Sim: optional über `USE_RVIZ`; Real: standardmäßig an).

## Bedienung

**1. Manipulation aktivieren**

Am Streamdeck: Button **ENABLE MANIPULATION**. Alternativ per Kommandozeile:

```bash
ros2 topic pub --once /g1pilot/arms/enabled std_msgs/msg/Bool "{data: true}"
```

Die Arme übernehmen dabei sanft ihre aktuelle Stellung (kein Sprung).

**2. Arme bewegen**

In RViz erscheint je ein würfelförmiger interaktiver Marker pro Hand. Marker
mit der Maus ziehen — der Arm folgt in Echtzeit (kartesisches Ziel, per IK
gelöst). Alternativ direkt eine `PoseStamped` veröffentlichen:

```bash
ros2 topic pub -1 /g1pilot/hand_goal/left geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'pelvis'}, pose: {position: {x: 0.20, y: 0.17, z: 0.09}, orientation: {w: 1.0}}}"
```

Solange ein Marker nicht gezogen wird, folgt er standardmäßig der Hand
(Leader-Follower) — nach einer fremden Bewegung (Homing, Positionsspeicher)
"springt" er also nicht, sondern schleicht sich unauffällig nach.

**3. Home-Position**

Button **HOMING ARMS** fährt beide Arme in eine definierte Ruhepose (nur bei
aktiver Manipulation).

**4. Positionsspeicher — Posen sichern und wieder anfahren**

Der Positionsspeicher merkt sich beliebige Kombinationen aus linkem Arm,
rechtem Arm, linker Hand, rechter Hand unter einem Namen, gruppiert in
Kategorien ("Ordner").

- **POSE SPEICHERN** öffnet einen Dialog: Name, Kategorie (mit Button
  „Kategorie anlegen"), und vier Häkchen für die zu speichernden
  Komponenten. Gespeichert wird die zuletzt *kommandierte* Stellung (nicht
  die verrauschte Messung).
- **POSE ANFAHREN** zeigt alle gespeicherten Posen gruppiert nach Kategorie,
  inklusive der enthaltenen Komponenten. Beim Anfahren wird die Bahn dorthin
  **jedes Mal neu geplant** — kollisionsfrei um Hindernisse und den eigenen
  Körper, da sich Startpose und Umgebung seit dem Speichern geändert haben
  können.
- **POSE ABBRECHEN** bricht eine laufende geplante Bewegung ab. Das passiert
  auch automatisch, sobald ein Marker manuell angefasst wird — manuelle
  Eingabe hat immer Vorrang.

Über die Kommandozeile (Speichern-Payload als JSON):

```bash
ros2 topic pub --once /g1pilot/pose_store/save std_msgs/msg/String \
  "{data: '{\"name\": \"Regal oben\", \"category\": \"Greifen\", \"components\": [\"arms\", \"hands\"]}'}"
ros2 topic pub --once /g1pilot/pose_store/goto std_msgs/msg/String "{data: 'Regal oben'}"
```

Die Datei liegt persistent im Repository unter `data/arm_poses.json` (im
Container gebunden gemountet) und übersteht damit Container-Neustarts.

**5. Hände**

`OPEN LEFT/RIGHT HAND` und `CLOSE LEFT/RIGHT HAND` öffnen/schließen die
Inspire-FTP-Hände, sofern eingerichtet — siehe
[60_inspire_haende_anleitung.md](60_inspire_haende_anleitung.md).

## Sicherheitsverhalten (was der Anwender wissen sollte)

- **Kollisions-Gate:** Eine kommandierte Zielpose, die den Arm in den
  eigenen Körper oder ein bekanntes Hindernis fahren würde, wird
  zurückgehalten — der Arm bleibt stehen, statt hineinzufahren. Marker
  einfach zurückziehen.
- **Geschwindigkeitslimits:** Kartesisch 0,25 m/s an Hand und Ellbogen
  (Industriewert für Betrieb mit Personen im Schutzraum), zusätzlich
  1,5 rad/s je Gelenk.
- **Notaus:** `EMERGENCY STOP` macht die Arme sofort drehmomentfrei
  (gedämpft, kein Sprung in eine Home-Pose) — Details in
  [70_echtroboter_anleitung.md](70_echtroboter_anleitung.md).
- **Marker hat immer Vorrang:** Wird während einer laufenden geplanten
  Bewegung (Positionsspeicher) ein Marker angefasst, bricht die Planung
  sofort ab.

## Konfiguration

Die wichtigsten Parameter (per `--ros-args -p <name>:=<wert>` beim Start
oder live via `ros2 param set /arm_controller <name> <wert>`):

| Parameter | Default (Sim / Real) | Bedeutung |
|---|---|---|
| `arm_velocity_limit` | 1.5 rad/s | Gelenk-Geschwindigkeitslimit |
| `ee_velocity_limit` | 0.25 m/s | Kartesisches Limit an Hand + Ellbogen |
| `self_collision_gate` | an | Selbstkollisionsprüfung vor jedem Kommando |
| `environment_collision_gate` | an | Prüfung gegen Hindernisse/Greif-Objekte |
| `kp_low` / `kd_low` | 150/12 (Sim) · 60/1.5 (Real) | PD-Gains der Armgelenke |
| `arm_weight_ramp_up_s` / `_down_s` | 0.0 (Sim) · 2.0 s (Real) | Übergangsrampe beim (De-)Aktivieren |

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| Arm bewegt sich nicht | Manipulation aktiviert? (`ENABLE MANIPULATION`) |
| Arm hält vor einem Hindernis an | Kollisions-Gate hat gegriffen — Marker zurückziehen; siehe Log „Kollisions-Gate: ...". |
| Marker springt nach Homing | Erwartetes Verhalten: nach Homing/Positionsspeicher wird die IK auf die neue Ist-Pose ausgerichtet. |
| Pose lässt sich nicht speichern | Mindestens eine Komponente auswählen; bei Händen muss die Inspire-Bridge laufen (sonst „skipped"). |
| Positionsspeicher-Datei weg nach Neustart | Läuft der Container ohne Bind-Mount des Repos? Prüfen mit `G1_POSE_STORE` bzw. Log-Zeile „Positionsspeicher: ...". |
