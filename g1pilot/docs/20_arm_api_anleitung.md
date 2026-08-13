# Arm-API — Anleitung

Richtet sich an: Anwender, die aus einem **eigenen, unabhängigen Projekt**
heraus Zielposen an den G1-Arm schicken wollen — ohne ROS 2, ohne den
gesamten g1pilot-Stack zu verstehen. Für die interne Funktionsweise siehe
[21_arm_api_technik.md](21_arm_api_technik.md).

## Überblick

`arm_api` ist eine kleine HTTP/JSON-Brücke. Sie läuft als eigener ROS-Node
neben `arm_controller` und übersetzt gewöhnliche HTTP-POST-Requests in
Arm-Kommandos. Nichts wird gespeichert — jedes Kommando wird sofort
ausgeführt (für Speichern siehe den `/arm/save`-Endpunkt oder direkt den
[Positionsspeicher](10_arm_manipulation_anleitung.md)). Es gelten exakt
dieselben Sicherheitsgates wie beim Streamdeck (E-Stop, ENABLE MANIPULATION,
Kollision, Limits) — die Brücke kann sie nicht umgehen.

Standardmäßig gebunden an `127.0.0.1:8770` — nur vom selben Rechner
erreichbar. Läuft der eigene Code auf demselben Host (auch außerhalb von
Docker, dank `network_mode: host`), reicht `http://localhost:8770`.

## Voraussetzungen

- Der g1pilot-Stack läuft (Simulation oder echter Roboter).
- Manipulation ist aktiviert (`ENABLE MANIPULATION`, siehe
  [10_arm_manipulation_anleitung.md](10_arm_manipulation_anleitung.md)) —
  ohne das lehnt die API jedes Bewegungskommando ab.

## Schnellstart mit dem mitgelieferten CLI

```bash
python3 g1pilot/examples/arm_api/arm_cli.py health
python3 g1pilot/examples/arm_api/arm_cli.py state
python3 g1pilot/examples/arm_api/arm_cli.py joints --right 0.3 -0.2 0.0 0.5 0.0 0.0 0.0 --wait 60
python3 g1pilot/examples/arm_api/arm_cli.py pose --right 0.35 -0.20 0.10 --rpy 0 0 0 --wait 60
python3 g1pilot/examples/arm_api/arm_cli.py save "Regal oben" --category Greifen --components arms hands
python3 g1pilot/examples/arm_api/arm_cli.py cancel
```

Das Skript nutzt nur die Python-Standardbibliothek (`urllib`) — es läuft auf
jedem fremden Rechner ohne Installation und dient zugleich als Vorlage: die
komplette HTTP-Logik steckt in der Funktion `request()` (rund 15 Zeilen).

## Endpunkte

| Endpunkt | Zweck |
|---|---|
| `GET /` | Liste aller Endpunkte + Beispiel |
| `GET /arm/health` | Läuft der Node? |
| `GET /arm/state` | Ist-Gelenkwinkel beider Arme + letzter Status |
| `GET /arm/status/<id>` | Status eines konkreten Vorgangs |
| `POST /arm/joints` | Gelenkwinkel direkt anfahren (7 Werte je Arm, rad) |
| `POST /arm/pose` | Kartesische Hand-Pose anfahren (IK + Kollisionsplanung) |
| `POST /arm/save` | Aktuelle Stellung in den Positionsspeicher schreiben |
| `POST /arm/cancel` | Laufende Bewegung abbrechen |

### `POST /arm/joints`

```json
{"right": [0.3, -0.2, 0.0, 0.5, 0.0, 0.0, 0.0], "wait": 60}
```

7 Gelenkwinkel in Radiant (Schulter Pitch/Roll/Yaw, Ellbogen, Handgelenk
Roll/Pitch/Yaw). `left` und/oder `right`, mindestens eine Seite.

### `POST /arm/pose`

```json
{
  "right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0], "frame": "pelvis"},
  "wait": 60
}
```

Kartesisches Ziel; die Gelenkwinkel werden serverseitig per IK berechnet und
dann kollisionsfrei geplant. Orientierung wahlweise als `rpy_deg`
(Roll/Pitch/Yaw in Grad) oder `orientation` (Quaternion `[x, y, z, w]`) — nur
eines von beidem angeben. `frame` leer = bereits im IK-Weltframe
(`pelvis`), sonst wird per TF transformiert.

Optional: `hands` (6 Fingerwinkel je Hand, fährt gemeinsam mit den Armen
los), `apply_ee_offset` (Default `true`, dieselbe Konvention wie der
RViz-Marker).

### `wait`-Parameter

`wait` gibt an, wie lange (in Sekunden) der Request auf den **Endzustand**
wartet (`reached`, `failed`, `rejected`, `cancelled`). `0` oder fehlend =
Request kehrt zurück, sobald das Kommando angenommen oder abgelehnt wurde
(nicht erst, wenn die Bewegung fertig ist). Maximum 300 Sekunden.

### `POST /arm/save`

```json
{"name": "Regal oben", "category": "Greifen", "components": ["arms", "hands"]}
```

Bewegt nichts — schreibt die aktuelle Stellung in dieselbe Datei, die auch
der Streamdeck-Speichern-Dialog nutzt (siehe
[10_arm_manipulation_anleitung.md](10_arm_manipulation_anleitung.md)).
`components`: `left_arm`, `right_arm`, `left_hand`, `right_hand`, oder die
Kurzformen `arms`, `hands`, `all`.

### HTTP-Statuscodes

| Code | Bedeutung |
|---|---|
| 200 | Ziel erreicht / erfolgreich gespeichert |
| 202 | Angenommen, läuft noch (kein bzw. zu kurzes `wait`) |
| 400 | Ungültiges Kommando (Format) |
| 401 | Fehlender/falscher `X-Auth-Token` |
| 409 | Abgelehnt (z. B. Manipulation nicht aktiviert, E-Stop aktiv) |
| 422 | Nicht erreichbar (IK konvergiert nicht / Planung gescheitert) |
| 504 | Keine Antwort vom `arm_controller` — läuft der Node? |

## Beispielantwort

```json
{
  "id": "3f9a1c4e77b2",
  "state": "reached",
  "reason": "",
  "detail": {"right": {"pos_err_m": 0.0012, "ori_err_deg": 0.4, "converged": true}},
  "sides": ["right"],
  "history": ["..."]
}
```

## Absicherung für Netzwerkzugriff

Standardmäßig ist die API nur lokal erreichbar. Soll ein anderer Rechner
zugreifen, `arm_api_host` bewusst öffnen **und** ein Token setzen
(`arm_api_token`, Header `X-Auth-Token`) — die Schnittstelle steuert einen
echten Roboterarm und gehört nicht ungeschützt ins Netz.

```bash
ros2 launch g1pilot manipulation_launcher.launch.py \
  arm_api_host:=0.0.0.0 arm_api_token:=<geheimes-token>
```

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| `Keine Verbindung` | Läuft `arm_api`? (`enable_arm_api:=true`, Default an). Port belegt? |
| `409` bei jedem Request | Manipulation nicht aktiviert — `ENABLE MANIPULATION` am Streamdeck. |
| `422`, Detail zeigt großen `pos_err_m` | Ziel außerhalb des erreichbaren Arbeitsraums. |
| `504` | `arm_controller` läuft nicht oder ist abgestürzt. |
| Request hängt lange | `wait` gesetzt und Bewegung braucht Zeit (Planung kann mehrere Sekunden dauern) — normal. |
