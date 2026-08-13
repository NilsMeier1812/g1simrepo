# ARM_API_REFCARD.md — alle Befehle auf einem Blatt

Basis `http://127.0.0.1:8770` · JSON rein/raus · Details: [`ARM_API.md`](ARM_API.md) · Anleitung: [`ARM_API_HOWTO.md`](ARM_API_HOWTO.md)

## Endpunkte

| Methode | Pfad | Zweck |
|---|---|---|
| POST | `/arm/joints` | Gelenkwinkel anfahren |
| POST | `/arm/pose` | Kartesische Hand-Pose anfahren (IK + Planung) |
| POST | `/arm/save` | Aktuelle Stellung in die Pose-Datei speichern |
| POST | `/arm/cancel` | Laufende Bewegung/Planung abbrechen |
| GET | `/arm/state` | Ist-Gelenkwinkel + letzter Status + offene Vorgänge |
| GET | `/arm/status/<id>` | Status + Verlauf eines Vorgangs |
| GET | `/arm/status` | Letzte Statusmeldung |
| GET | `/arm/health` | Lebt der Node? (ohne Token) |
| GET | `/` | Selbstbeschreibung (ohne Token) |

## POST /arm/joints

| Feld | Typ | Default | Bedeutung |
|---|---|---|---|
| `left` / `right` | 7 × float [rad] | — | mind. eine Seite Pflicht |
| `hands` | `{left: 6×float, right: 6×float}` | — | Finger, fahren mit los |
| `wait` | float [s] | `0` | 0 = nicht auf Ende warten, max. 300 |
| `id` | string ≤64 | generiert | eigene Vorgangs-ID |

```json
{"right": [0.3, -0.2, 0.0, 0.5, 0.0, 0.0, 0.0], "wait": 60}
```

## POST /arm/pose

Pro Seite (`left` / `right`), mind. eine:

| Feld | Typ | Default | Bedeutung |
|---|---|---|---|
| `position` | `[x, y, z]` [m] | — | Pflicht |
| `orientation` | `[x, y, z, w]` | — | Quaternion, wird normiert |
| `rpy_deg` | `[r, p, y]` [°] | — | Alternative zu `orientation` (genau eines) |
| `frame` | string | `pelvis` | anderer Frame → TF; unbekannt → Ablehnung |

Top-Level: `wait`, `id`, `hands` (wie bei `/arm/joints`) + `apply_ee_offset` (bool, Default `true` = Marker-Konvention).

```json
{"right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0]}, "wait": 60}
```

## POST /arm/save

| Feld | Typ | Default | Bedeutung |
|---|---|---|---|
| `name` | string ≤128 | — | **Pflicht**, gleicher Name überschreibt |
| `category` | string ≤128 | `Allgemein` | leer/fehlend = Default-Kategorie; neue wird angelegt |
| `components` | string[] | `["left_arm","right_arm"]` | siehe unten |
| `id` | string ≤64 | generiert | eigene Vorgangs-ID |

`components`: `left_arm` · `right_arm` · `left_hand` · `right_hand` · `arms` (beide Arme) · `hands` / `hand` (beide Hände) · `all`

Antwort: `requested` (angefordert) vs. `components` (gespeichert) vs. `skipped` (nicht verfügbar).
Codes hier: `200` gespeichert · `400` Format · `422` nichts speicherbar/Schreibfehler. Kein `wait` nötig.

```json
{"name": "Regal oben", "category": "Greifen", "components": ["arms", "hands"]}
```

## Zustände (`state`)

| Zustand | Endzustand | Wann |
|---|---|---|
| `accepted` | | angenommen, IK/Planung läuft |
| `executing` | | Bahn wird abgefahren |
| `reached` | ✔ | Ziel erreicht |
| `saved` | ✔ | nur `/arm/save`: in Datei geschrieben |
| `rejected` | ✔ | Format, Gelenklimit, Arme nicht bereit, TF fehlt |
| `failed` | ✔ | IK/Planung erfolglos, nichts speicherbar |
| `cancelled` | ✔ | `cancel`, E-Stop, DISABLE, Homing, WALK, Marker angefasst |

## HTTP-Codes

| Code | Bedeutung |
|---|---|
| 200 | Endzustand (`reached` / `saved` / `cancelled`) |
| 202 | läuft noch (`accepted` / `executing`) |
| 400 | Kommando kaputt — nichts publiziert |
| 401 | Token fehlt/falsch |
| 404 | unbekannter Pfad |
| 409 | jetzt nicht ausführbar (`rejected`) — bei `/arm/save` stattdessen `400` |
| 422 | Ziel unerreichbar / nichts speicherbar (`failed`) |
| 504 | `arm_controller` antwortet nicht |

## Gelenkreihenfolge (7 je Arm, rad)

| # | Gelenk | links | rechts |
|---|---|---|---|
| 0 | `shoulder_pitch` | −3.089 … +2.670 | −3.089 … +2.670 |
| 1 | `shoulder_roll` | −1.588 … +2.252 | −2.252 … +1.588 |
| 2 | `shoulder_yaw` | −2.618 … +2.618 | −2.618 … +2.618 |
| 3 | `elbow` | −1.047 … +2.094 | −1.047 … +2.094 |
| 4 | `wrist_roll` | −1.972 … +1.972 | −1.972 … +1.972 |
| 5 | `wrist_pitch` | −1.614 … +1.614 | −1.614 … +1.614 |
| 6 | `wrist_yaw` | −1.614 … +1.614 | −1.614 … +1.614 |

## CLI (`examples/arm_api/arm_cli.py`, nur stdlib)

```bash
python3 arm_cli.py health
python3 arm_cli.py state
python3 arm_cli.py status [<id>]
python3 arm_cli.py cancel
python3 arm_cli.py joints --right Q1..Q7 [--left Q1..Q7] [--wait S] [--id X]
python3 arm_cli.py pose  --right X Y Z [--left X Y Z] (--rpy R P Y | --quat X Y Z W)
                         [--frame F] [--no-ee-offset] [--wait S] [--id X]
python3 arm_cli.py save  <name> [--category K] [--components arms hands ...]
# global: --url http://host:8770  --token <X-Auth-Token>
```

Exit-Code `0` = `reached`/`saved`/Abfrage ok, sonst `1`.

## ROS-Topics (statt HTTP)

| Topic | Typ | Richtung |
|---|---|---|
| `/g1pilot/arm_command` | `std_msgs/String` (JSON + `"type": "joints"\|"pose"`) | rein |
| `/g1pilot/arm_command/cancel` | `std_msgs/Bool` | rein |
| `/g1pilot/arm_command/status` | `std_msgs/String` (JSON) | raus |
| `/g1pilot/pose_store/save` | `std_msgs/String` (JSON) | rein |
| `/g1pilot/pose_store/save/status` | `std_msgs/String` (JSON) | raus |
| `/g1pilot/pose_store/goto` | `std_msgs/String` (Pose-Name) | rein |

## Voraussetzungen / Konfiguration

Vor dem ersten Fahrbefehl: `START` → `START BALANCING` → `ENABLE MANIPULATION`; kein Homing/WALK, E-Stop quittiert. (Sim: automatisch ~3 s nach Start.) Speichern braucht das nicht.

Launch-Argumente: `enable_arm_api` (`true`) · `arm_api_host` (`127.0.0.1`) · `arm_api_port` (`8770`) · `arm_api_token` (leer). Token gesetzt → Header `X-Auth-Token`.

---

# Beispiele

Alle Antworten unten sind echte Ausgaben der Schnittstelle (Feldnamen und
Struktur 1:1). `history` ist im Beispiel gekürzt.

## Bewegen: Gelenkwinkel

**Vollständig** — `POST /arm/joints`, beide Arme, Finger, eigene ID, wartet bis zum Ende:

```json
{
  "id": "hebe-links-rechts",
  "left":  [0.30,  0.20, 0.00, 0.50, 0.00, 0.00, 0.00],
  "right": [0.30, -0.20, 0.00, 0.50, 0.00, 0.00, 0.00],
  "hands": {"left": [0, 0, 0, 0, 0, 0], "right": [1000, 1000, 1000, 1000, 1000, 1000]},
  "wait": 60
}
```

Antwort `200`:

```json
{
  "id": "hebe-links-rechts",
  "state": "reached",
  "reason": "",
  "detail": null,
  "sides": ["left", "right"],
  "history": [
    {"id": "hebe-links-rechts", "state": "accepted",  "source": "arm_command", "sides": ["left", "right"], "received_at": 1786000000.0},
    {"id": "hebe-links-rechts", "state": "executing", "source": "arm_command", "sides": ["left", "right"], "received_at": 1786000000.4},
    {"id": "hebe-links-rechts", "state": "reached",   "source": "arm_command", "sides": ["left", "right"], "received_at": 1786000004.1}
  ]
}
```

**Minimal** — `POST /arm/joints`, ein Arm, ohne Warten (Antwort `202`, `state: "accepted"`):

```json
{"right": [0.30, -0.20, 0.00, 0.50, 0.00, 0.00, 0.00]}
```

## Bewegen: kartesische Pose

**Vollständig** — `POST /arm/pose`, beide Arme, links per Quaternion, rechts per RPY, Finger mit:

```json
{
  "id": "greifen-regal",
  "left":  {"position": [0.35,  0.20, 0.10], "orientation": [0.0, 0.0, 0.0, 1.0], "frame": "pelvis"},
  "right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0]},
  "hands": {"right": [800, 800, 800, 800, 800, 500]},
  "apply_ee_offset": true,
  "wait": 90
}
```

Antwort `200` (`detail` = IK-Restfehler je Seite):

```json
{
  "id": "greifen-regal",
  "state": "reached",
  "reason": "",
  "detail": {"right": {"pos_err_m": 0.0004, "ori_err_deg": 0.12, "converged": true}},
  "sides": ["left", "right"],
  "history": ["..."]
}
```

**Minimal** — `POST /arm/pose`:

```json
{"right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0]}}
```

**Ziel in Weltkoordinaten** — `POST /arm/pose`, Frame wird per TF umgerechnet:

```json
{"right": {"position": [1.20, 0.30, 0.90], "rpy_deg": [0, 0, 0], "frame": "map"}, "wait": 90}
```

## Speichern

**Vollständig** — `POST /arm/save`:

```json
{"name": "Regal oben", "category": "Greifen", "components": ["arms", "hands"]}
```

Antwort `200` — Inspire-Bridge lief nicht, deshalb `skipped`:

```json
{
  "id": "bad037326be6",
  "state": "saved",
  "name": "Regal oben",
  "category": "Greifen",
  "requested": ["left_arm", "right_arm", "left_hand", "right_hand"],
  "components": ["left_arm", "right_arm"],
  "skipped": ["left_hand", "right_hand"],
  "reason": ""
}
```

**Minimal** — beide Arme in die Default-Kategorie `Allgemein`:

```json
{"name": "Zwischenstellung"}
```

## Abfragen

Antwort `GET /arm/state`:

```json
{
  "last_status": {"id": "3f9a1c4e77b2", "state": "reached", "source": "arm_command", "sides": ["right"], "received_at": 1786000000.0},
  "joints": {"left": [0.301, 0.198, 0.0, 0.502, 0.0, 0.0, 0.0],
             "right": [0.298, -0.201, 0.0, 0.499, 0.0, 0.0, 0.0],
             "stamp": 1786000000.0},
  "open": []
}
```

Antwort `GET /arm/status/<id>`:

```json
{"id": "hebe-links-rechts", "state": "reached", "history": ["..."]}
```

Antwort `GET /arm/health` · `POST /arm/cancel`:

```json
{"ok": true, "node": "arm_api"}
```

```json
{"cancelled": true}
```

## Fehlerantworten

`400` — Kommando kaputt, nichts wurde publiziert:

```json
{"error": "right (Gelenkwinkel): erwarte 7 Werte, bekam 3."}
```

`409` — verstanden, aber jetzt nicht ausführbar:

```json
{
  "id": "355c5686037d",
  "state": "rejected",
  "reason": "Arme nicht aktiviert (ENABLE MANIPULATION).",
  "detail": null,
  "sides": ["right"],
  "history": ["..."]
}
```

`422` — Ziel unerreichbar (Arm bewegt sich nicht):

```json
{
  "id": "9ebecee3384a",
  "state": "failed",
  "reason": "Ziel-Pose fuer right nicht erreichbar (IK konvergiert nicht -- Restfehler siehe detail).",
  "detail": {"right": {"pos_err_m": 1.60312, "ori_err_deg": 12.4, "converged": false}},
  "sides": ["right"],
  "history": ["..."]
}
```

`504` — `arm_controller` läuft nicht:

```json
{"id": "5f1c0f0a9b31", "state": "unknown", "error": "Keine Antwort vom arm_controller -- laeuft der Node?"}
```

## curl

```bash
A=http://localhost:8770; H='Content-Type: application/json'

curl -sS -X POST $A/arm/joints -H "$H" \
  -d '{"right":[0.30,-0.20,0.00,0.50,0.00,0.00,0.00],"wait":60}'

curl -sS -X POST $A/arm/pose -H "$H" \
  -d '{"right":{"position":[0.35,-0.20,0.10],"rpy_deg":[0,0,0]},"wait":90}'

curl -sS -X POST $A/arm/save -H "$H" \
  -d '{"name":"Regal oben","category":"Greifen","components":["arms","hands"]}'

curl -sS -X POST $A/arm/cancel -H "$H" -d '{}'
curl -sS $A/arm/state
curl -sS $A/arm/status/hebe-links-rechts
curl -sS -H 'X-Auth-Token: geheim' $A/arm/state      # nur wenn Token konfiguriert
```

## CLI (dieselben Vorgänge)

```bash
cd g1pilot/examples/arm_api
python3 arm_cli.py joints --right 0.30 -0.20 0.00 0.50 0.00 0.00 0.00 --wait 60
python3 arm_cli.py pose --right 0.35 -0.20 0.10 --rpy 0 0 0 --wait 90
python3 arm_cli.py pose --left 0.35 0.20 0.10 --quat 0 0 0 1 --frame map --wait 90
python3 arm_cli.py save "Regal oben" --category Greifen --components arms hands
python3 arm_cli.py state && python3 arm_cli.py cancel
```

## Python (fahren, prüfen, speichern)

```python
import requests
A = "http://localhost:8770"

start = requests.get(f"{A}/arm/state", timeout=5).json()["joints"]["right"]

r = requests.post(f"{A}/arm/pose", timeout=120, json={
    "right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0]}, "wait": 90})
res = r.json()
if r.status_code != 200 or res["state"] != "reached":
    raise RuntimeError(f"{r.status_code} {res.get('state')}: "
                       f"{res.get('reason') or res.get('detail') or res.get('error')}")

s = requests.post(f"{A}/arm/save", timeout=15, json={
    "name": "Regal oben", "category": "Greifen", "components": ["arms", "hands"]}).json()
if s.get("skipped"):
    print("nicht mitgespeichert:", s["skipped"])
```

## ROS (ohne HTTP)

```bash
ros2 topic pub --once /g1pilot/arm_command std_msgs/msg/String \
  '{data: "{\"type\":\"joints\",\"right\":[0.3,-0.2,0.0,0.5,0.0,0.0,0.0]}"}'

ros2 topic pub --once /g1pilot/arm_command std_msgs/msg/String \
  '{data: "{\"type\":\"pose\",\"right\":{\"position\":[0.35,-0.2,0.1],\"rpy_deg\":[0,0,0]}}"}'

ros2 topic pub --once /g1pilot/pose_store/save std_msgs/msg/String \
  '{data: "{\"name\":\"Regal oben\",\"category\":\"Greifen\",\"components\":[\"arms\"]}"}'

ros2 topic pub --once /g1pilot/arm_command/cancel std_msgs/msg/Bool '{data: true}'
ros2 topic echo /g1pilot/arm_command/status
ros2 topic echo /g1pilot/pose_store/save/status
```
