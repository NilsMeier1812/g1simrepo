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
