# ARM_API.md — Posen live einspielen und direkt ausführen

> **Status: implementiert.** Für fremde Projekte, die eine Armposition
> *berechnen*, sie aber nicht ausführen können. Sie schicken das Ziel hierher,
> g1pilot fährt es an. **Es wird nichts gespeichert** — der Positionsspeicher
> (`pose_store.py`, siehe `SCENE_BRIDGE.md` §9) ist eine andere Sache.

Zwei Varianten, je nachdem was das aufrufende Projekt ausrechnet:

| Variante | Eingabe | Wozu |
|---|---|---|
| **Gelenkwinkel** | 7 Werte je Arm [rad] | Das Projekt kennt die Gelenkstellung schon. Eindeutig, keine IK. |
| **Kartesische Pose** | Position + Orientierung der Hand | Das Projekt kennt **nur die Wunsch-Pose**. g1pilot rechnet die Gelenkwinkel per IK aus und plant den Weg. |

In beiden Fällen wird die Bahn **jedes Mal neu geplant** (OMPL RRTConnect,
kollisionsfrei gegen Körper und Umgebung) und die Arme fahren **synchron**.
Gespeichert wird nichts, geschummelt an Sicherheitsgates auch nicht: ein
eingespieltes Kommando geht durch **genau dieselben Schranken** wie der
Streamdeck-Button „POSE ANFAHREN".

---

## 1 · Zwei Wege hinein

```
        HTTP + JSON                        ROS 2 Topic
   (kein ROS beim Aufrufer)          (Aufrufer ist im ROS-Graph)
            │                                   │
            ▼                                   │
   arm_api  (Node, :8770)                       │
   POST /arm/joints | /arm/pose                 │
            │  std_msgs/String (JSON)           │
            └────────► /g1pilot/arm_command ◄───┘
                              │
                       arm_controller (250 Hz)
                       IK → Planung → Ausführung
                              │
                       /g1pilot/arm_command/status
```

**HTTP ist der empfohlene Weg für fremde Projekte.** Über ROS müsste der
Aufrufer ROS 2 Humble, dieselbe `ROS_DOMAIN_ID` und die CycloneDDS-Config des
Containers mitbringen; per HTTP genügt ein POST aus beliebiger Sprache. Die
Brücke enthält **keine** eigene Logik — sie validiert mit derselben Funktion wie
der Controller (`manipulation/arm_command.py`) und übersetzt auf das Topic.

Der Container läuft mit `network_mode: host`, deshalb erreicht ein Prozess **auf
dem Host** die API unter `http://localhost:8770` — ohne dass sie im Netz steht
(Default-Bind ist `127.0.0.1`, siehe §7).

---

## 2 · Voraussetzungen (sonst wird abgelehnt)

Der Arm bewegt sich nur, wenn der Stack bereit ist — dieselbe Reihenfolge wie
beim manuellen Betrieb:

1. `START`
2. `START BALANCING`
3. `ENABLE MANIPULATION` (setzt `/g1pilot/arms/enabled`)
4. kein Homing, kein `WALK`, E-Stop quittiert

Fehlt etwas, kommt eine **Ablehnung mit Grund** zurück (HTTP 409) — die
Bewegung wird nicht „irgendwann später" ausgeführt. In der Sim aktiviert die
Streamdeck-GUI Schritte 1–3 automatisch ~3 s nach dem Start; auf dem echten G1
**nie** automatisch.

---

## 3 · HTTP-Endpunkte

Basis: `http://127.0.0.1:8770`. Alle Bodies und Antworten sind JSON.

### `POST /arm/joints` — Gelenkwinkel anfahren

```json
{
  "id": "optional-eigene-id",
  "left":  [0.3, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0],
  "right": [0.3, -0.2, 0.0, 0.5, 0.0, 0.0, 0.0],
  "hands": {"right": [0, 0, 0, 0, 0, 0]},
  "wait": 60
}
```

* `left` / `right`: **7 Werte in rad**, mindestens eine Seite. Reihenfolge und
  Grenzen siehe §5. Werte außerhalb der Grenzen werden abgelehnt (mit Gelenkname).
* `hands`: optional, 6 Inspire-Fingerwinkel je Hand — fahren mit der Armbewegung
  gemeinsam los. Ohne laufende Inspire-Bridge wird das ignoriert.
* `wait`: Sekunden, die der Request auf den **Endzustand** wartet (Default `0` =
  Antwort, sobald angenommen/abgelehnt; max. 300).

### `POST /arm/pose` — kartesische Hand-Pose anfahren (IK)

```json
{
  "right": {
    "position": [0.35, -0.20, 0.10],
    "orientation": [0.0, 0.0, 0.0, 1.0],
    "frame": "pelvis"
  },
  "wait": 60
}
```

* `position`: `[x, y, z]` in **Metern**.
* `orientation`: Quaternion **`[x, y, z, w]`** (wird normiert) — *oder* statt
  dessen `"rpy_deg": [roll, pitch, yaw]` in Grad, wenn der Aufrufer keine
  Quaternion-Bibliothek hat. Beides gleichzeitig ist ein Fehler.
* `frame`: TF-Frame des Ziels. Weglassen = `pelvis` (der IK-World-Frame,
  Parameter `ik_world_frame`). Jeder andere Frame wird per TF transformiert;
  **ist der Frame unbekannt, wird abgelehnt** — im falschen Frame zu fahren wäre
  gefährlich.
* `apply_ee_offset` (Default `true`): Ziel wird wie eine **Marker-Pose**
  interpretiert, d.h. der konfigurierte statische EE-Offset (`ee_offset_*`) wird
  aufgerechnet — dann landet die Hand dort, wo sie auch mit dem RViz-Marker
  landen würde. `false` = Ziel ist die Pose des IK-Frames selbst. Bei
  unkonfigurierten Offsets (Default: alles 0) macht das keinen Unterschied.
  Die **Auto-Kalibrierung** des Markers wird hier grundsätzlich nicht angewandt —
  sie richtet den Marker auf die Ist-Hand aus und würde ein absolut gemeintes
  Ziel verschieben.

Ist die Pose nicht erreichbar (außerhalb der Reichweite, Singularität), kommt
**HTTP 422** mit dem Restfehler in `detail` — der Arm bewegt sich nicht.

### Weitere Endpunkte

| Endpunkt | Zweck |
|---|---|
| `POST /arm/cancel` | Laufende Bewegung/Planung abbrechen (Arm hält an) |
| `GET /arm/status/<id>` | Status + kompletter Verlauf eines Vorgangs |
| `GET /arm/status` | Letzte Statusmeldung (egal welcher Vorgang) |
| `GET /arm/state` | Ist-Gelenkwinkel beider Arme (aus `/joint_states`), letzter Status, offene Vorgänge |
| `GET /arm/health` | Lebt der Node? (kein Token nötig) |
| `GET /` | Selbstbeschreibung: Endpunkte + Beispiel |

`GET /arm/state` ist der bequeme Weg, die **Startpose** zu holen und nach dem
Anfahren nachzurechnen.

---

## 4 · Antworten und Zustände

Jedes Kommando hat eine `id` (selbst gesetzt oder generiert). Der Verlauf läuft
über diese Zustände — auch auf dem ROS-Topic `/g1pilot/arm_command/status`. Dort
erscheinen **auch** die Bewegungen des Positionsspeichers (Streamdeck-Button),
erkennbar an `"source": "pose_store"` und leerer `id` — praktisch, wenn ein
externes Werkzeug mitlesen will, was der Arm gerade tut:

| Zustand | Bedeutung | Endzustand |
|---|---|---|
| `accepted` | Angenommen, IK/Planung läuft | |
| `executing` | Bahn wird abgefahren | |
| `reached` | Ziel erreicht | ✔ |
| `rejected` | Nicht ausführbar: Format, Gelenklimit, Arme nicht bereit, TF fehlt | ✔ |
| `failed` | IK konvergiert nicht oder Planung findet keinen kollisionsfreien Weg | ✔ |
| `cancelled` | Abgebrochen: `/arm/cancel`, E-Stop, DISABLE, Homing, WALK, oder **Marker angefasst** (manueller Vorrang) | ✔ |

HTTP-Codes:

| Code | Heißt |
|---|---|
| `200` | Endzustand erreicht (`reached` / `cancelled`) |
| `202` | Angenommen, läuft noch (kein oder zu kurzes `wait`) |
| `400` | Kommando ist syntaktisch/inhaltlich kaputt — nichts wurde publiziert |
| `409` | Verstanden, aber jetzt nicht ausführbar (`rejected`, Grund in `reason`) |
| `422` | Ziel nicht erreichbar (`failed`, Restfehler in `detail`) |
| `401` | Token fehlt/falsch (nur wenn `auth_token` gesetzt ist) |
| `504` | Der `arm_controller` antwortet nicht — läuft der Node? |

Beispiel-Antwort auf ein unerreichbares kartesisches Ziel:

```json
{
  "id": "3f9a1c4e77b2",
  "state": "failed",
  "reason": "Ziel-Pose fuer right nicht erreichbar (IK konvergiert nicht -- Restfehler siehe detail).",
  "detail": {"right": {"pos_err_m": 1.60312, "ori_err_deg": 12.4, "converged": false}},
  "sides": ["right"]
}
```

---

## 5 · Gelenkreihenfolge und -grenzen

7 DOF je Arm, **ohne** Taille (konsistent mit Home-/Walk-Pose und
Positionsspeicher). Reihenfolge der Werte in `left` / `right`:

| # | Gelenk | Grenzen links [rad] | Grenzen rechts [rad] |
|---|---|---|---|
| 0 | `shoulder_pitch` | −3.089 … +2.670 | −3.089 … +2.670 |
| 1 | `shoulder_roll` | −1.588 … +2.252 | −2.252 … +1.588 |
| 2 | `shoulder_yaw` | −2.618 … +2.618 | −2.618 … +2.618 |
| 3 | `elbow` | −1.047 … +2.094 | −1.047 … +2.094 |
| 4 | `wrist_roll` | −1.972 … +1.972 | −1.972 … +1.972 |
| 5 | `wrist_pitch` | −1.614 … +1.614 | −1.614 … +1.614 |
| 6 | `wrist_yaw` | −1.614 … +1.614 | −1.614 … +1.614 |

(Quelle: `g1pilot/utils/joints_names.py` — dort steht die Wahrheit.)

---

## 6 · Beispiele

### curl

```bash
# Gelenkwinkel, warten bis erreicht
curl -sS -X POST http://localhost:8770/arm/joints \
  -H 'Content-Type: application/json' \
  -d '{"right": [0.3, -0.2, 0.0, 0.5, 0.0, 0.0, 0.0], "wait": 60}'

# Kartesisch, Orientierung als RPY in Grad
curl -sS -X POST http://localhost:8770/arm/pose \
  -H 'Content-Type: application/json' \
  -d '{"right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0]}, "wait": 60}'

# Ist-Zustand holen / abbrechen
curl -sS http://localhost:8770/arm/state
curl -sS -X POST http://localhost:8770/arm/cancel
```

### Python (das fremde Projekt)

```python
import requests

API = "http://localhost:8770"

# Wo stehen die Arme gerade?
start = requests.get(f"{API}/arm/state", timeout=5).json()["joints"]

# Wunsch-Pose der rechten Hand -- Gelenkwinkel rechnet g1pilot aus.
r = requests.post(f"{API}/arm/pose", timeout=120, json={
    "right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0]},
    "wait": 90,
})
res = r.json()
if r.status_code == 200 and res["state"] == "reached":
    print("angefahren")
elif r.status_code == 422:
    print("nicht erreichbar:", res["detail"])
else:
    print(r.status_code, res.get("reason") or res.get("error"))
```

### Direkt über ROS (ohne die HTTP-Brücke)

```bash
ros2 topic pub --once /g1pilot/arm_command std_msgs/msg/String \
  '{data: "{\"type\":\"joints\",\"right\":[0.3,-0.2,0.0,0.5,0.0,0.0,0.0]}"}'

ros2 topic echo /g1pilot/arm_command/status          # Verlauf mitlesen
ros2 topic pub --once /g1pilot/arm_command/cancel std_msgs/msg/Bool '{data: true}'
```

Das Topic-Format ist identisch zum HTTP-Body, plus `"type": "joints"|"pose"`
(bei HTTP bestimmt der Pfad den Typ).

---

## 7 · Konfiguration und Sicherheit

Launch-Argumente (`manipulation_launcher.launch.py`, greift über
`bringup_sim` / `bringup_real`):

| Argument | Default | Bedeutung |
|---|---|---|
| `enable_arm_api` | `true` | HTTP-Brücke starten |
| `arm_api_host` | `127.0.0.1` | Bind-Adresse |
| `arm_api_port` | `8770` | Port |
| `arm_api_token` | *(leer)* | Wenn gesetzt: Header `X-Auth-Token` erforderlich |

**Der Default-Bind ist bewusst `127.0.0.1`.** Diese Schnittstelle bewegt einen
echten Roboterarm; sie gehört nicht ungeschützt ins Netz. Soll ein anderer
Rechner sie nutzen, `arm_api_host` auf `0.0.0.0` setzen **und** `arm_api_token`
vergeben:

```bash
ros2 launch g1pilot bringup_real.launch.py \
    arm_api_host:=0.0.0.0 arm_api_token:=langes-geheimnis
curl -H 'X-Auth-Token: langes-geheimnis' ...
```

Was die API **nicht** kann (bewusst):

* Sicherheitsgates umgehen — E-Stop, `arms_enabled`, Kollisions-Gate,
  Gelenklimits und Geschwindigkeitslimits sitzen im `arm_controller`.
* Mehrere Bewegungen gleichzeitig — eine zweite Anforderung wird abgelehnt,
  solange geplant wird (`"Es laeuft bereits eine Planung."`).
* Den Marker überstimmen — wird ein RViz-Marker angefasst, gewinnt der Mensch
  und die eingespielte Bewegung wird `cancelled`.
* Etwas speichern — dafür ist der Positionsspeicher da.

---

## 8 · Wenn es nicht tut

| Symptom | Ursache / Abhilfe |
|---|---|
| `504`, „Keine Antwort vom arm_controller" | `arm_controller` läuft nicht oder anderer `ROS_DOMAIN_ID`. `ros2 node list` prüfen. |
| `409` „Arme nicht aktiviert" | `ENABLE MANIPULATION` (Streamdeck) bzw. `/g1pilot/arms/enabled` auf `true`. |
| `409` „E-Stop aktiv" | Mit `START` quittieren. |
| `422` mit großem `pos_err_m` | Ziel außerhalb der Reichweite. Reichweite ~ Schulter + Armlänge; `GET /arm/state` + FK zum Gegenrechnen. |
| `422` „Planung: …" | Kein kollisionsfreier Weg. Umgebung in `/scene_markers` prüfen (siehe `SCENE_BRIDGE.md`). |
| Verbindung abgelehnt | Node aus (`enable_arm_api:=false`), falscher Port, oder Aufruf von einem anderen Rechner bei Bind auf `127.0.0.1`. |
| Sofort `cancelled` | Marker angefasst, Homing/WALK gestartet, oder E-Stop. |

Logs: der `arm_controller` loggt jede Annahme, die IK-Restfehler, das
Planungsergebnis und jeden Abbruch mit Grund.
