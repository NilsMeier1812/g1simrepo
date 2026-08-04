# ARM_API_HOWTO.md — Befehle senden, Schritt für Schritt

Praktische Anleitung zum Einspielen von Armposen. Die **Referenz** (jedes Feld,
jeder Zustand, jeder HTTP-Code) steht in [`ARM_API.md`](ARM_API.md) — hier geht
es darum, *wie man es tatsächlich macht*.

Fertiges Werkzeug dazu: **[`examples/arm_api/arm_cli.py`](examples/arm_api/arm_cli.py)**
— nur Standard-Bibliothek, läuft auf jedem Rechner mit Python 3, ohne ROS und
ohne `pip install`. Gleichzeitig Vorlage zum Kopieren: die ganze Anbindung ist
die Funktion `request()`, rund 15 Zeilen.

---

## 1 · In 60 Sekunden

```bash
cd g1pilot/examples/arm_api

python3 arm_cli.py health                      # 1. Lebt die Schnittstelle?
python3 arm_cli.py state                       # 2. Wo stehen die Arme gerade?
python3 arm_cli.py pose --right 0.35 -0.20 0.10 --rpy 0 0 0 --wait 60
                                               # 3. Rechte Hand dorthin fahren
```

Antwort auf den dritten Befehl, wenn alles gut geht:

```json
{
  "id": "3f9a1c4e77b2",
  "state": "reached",
  "sides": ["right"],
  "history": [ { "state": "accepted" }, { "state": "executing" }, { "state": "reached" } ]
}
```

Exit-Code `0` = erreicht, `1` = nicht (Grund steht in `reason` bzw. `detail`).
Damit taugt das Skript direkt für Shell-Pipelines: `python3 arm_cli.py … && echo ok`.

---

## 2 · Bevor irgendwas fährt: Stack hochfahren

Die Schnittstelle nimmt nur an, wenn der Roboter bereit ist. Reihenfolge:

```bash
cd g1pilot
./start.sh                 # bzw. docker compose --profile sim up
```

Dann in der **Streamdeck-GUI**:

1. `START`
2. `START BALANCING`
3. `ENABLE MANIPULATION`

In der **Sim** macht die GUI Schritte 1–3 automatisch ~3 s nach dem Start. Auf
dem **echten G1 nie automatisch** — dort drückt der Operator.

Prüfen, ob es passt:

```bash
python3 examples/arm_api/arm_cli.py health     # {"ok": true, ...}
python3 examples/arm_api/arm_cli.py state      # joints: Ist-Winkel beider Arme
```

Kommt bei `health` „Keine Verbindung", läuft der `arm_api`-Node nicht (siehe §8).
Kommt beim ersten Fahrbefehl `409` mit `"Arme nicht aktiviert"`, fehlt Schritt 3.

---

## 3 · Die drei Arten von Befehlen

### a) Gelenkwinkel — wenn dein Projekt die Gelenkstellung schon kennt

7 Werte je Arm, **in Radiant**, Reihenfolge
`shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw`:

```bash
python3 arm_cli.py joints --right 0.3 -0.2 0.0 0.5 0.0 0.0 0.0 --wait 60
python3 arm_cli.py joints --left 0.3 0.2 0.0 0.5 0.0 0.0 0.0 \
                          --right 0.3 -0.2 0.0 0.5 0.0 0.0 0.0 --wait 60
```

Grenzen stehen in `ARM_API.md` §5. Ein Wert daneben → `409` mit Gelenkname:
`"right Gelenk 3 (right_elbow_joint) = 3.500 rad liegt ausserhalb [-1.047, 2.094]."`

### b) Kartesische Pose — wenn dein Projekt nur die Wunsch-Pose kennt

Das ist der Normalfall für fremde Projekte: **g1pilot rechnet die Gelenkwinkel
per IK aus** und plant den kollisionsfreien Weg.

```bash
# Orientierung in Grad (bequem)
python3 arm_cli.py pose --right 0.35 -0.20 0.10 --rpy 0 0 0 --wait 60

# Orientierung als Quaternion -- Reihenfolge x y z w (nicht w x y z!)
python3 arm_cli.py pose --right 0.35 -0.20 0.10 --quat 0 0 0 1 --wait 60

# Ziel in einem anderen TF-Frame (wird transformiert)
python3 arm_cli.py pose --right 1.20 0.30 0.90 --rpy 0 0 0 --frame map --wait 60
```

Position in **Metern**, Default-Frame ist `pelvis` (Ursprung im Becken, x nach
vorn, y nach links, z nach oben).

### c) Aktuelle Stellung speichern

Bewegt nichts — schreibt die Stellung, in der der Arm gerade steht, in die
Pose-Datei. Danach ist sie in der GUI unter „POSE ANFAHREN" auswählbar.

```bash
# In eine Kategorie ("Ordner"), Arme und Hände
python3 arm_cli.py save "Regal oben" --category Greifen --components arms hands

# Ohne Kategorie -> Default-Kategorie "Allgemein"; ohne --components -> beide Arme
python3 arm_cli.py save "Zwischenstellung"

# Nur einzelne Komponenten
python3 arm_cli.py save "Nur links" --components left_arm left_hand
```

`--components` versteht `left_arm`, `right_arm`, `left_hand`, `right_hand` sowie
die Kurzformen `arms`, `hands`, `all`.

Die Antwort sagt, was **wirklich** gespeichert wurde. Läuft die Inspire-Bridge
nicht, landen die Arme in der Datei und die Hände erscheinen unter `skipped`:

```json
{"state": "saved", "name": "Regal oben", "category": "Greifen",
 "requested": ["left_arm", "right_arm", "left_hand", "right_hand"],
 "components": ["left_arm", "right_arm"],
 "skipped": ["left_hand", "right_hand"]}
```

Typischer Ablauf aus einem Programm: **erst fahren, dann speichern.**

```python
requests.post(f"{API}/arm/pose", timeout=120, json={
    "right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0]}, "wait": 90})
requests.post(f"{API}/arm/save", timeout=15, json={
    "name": "Regal oben", "category": "Greifen", "components": ["arms", "hands"]})
```

### d) Abbrechen

```bash
python3 arm_cli.py cancel
```

Wirkt sofort — auch mitten in der Planung. Der Arm hält an, wo er ist.

---

## 4 · Aus eigenem Code

### Python, ohne Fremdpakete (stdlib)

```python
import json, urllib.request, urllib.error

def arm(path, body, token=""):
    req = urllib.request.Request("http://localhost:8770" + path,
                                 data=json.dumps(body).encode(), method="POST")
    req.add_header("Content-Type", "application/json")
    if token:
        req.add_header("X-Auth-Token", token)
    try:
        with urllib.request.urlopen(req, timeout=310) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:          # 400/409/422 tragen den Grund im Body
        return e.code, json.loads(e.read() or b"{}")

code, res = arm("/arm/pose", {
    "right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0]},
    "wait": 90,
})
print(code, res["state"], res.get("reason", ""))
```

### Python mit `requests`

```python
import requests

API = "http://localhost:8770"

def fahre_pose(xyz, rpy=(0, 0, 0), side="right", timeout_s=90):
    r = requests.post(f"{API}/arm/pose", timeout=timeout_s + 20, json={
        side: {"position": list(xyz), "rpy_deg": list(rpy)},
        "wait": timeout_s,
    })
    res = r.json()
    if r.status_code == 200 and res["state"] == "reached":
        return True
    if r.status_code == 422:
        raise RuntimeError(f"unerreichbar: {res['detail']}")
    if r.status_code == 409:
        raise RuntimeError(f"nicht bereit: {res['reason']}")
    raise RuntimeError(f"HTTP {r.status_code}: {res}")

fahre_pose((0.35, -0.20, 0.10))
```

### JavaScript / Node

```js
const res = await fetch("http://localhost:8770/arm/pose", {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    right: { position: [0.35, -0.20, 0.10], rpy_deg: [0, 0, 0] },
    wait: 90,
  }),
});
const out = await res.json();
console.log(res.status, out.state, out.reason ?? "");
```

### curl

```bash
curl -sS -X POST http://localhost:8770/arm/joints \
  -H 'Content-Type: application/json' \
  -d '{"right":[0.3,-0.2,0.0,0.5,0.0,0.0,0.0],"wait":60}'
```

### Direkt über ROS (nur wenn du im ROS-Graph bist)

```bash
ros2 topic pub --once /g1pilot/arm_command std_msgs/msg/String \
  '{data: "{\"type\":\"joints\",\"right\":[0.3,-0.2,0.0,0.5,0.0,0.0,0.0]}"}'
ros2 topic echo /g1pilot/arm_command/status
```

Gleiches JSON wie im HTTP-Body, plus `"type"` (bei HTTP bestimmt der Pfad den
Typ). Die Antwort kommt hier **nur** über das Status-Topic — kein Rückkanal, kein
`wait`. Genau deshalb ist HTTP der empfohlene Weg.

---

## 5 · Rezepte

### Relativ zur aktuellen Stellung bewegen

Es gibt absichtlich keine „relative" Variante — hol den Ist-Zustand und rechne
selbst, dann weiß dein Projekt immer, was es kommandiert:

```python
import requests
API = "http://localhost:8770"

q = requests.get(f"{API}/arm/state", timeout=5).json()["joints"]["right"]
q[3] += 0.20                                   # Ellbogen 0.2 rad weiter beugen
requests.post(f"{API}/arm/joints", json={"right": q, "wait": 30}, timeout=60)
```

### Mehrere Posen hintereinander

Eine Bewegung zur Zeit — also **sequenziell mit `wait`**, nicht parallel
abfeuern (sonst wird die zweite abgelehnt oder ersetzt die erste):

```python
for xyz in [(0.35, -0.20, 0.10), (0.40, -0.10, 0.25), (0.30, -0.25, 0.05)]:
    r = requests.post(f"{API}/arm/pose", timeout=120, json={
        "right": {"position": list(xyz), "rpy_deg": [0, 0, 0]},
        "wait": 90,
    })
    if r.json().get("state") != "reached":
        print("Abbruch bei", xyz, r.status_code, r.json().get("reason"))
        break
```

### Nicht blockieren: abschicken und pollen

`wait` weglassen → `202` mit `id`, danach selbst nachfragen:

```python
r = requests.post(f"{API}/arm/joints", json={"right": q}, timeout=10)
vid = r.json()["id"]

while True:
    st = requests.get(f"{API}/arm/status/{vid}", timeout=5).json()
    if st["state"] in ("reached", "failed", "rejected", "cancelled"):
        break
    time.sleep(0.2)      # hier kann dein Projekt weiterarbeiten
```

### Beide Arme gleichzeitig

Beide Seiten in **einem** Kommando — dann werden sie gemeinsam geplant (14 DOF,
Arm-gegen-Arm geprüft) und fahren synchron. Zwei getrennte Kommandos wären
*nicht* gleichzeitig:

```json
{"left":  {"position": [0.35, 0.20, 0.10], "rpy_deg": [0, 0, 0]},
 "right": {"position": [0.35, -0.20, 0.10], "rpy_deg": [0, 0, 0]},
 "wait": 90}
```

### Finger mitfahren lassen

```json
{"right": [0.3, -0.2, 0.0, 0.5, 0.0, 0.0, 0.0],
 "hands": {"right": [0, 0, 0, 0, 0, 0]},
 "wait": 60}
```

6 Werte je Hand (Inspire). Die Finger starten **gemeinsam mit** der Armbewegung.
Läuft die Inspire-Bridge nicht, wird das stillschweigend übersprungen und die
Arme fahren trotzdem.

### Aus einem anderen Prozess abbrechen

```bash
curl -sS -X POST http://localhost:8770/arm/cancel
```

Der wartende Request des Fahrbefehls kehrt dann mit `state: "cancelled"` zurück
— dein Programm hängt also nicht bis zum Timeout.

### Sicherheitsschleife um alles herum

```python
def sicher_fahren(body, path="/arm/pose"):
    """Bricht bei Problemen sauber ab statt weiterzumachen."""
    try:
        r = requests.post(API + path, json=body, timeout=body.get("wait", 0) + 30)
    except requests.RequestException as e:
        requests.post(f"{API}/arm/cancel", timeout=5)   # im Zweifel: anhalten
        raise RuntimeError(f"Verbindung weg, Bewegung abgebrochen: {e}")
    res = r.json()
    if res.get("state") != "reached":
        raise RuntimeError(f"{r.status_code} {res.get('state')}: "
                           f"{res.get('reason') or res.get('detail')}")
    return res
```

---

## 6 · Was tun bei welcher Antwort

| Code / Zustand | Bedeutung | Deine Reaktion |
|---|---|---|
| `200` `reached` | Ziel erreicht | weiter |
| `200` `saved` | Pose steht in der Datei | `components` gegen `requested` prüfen (`skipped`!) |
| `202` `accepted`/`executing` | läuft noch | `GET /arm/status/<id>` pollen, oder `wait` nutzen |
| `400` | Kommando kaputt (Feld fehlt, falsche Anzahl, NaN) | **Bug im Aufrufer** — Meldung in `error` lesen, nichts wurde gesendet |
| `409` `rejected` | Verstanden, jetzt nicht möglich (E-Stop, nicht aktiviert, Gelenklimit, Planung läuft) | Ursache beheben (`reason`), dann erneut |
| `422` `failed` | Ziel nicht erreichbar oder kein kollisionsfreier Weg | `detail.pos_err_m` ansehen, Ziel näher/anders wählen |
| `200` `cancelled` | Abgebrochen (Marker angefasst, E-Stop, `cancel`) | **nicht blind wiederholen** — jemand hat eingegriffen |
| `401` | Token fehlt/falsch | `X-Auth-Token` mitschicken |
| `504` | `arm_controller` antwortet nicht | Stack prüfen (§8) |

Merksatz: `400` ist dein Fehler, `409` ist der Zustand des Roboters, `422` ist
die Physik, `504` ist die Infrastruktur.

---

## 7 · Stolperfallen

* **Radiant, nicht Grad** — bei Gelenkwinkeln. Nur `rpy_deg` ist in Grad (steht im Namen).
* **Quaternion ist `[x, y, z, w]`** — nicht `[w, x, y, z]`. Wer unsicher ist, nimmt `rpy_deg`.
* **Eine Bewegung zur Zeit.** Während geplant wird, kommt `409`. Fährt der Arm
  gerade und du schickst ein neues Ziel, wird die alte Bewegung **abgebrochen**
  (`cancelled` unter ihrer alten `id`) und die neue übernimmt.
* **Der Mensch gewinnt.** Wird ein RViz-Marker angefasst, wird deine Bewegung
  `cancelled`. Das ist Absicht.
* **Fahren und Speichern sind getrennt.** `/arm/pose` und `/arm/joints` führen
  nur aus; erst `POST /arm/save` schreibt etwas in die Pose-Datei — in dieselbe,
  die die GUI benutzt (`SCENE_BRIDGE.md` §9).
* **Gleicher Name überschreibt.** `POST /arm/save` fragt nicht nach.
* **`skipped` beachten.** „Gespeichert" heißt nicht zwangsläufig „alles
  gespeichert" — ohne Inspire-Bridge fehlen die Hände.
* **`wait` ist rein HTTP-seitig** — es beeinflusst die Bewegung nicht, nur wie
  lange dein Request offen bleibt. Maximal 300 s.
* **Reichweite.** Rund **0,45 m** von der Schulter zum Hand-TCP (aus dem URDF
  gemessen: 0,466 m × 0,97 Sicherheitsmarge). Ziele darüber hinaus werden auf
  diese Kugel geklemmt, der Restfehler bleibt groß → `422` mit `pos_err_m`, statt
  halb angefahren zu werden. Wer prüfen will, ob ein Ziel überhaupt in Frage
  kommt: Abstand zur Schulter rechnen, nicht zum Becken-Ursprung.
* **Timeout des HTTP-Clients** immer größer als `wait` setzen, sonst bricht dein
  Client ab, während die Bewegung weiterläuft.

---

## 8 · Fehlersuche

```bash
# Läuft der Node und antwortet er?
curl -sS http://localhost:8770/arm/health

# Welche Endpunkte gibt es? (Selbstbeschreibung)
curl -sS http://localhost:8770/

# Was tut der Arm gerade -- live mitlesen (im Container)
ros2 topic echo /g1pilot/arm_command/status

# Kommt mein Kommando überhaupt an?
ros2 topic echo /g1pilot/arm_command

# Läuft der Controller?
ros2 node list | grep arm_
```

| Symptom | Ursache |
|---|---|
| „Keine Verbindung zu …" | `arm_api`-Node läuft nicht (`enable_arm_api:=false`?), falscher Port, oder Aufruf von einem **anderen Rechner** bei Bind auf `127.0.0.1` (siehe `ARM_API.md` §7) |
| `504` bei jedem Befehl | `arm_api` läuft, `arm_controller` nicht — oder unterschiedliche `ROS_DOMAIN_ID` |
| `409` „Arme nicht aktiviert" | `ENABLE MANIPULATION` fehlt |
| `409` „E-Stop aktiv" | mit `START` quittieren |
| `409` „Es laeuft bereits eine Planung" | vorherigen Befehl abwarten oder `cancel` |
| Sofort `cancelled` | Marker angefasst, `WALK`/Homing gestartet, oder E-Stop |
| Arm fährt woanders hin als gedacht | Frame verwechselt (`pelvis` vs. `map`), oder `apply_ee_offset` — siehe `ARM_API.md` §3 |

Der `arm_controller` loggt jede Annahme, den IK-Restfehler in mm/Grad, das
Planungsergebnis und jeden Abbruch mit Grund — bei unklaren Fällen dort zuerst
schauen.

---

## 9 · Checkliste für die andere Person

Wenn jemand sein Projekt anbinden soll, reicht das hier:

1. **Adresse:** `http://localhost:8770` (auf demselben Rechner). Von einem
   anderen Rechner nur mit geöffnetem Bind + Token — dann bekommt er beides von dir.
2. **Was er schickt:** entweder 7 Gelenkwinkel je Arm (rad) oder eine
   Hand-Pose (Position in m + Orientierung). Format: `ARM_API.md` §3. Wenn er
   Posen auch ablegen soll: `POST /arm/save` mit `name` (+ optional `category`
   und `components`).
3. **Was er zurückbekommt:** `state` + bei Problemen `reason`/`detail`, plus die
   HTTP-Codes aus §6 hier.
4. **Was er vorher tun muss:** nichts — nur prüfen, ob `GET /arm/health`
   antwortet. Das Aktivieren des Roboters (§2) machst du.
5. **Zum Ausprobieren:** `examples/arm_api/arm_cli.py` mitgeben. Braucht nur
   Python 3, kein ROS, kein `pip install`.
