# Arm-API — Technik

Richtet sich an: Entwickler, die die HTTP-Brücke ändern oder ein eigenes,
unabhängiges Projekt tiefer an die G1-DDS-Schnittstelle anbinden wollen (statt
über HTTP). Für die reine Nutzung siehe
[20_arm_api_anleitung.md](20_arm_api_anleitung.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/manipulation/arm_api.py` | ROS-Node + `http.server`-Handler, HTTP ↔ ROS-Topics |
| `g1pilot/manipulation/arm_command.py` | Wire-Format: Parsen/Validieren, Status-Konstanten (ROS-frei, testbar ohne laufenden Roboter) |
| `g1pilot/examples/arm_api/arm_cli.py` | Referenz-Client (nur Standardbibliothek) |

## Architektur

```
Client --HTTP--> arm_api --/g1pilot/arm_command--> arm_controller (250 Hz)
                        <--/g1pilot/arm_command/status--
```

`arm_api` ist bewusst ein **eigener** Node: der `arm_controller` regelt mit
250 Hz, dort hat ein Webserver nichts zu suchen. Die Brücke selbst hält
**keine** Kontrolllogik — sie validiert über `manipulation/arm_command.py`
(dieselbe Funktion, die auch `arm_controller._on_arm_command` nutzt, damit
Format und Grenzen an genau einer Stelle definiert sind), publiziert auf
ROS-Topics und sammelt die Statusantworten pro Vorgang.

## Wire-Format (`arm_command.py`)

`parse_command(raw)` normalisiert ein rohes JSON-Objekt zu:

```python
{"id": str, "type": "joints"|"pose", "sides": [...],
 "targets": {side: [7 rad] | {"position","quat_xyzw","frame"}},
 "hands": {side: [6]}, "apply_ee_offset": bool}
```

Geprüft wird **nur das Format** (Struktur, Anzahl, endliche Zahlen) —
Gelenklimits, Kollision, Reichweite und ob die Arme überhaupt bereit sind,
prüft der `arm_controller`, weil dafür Robotermodell und Live-Zustand nötig
sind (siehe
[11_arm_manipulation_technik.md](11_arm_manipulation_technik.md)). Ein
ungültiges Kommando wirft `CommandError` mit einer Begründung, die 1:1 an den
Aufrufer zurückgeht.

Zustände (`ST_*`, auf `/g1pilot/arm_command/status`):

```
ST_ACCEPTED    angenommen, Planung/Fahrt läuft
ST_REJECTED    abgelehnt (Format/Limits/Arme nicht bereit)   -- Endzustand
ST_EXECUTING   Bahn wird abgefahren
ST_REACHED     Ziel erreicht                                  -- Endzustand
ST_FAILED      IK/Planung fehlgeschlagen                      -- Endzustand
ST_CANCELLED   abgebrochen (E-Stop, Marker, /arm/cancel)       -- Endzustand
```

Speichern (`/arm/save`) läuft über ein eigenes Topic
(`/g1pilot/pose_store/save`) mit eigenem Erfolgs-Endzustand `ST_SAVED`
(`parse_save_command`, `save_status_message`) — dieselbe Wire-Struktur, die
auch der Streamdeck-Speichern-Dialog sendet (siehe
`teleoperation/ui_interface.py::PoseSaveDialog`).

## `ArmApiNode` (ROS-Seite)

- Publisher: `/g1pilot/arm_command` (String, JSON), `/g1pilot/arm_command/cancel`
  (Bool), `/g1pilot/pose_store/save` (String, gleiches Topic wie die GUI).
- Subscriber: `/g1pilot/arm_command/status`, `/g1pilot/pose_store/save/status`
  (beide laufen in `_on_status` zusammen), `/joint_states` (für `GET
  /arm/state`).
- `submit(wire, cmd, save=False)`: registriert den Vorgang (`_Vorgang`,
  Status-Verlauf + zwei `threading.Event`s: `answered`/`terminal`) **vor**
  dem Publish — sonst könnte die Statusantwort des Controllers das Rennen
  gewinnen und ins Leere laufen.
- Auf das Topic geht das **Wire-Format** (`wire`, so wie der Client es
  geschickt hat, nur mit gesetzter `id`/`type`), nicht das normalisierte
  `cmd` — der `arm_controller` parst mit derselben `parse_command()`, erwartet
  also dieselbe Rohstruktur.

## `_Handler` (HTTP-Seite)

`http.server.ThreadingHTTPServer`, läuft in einem Daemon-Thread neben `rclpy.spin(node)`.

- `_authorized()`: ohne gesetztes `auth_token`-Parameter immer `True`
  (nur zusammen mit `bind_host=127.0.0.1` vertretbar).
- `do_POST` für `/arm/joints` und `/arm/pose`: liest JSON, erzwingt, dass ein
  im Body mitgegebenes `type` zum Pfad passt (verhindert versehentliches
  Vertauschen von Gelenkwinkeln und kartesischem Ziel), ruft
  `ac.parse_command`, `submit()`, wartet mindestens auf die erste Antwort
  (`ACCEPT_TIMEOUT_S = 2.0`), danach optional bis `wait` Sekunden auf den
  Endzustand (`MAX_WAIT_S = 300.0`). Bildet den Endzustand auf einen
  HTTP-Code ab (siehe Tabelle in der Anleitung).
- `_do_save`: analog für `/arm/save`, aber ohne Warteschleife über den
  ROS-Regeltakt hinaus — die Antwort kommt innerhalb eines Ticks.

## Sicherheit

- Default-Bind `127.0.0.1` — die Schnittstelle steuert einen echten
  Roboterarm, sie gehört nicht ungeschützt ins Netz.
- Alle Sicherheitsgates (E-Stop, `ENABLE MANIPULATION`, Kollision,
  Geschwindigkeits-/Gelenklimits) sitzen im `arm_controller`, nicht in
  `arm_api` — die Brücke kann sie prinzipbedingt nicht umgehen, weil sie nur
  auf dasselbe Topic schreibt wie jeder andere Aufrufer (Streamdeck
  eingeschlossen).
- Für Netzwerkzugriff: `arm_api_host` öffnen **und** `arm_api_token`
  setzen (Header `X-Auth-Token`); die Prüfung läuft dann für GET/POST
  gleichermaßen.

## Parameter (`manipulation_launcher.launch.py`)

| Parameter | Default | Bedeutung |
|---|---|---|
| `enable_arm_api` | `true` | Node überhaupt starten |
| `arm_api_host` | `127.0.0.1` | Bind-Adresse |
| `arm_api_port` | `8770` | Port |
| `arm_api_token` | `""` (leer) | `X-Auth-Token`, leer = keine Prüfung |

Zusätzlich node-intern: `history` (wie viele abgeschlossene Vorgänge im
Speicher bleiben, für `GET /arm/status`, Default 200).

## Ohne eigenen HTTP-Client: direkt über DDS

Wer ohnehin schon ein Projekt hat, das den echten G1 über das Unitree SDK2
(Python oder C++) oder `unitree_ros2` anspricht, kann auch ganz ohne
`arm_api` und ohne den restlichen g1pilot-Stack direkt gegen die
MuJoCo-Simulation testen: Die Simulation spricht exakt dieselbe
DDS-Schnittstelle wie der echte Roboter (`rt/lowcmd`/`rt/lowstate`,
`unitree_hg`-IDL). Es genügt, die eigene DDS-Verbindung auf **Domain 1,
Interface `lo`** zu zeigen statt auf das Roboter-Ethernet (siehe
[02_architektur.md](02_architektur.md), Abschnitt „Sim ↔ Real"). Das
umgeht den kompletten g1pilot-ROS-Graph inklusive Arm-Controller und
Sicherheitsgates — eigenverantwortlich zu nutzen, meist für Projekte, die
selbst einen vollständigen Regler (inklusive eigener Kollisionsprüfung)
mitbringen. Für alles, was Sicherheitsgates, IK und Planung des
g1pilot-Stacks mitnutzen soll, ist `arm_api` der vorgesehene Weg.
