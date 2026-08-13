# Inspire-FTP-Hände — Technik

Richtet sich an: Entwickler, die die Hand-Bridge, das Gelenk-Mapping oder ein
neues Backend hinzufügen wollen. Für die Bedienung siehe
[60_inspire_haende_anleitung.md](60_inspire_haende_anleitung.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/manipulation/inspire_ftp/bridge.py` | ROS-2-Node `inspire_hand`: WebSocket-Server (GUIs) + HTTP-Server (GUI-Dateien) + `/joint_states` |
| `g1pilot/manipulation/inspire_ftp/joint_map.py` | Single Source of Truth: 6-DOF-Mapping → 24 URDF-Fingergelenke |
| `g1pilot/manipulation/inspire_ftp/tactile.py` | Taktil-Zonenlayout inkl. echter Modbus-Registeradressen |
| `g1pilot/manipulation/inspire_ftp/model.py` | `HandModel`: geteilter Soll-/Ist-Zustand einer Hand (threadsicher) |
| `g1pilot/manipulation/inspire_ftp/backends.py` | Die drei Backends (siehe unten) |
| `g1pilot/manipulation/inspire_ftp/modbus_client.py` | Raw-Socket-Modbus-TCP-Client (nur Standardbibliothek) |
| `g1pilot/manipulation/inspire_ftp/web/*.html` | Die beiden GUIs (Controller/Viewer), backend-unabhängig |
| `g1pilot/test/test_inspire_modbus_backend.py` | Test des Modbus-Backends gegen einen In-Process-Fake-Server |

```
  hand_controller_viewer.html ──ws://…:8766──┐
                                             ├──► inspire_hand (ROS-2-Node) ──► /joint_states ──► RViz
  inspire_hand_viewer.html    ──ws://…:8765──┘            │
                                                          └──► Backend (austauschbar)
```

## Die drei Backend-Stufen

| Stufe | Backend | Was passiert |
|---|---|---|
| 1 | `SimJointStateBackend` | Finger-Gelenke folgen der Steuerung nur in RViz (`/joint_states`); Kraft/Taktil = 0. |
| 2 | `MujocoContactBackend` | Finger als echte MuJoCo-Aktuatoren (Greifen/Kollision) via DDS `rt/inspire/cmd|state`; alle 17 Taktil-Zonen je Hand aus MuJoCo-Touch-Sensoren. **Sim-Default.** |
| 3 | `InspireModbusBackend` | Echte Hände via Modbus TCP (eine IP je Hand). Liest Ist-Winkel, Kräfte (Gramm) und alle 17 Taktil-Zonen. **Real-Default.** |

Auswahl über den Parameter `backend` (`sim`/`mujoco`/`modbus`); für `modbus`
zusätzlich `left_host`/`right_host` (leer = Seite nicht ansteuern) und
`modbus_port`.

## Mapping-Konvention (`joint_map.py`)

Inspire-Winkel-Konvention: `1000` = offen (Gelenk 0 rad), `0` = geschlossen
(`CLOSED_RAD`, linear dazwischen). DOF-Reihenfolge, identisch zu den GUIs:

```
0 Kleiner (little)      -> little_1, little_2
1 Ringfinger (ring)     -> ring_1,   ring_2
2 Mittelfinger (middle) -> middle_1, middle_2
3 Zeigefinger (index)   -> index_1,  index_2
4 Daumen-Beugung        -> thumb_2, thumb_3, thumb_4
5 Daumen-Rotation       -> thumb_1
```

Die „closed"-Zielwerte sind explizit gesetzt (nicht stumpf das URDF-Limit),
weil die distalen `*_2`-Gelenke bis 180° zulassen würden — jeder Wert wird
zusätzlich an das echte URDF-Limit geklemmt (`load_limits_from_urdf`).

## `bridge.py` — Node

- **WebSocket `:8766`** (Controller): sendet beim Verbinden die aktuelle
  Konfiguration, nimmt danach Kommandos entgegen (`_handle_cmd`):
  `set_angle`, `set_force`, `set_speed` (beide Hände gemeinsam), `set_enabled`,
  `set_all_angles`, `open_hand`/`close_hand`.
- **WebSocket `:8765`** (Viewer): sendet Metadaten (Zonenlayout) beim
  Verbinden, danach nur Broadcast (`_viewer_broadcast`, 20 Hz).
- **HTTP `:8767`**: liefert die GUI-Dateien selbst aus (statt `file://`),
  weil `?autoconnect=1` je nach Betriebssystem/Opener bei `file://`-URLs
  verlorengeht.
- **`/joint_states`**: nur die 24 beweglichen Fingergelenke; `robot_state`
  muss dafür mit `publish_hand_joints:=false` laufen, sonst überschreiben
  sich beide Quellen gegenseitig (siehe
  [02_architektur.md](02_architektur.md)).
- **Streamdeck-Anbindung** (`/g1pilot/hand_action/{left,right}`, String
  `open`/`close`): `_on_hand_action` → `_apply_open_close` — dieselbe
  Funktion, die auch die Controller-GUI für `open_hand`/`close_hand`
  aufruft, damit beide Wege identisch wirken. Beim Schließen bleibt die
  Daumen-Beugung absichtlich nicht ganz auf 0 (`200` statt `0`), damit der
  Daumen nicht in die Handfläche fährt.
- **E-Stop** (`/g1pilot/emergency_stop`): deaktiviert beide Hände sofort —
  keine neuen Sollwerte mehr, aktuelle Position wird gehalten. Re-Aktivierung
  ist bewusst manuell (nächster GUI-Hauptschalter oder OPEN/CLOSE-Befehl).
- **Positionsspeicher-Anbindung**: `hand_state_pub` veröffentlicht laufend
  den Ist-Fingerzustand (`/g1pilot/hand_state/{left,right}`, 6 Werte
  0..1000) für `arm_controller` zum Speichern; `_on_hand_goal` nimmt
  gespeicherte Winkel entgegen (`/g1pilot/hand_goal/{left,right}`) und
  aktiviert die Hand dabei automatisch.

## Sim-Kontaktphysik (Stufe 2, `MujocoContactBackend`)

Die echte Hand liefert zwei unabhängige Kraftgrößen, beide in der Sim
abgebildet:

- **`force_act`** — echte Kontaktkraft je Finger (Summe seiner
  Taktil-Zonen), ungedeckelt, zeigt auch kurze Überschreitungen des Limits
  ehrlich an.
- **17 Taktil-Zonen je Hand** — die Kontakt-„Haut", platziert an den
  `*_force_sensor_*`-Frames des URDF; echte, physikbasierte Kontaktkräfte
  pro Zone. Der räumliche Feindruck *innerhalb* einer Zone ist synthetisch
  verteilt, die Zonen-Gesamtkraft ist echt.

Greifbar ist ein Objekt nur, wenn es Kollisions-Bit 2 setzt (Finger-Zonen
kollidieren nicht mit Boden/Körper, nur mit solchen Objekten und
untereinander) — daher der Sinn der GRASP-BOX-Testkugel.

**Kraftregelung** (`force_set`, Register 1498, Gramm): Übersteigt die
gemessene Kraft das Limit, fährt der Finger seinen Winkel aktiv wieder auf
(proportional zur Überschreitung), bis die Kraft darunter ist. Die dabei
erreichte Halteposition wird *gelatcht* — der Finger bleibt dort und fährt
nicht dauernd wieder ans Ziel (sonst Grenzzyklus). Der Latch löst erst bei
einem neuen, engeren Griffversuch oder beim Öffnen.

## Backend `InspireModbusBackend` (Stufe 3, echte Hände)

Reiner Standardbibliotheks-Client (`modbus_client.py`, FC03/FC16) — kein
externes Modbus-Paket nötig. Ein `HandModel` je Hand hält Soll- und
Ist-Zustand threadsicher; ein eigener Poll-Thread (`hand_poll_rate_hz`)
liest zyklisch Ist-Winkel/Kraft, alle `tactile_every`-te Runde zusätzlich
alle 17 Taktil-Zonen. Register-Adressen und Layout sind aus dem
ursprünglichen `reference/ftp_hand_controller/` übernommen.

## Konfiguration (`hand_launcher.launch.py`)

| Parameter | Bedeutung |
|---|---|
| `backend` | `sim` / `mujoco` / `modbus` |
| `interface` | DDS-Interface für `mujoco`-Backend |
| `left_host` / `right_host` | Ziel-IPs für `modbus`; leer = Seite aus |
| `modbus_port` | Modbus-TCP-Port (Default 6000) |
| `hand_poll_rate_hz` / `tactile_every` | Poll-Rate des Modbus-I/O-Threads |
| `controller_port` / `viewer_port` / `http_port` | WebSocket-/HTTP-Ports |
| `update_rate_hz` | Takt des Backend-Updates + `/joint_states`-Publish |

## Bekannte Einschränkungen

- Stufe 1 (`sim`) hat keine Kontaktphysik — Kraft/Taktil bleiben konstant 0,
  nur die Ist-Winkel werden geschwindigkeitsbegrenzt nachgeführt.
- Die per-Taxel-Matrix des Viewers innerhalb einer Zone ist ein synthetisch
  verteiltes Bild des einen echten Zonen-Kraftskalars, kein echtes
  Feindruckbild.
