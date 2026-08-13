# Teleoperation — Technik

Richtet sich an: Entwickler, die die Bedienoberfläche oder die
Controller-Anbindung ändern wollen. Für die Bedienung siehe
[40_teleoperation_anleitung.md](40_teleoperation_anleitung.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/teleoperation/ui_interface.py` | Streamdeck-GUI (PyQt6), Hauptbedienoberfläche |
| `g1pilot/teleoperation/joystick.py` | Physischer Controller (evdev) → `/g1pilot/joy_manual` |
| `g1pilot/teleoperation/joy_mux.py` | Multiplexer manuell/autonom |
| `g1pilot/g1_gui.py` | Start-Menü (getrennt von der Streamdeck-GUI, siehe [01_installation.md](01_installation.md)) |

## `ui_interface.py` (Streamdeck)

`StreamDeck(Node)` hält alle ROS-Publisher; `ButtonGUI(QWidget)` ist die
eigentliche PyQt6-Oberfläche. Jeder Button ist ein einfacher Callback, der
auf ein Bool-/String-Topic publiziert — die Logik liegt bewusst in den
empfangenden Nodes (`arm_controller`, `loco_sim`/`loco_client`,
`inspire_hand`), nicht in der GUI.

Wichtige Muster:

- `toggle_button` / `flash_button` / `radio_loco`: generische Hilfsfunktionen
  für Toggle-, Impuls- und Radio-Buttons (z. B. BALANCING/WALK sind
  gegenseitig exklusiv, `loco_group`).
- `VirtualJoystick`: zeichnet einen Kreis, mapped Mausposition relativ zum
  Zentrum auf `vx`/`vy` in `[-1, 1]`; `publish_cmd_vel()` läuft über einen
  33-ms-Timer und sendet kontinuierlich (auch 0 im Ruhezustand) — dauerhaftes
  Senden ist harmlos und garantiert beim Loslassen sauber das
  Stop-Kommando.
- `PoseSaveDialog` / `PoseLoadDialog`: siehe
  [11_arm_manipulation_technik.md](11_arm_manipulation_technik.md), Abschnitt
  Positionsspeicher — die GUI liest den `PoseStore` direkt von der Platte
  (derselbe Prozessraum/Container wie `arm_controller`, kein ROS-Service
  nötig für die Anzeige; Schreiben läuft über das Topic
  `/g1pilot/pose_store/save`, damit `arm_controller` die einzige Quelle für
  „was ist gerade kommandiert" bleibt).
- `open_hand_guis()`: `ui_interface` läuft im Container ohne Browser. Statt
  selbst einen Browser zu starten, fasst der Node eine Trigger-Datei
  (`.gui_open_request`) im bind-gemounteten Repo an; ein Host-Watcher in
  `start.sh` beobachtet die Datei und öffnet den Browser dort, wo tatsächlich
  einer existiert (siehe [61_inspire_haende_technik.md](61_inspire_haende_technik.md)).
- **Auto-Start nur in der Simulation** (`_auto_start`, `QTimer.singleShot(3000, ...)`):
  aktiviert nach 3 Sekunden automatisch die Arme und startet BALANCING. Auf
  echter Hardware ist dieser Pfad **hart ausgeschaltet** (kein Parameter zum
  Umgehen) — der Operator entscheidet dort immer selbst, wann sich der
  Roboter bewegt (`self.sim_mode = is_sim_mode()`).
- `emergency_stop()`: setzt alle Buttons zurück und publiziert `False` auf
  sämtliche Steuertopics sowie `True` auf `/g1pilot/emergency_stop` — die
  eigentliche Wirkung (Arme drehmomentfrei, Beine `Damp()`) passiert in den
  jeweiligen Regel-Nodes, nicht in der GUI.

## `joystick.py` (physischer Controller)

`ManualJoystick(Node)` liest per `evdev` direkt von `/dev/input` (im
Docker-Compose-Profil `real` mit `privileged: true` und
`/dev/input:/dev/input` gemountet). Läuft in einem eigenen Thread
(`read_loop`), der Achsen-/Button-Zustand in eine gemeinsame, gelockte
Struktur schreibt; ein 50-Hz-Timer publiziert daraus `sensor_msgs/Joy` auf
`/g1pilot/joy_manual`. `BTN_NORTH` (Dreieck) togglet zusätzlich
`/g1pilot/auto_enable` direkt in diesem Node (steigende Flanke).

Die eigentliche Button-/Achsen-Interpretation (Deadman, Move, E-Stop,
Greifer) sitzt **nicht** hier, sondern in `loco_client.joystick_callback`
(siehe [31_loco_technik.md](31_loco_technik.md)) — `joystick.py` liefert nur
das rohe `Joy`-Signal.

## `joy_mux.py` (Auto/Manuell-Mux)

Mischt `/g1pilot/joy_manual` (Mensch) und `/g1pilot/auto_joy` (von
`nav2point`, siehe [51_navigation_technik.md](51_navigation_technik.md)) auf
ein gemeinsames Ausgabe-Topic `/g1pilot/joy`:

```python
if self.auto_enabled and self.last_auto is not None:
    self.pub.publish(self.last_auto)
elif use_manual:
    self.pub.publish(self.last_manual)
```

`use_manual` gilt, wenn entweder Autonomie aus ist oder eine manuelle
Eingabe innerhalb eines kurzen Vorrangfensters (`manual_priority_window`,
Default 50 ms) eingetroffen ist — ein kurzer manueller Eingriff kann die
Autonomie also jederzeit kurz überstimmen. Beim Ausschalten von
`auto_enable` wird einmalig ein neutraler Stop-`Joy` gesendet (sonst behielte
`loco_client`/`loco_sim` das zuletzt kommandierte Move bei).

## Datenfluss (Real, mit Navigation)

```
joystick.py ──/g1pilot/joy_manual──┐
                                    ├──► joy_mux ──/g1pilot/joy──► loco_client.joystick_callback ──► Move()
nav2point   ──/g1pilot/auto_joy────┘
```

In der Simulation läuft derselbe Pfad, nur dass `loco_sim` keinen `Joy`
liest — `joy_to_cmdvel` übersetzt `/g1pilot/joy` zusätzlich in
`/g1pilot/loco_cmd_vel` (siehe [51_navigation_technik.md](51_navigation_technik.md)).

## Bekannte Einschränkungen

- Die Streamdeck-GUI hat keinen eigenen Zustand über den ROS-Graph hinaus —
  nach einem Neustart der GUI (nicht des Stacks) zeigt sie ggf. kurzzeitig
  einen anderen Button-Zustand als der tatsächliche Roboterzustand, bis der
  nächste Status-Callback eintrifft (z. B. Marker-Follow-Default).
- `joystick.py` erwartet einen exakten Gerätenamen-Treffer
  (`dev.name == self.joystick_name`) — kein Teilstring-Match.
