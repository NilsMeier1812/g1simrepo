# Navigation — Anleitung

Richtet sich an: Anwender, die den G1 autonom von Punkt zu Punkt fahren
lassen wollen. Für die interne Funktionsweise siehe
[51_navigation_technik.md](51_navigation_technik.md).

## Überblick

Kurzfassung des Prinzips: Ein **Ziel** (Punkt in der Karte) wird gesetzt. Ein
globaler Planer (Dijkstra) berechnet einen Pfad, ein Follower fährt ihn ab,
indem er einen **virtuellen Joystick** erzeugt — denselben Signalweg, den
auch eine manuelle Bedienung nutzt. Die Autonomie läuft dadurch über exakt
denselben, erprobten Steuerpfad wie die Teleoperation.

Es ist derselbe Navigations-Stack in Simulation und auf echter Hardware; nur
die Pose-Quelle unterscheidet sich (Simulation: aus der MuJoCo-Ground-Truth,
Real: MOLA-LiDAR-Odometrie).

## Voraussetzungen

- **Simulation:** Navigation beim Start aktivieren (Menüpunkt „Navigation
  mitstarten" im Startmenü, oder `G1_ENABLE_NAV=1 ./start.sh --yes`).
- **Echter Roboter:** Livox-MID360-LiDAR + das große Docker-Image, aktiviert
  über `G1_ENABLE_LIDAR=1`.
- RViz wird bei aktiver Navigation automatisch mitgestartet (Nav-Ansicht,
  Fixed Frame `map`) — es gibt kein eigenes Navigationsfenster.

## Bedienung

**1. Roboter laufbereit machen**

```bash
ros2 topic pub --once /g1pilot/start_balancing std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /g1pilot/start_walking   std_msgs/msg/Bool "{data: true}"   # nur Sim nötig
```

Oder am Streamdeck: START BALANCING, dann (in der Sim) WALK. Erst danach
reagiert der Roboter auf Geschwindigkeitskommandos.

**2. Ziel setzen**

In RViz: Toolbar-Werkzeug „2D Goal Pose" anklicken, dann in die Karte
klicken-ziehen. Die Zieh-Richtung bestimmt die End-Ausrichtung — der
Roboter fährt zum Punkt und dreht sich dort auf der Stelle auf genau diesen
Winkel (kürzester Weg). Nur klicken ohne Ziehen ergibt Ausrichtung 0.

Per Kommandozeile:

```bash
ros2 topic pub --once /g1pilot/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}}}"
```

**3. Autonomie scharfschalten**

Am Streamdeck: Button **AUTO NAV** (erscheint automatisch, sobald der
Nav-Stack läuft). Per Kommandozeile:

```bash
ros2 topic pub --once /g1pilot/auto_enable std_msgs/msg/Bool "{data: true}"
```

Der Roboter folgt jetzt dem Pfad und stoppt selbstständig am Ziel.

**4. Abschalten / anhalten**

**AUTO NAV** erneut klicken — der Roboter stoppt sofort. Sanftes
Not-Anhalten wie immer: START BALANCING. Harter Notaus: EMERGENCY STOP.

## Worauf zu achten ist

- **Hindernisvermeidung nur für die geladene Umgebung, nicht spontan.** In
  der Simulation rastert die Karte die Objekte der geladenen Umgebung
  (siehe [61_inspire_haende_technik.md](61_inspire_haende_technik.md) für
  verwandte Szenen-Konzepte bzw. die Architektur-Doku für die Szenen-Brücke)
  — spontane Hindernisse (Menschen, verschobene Objekte) werden **nicht**
  erkannt, es gibt keine Live-Perzeption. Auf echter Hardware ohne
  entsprechendes Setup bleibt die Karte leer. Nur in freier, kontrollierter
  Fläche einsetzen.
- **Auto ODER manuell, nicht beides gleichzeitig.** Bei aktivem `auto_enable`
  hat der Navigations-Joystick Vorrang; UI-/PS4-Joystick nicht gleichzeitig
  anfassen.
- **Deadman gilt weiter.** Fällt der Navigationsstream aus, greift derselbe
  Timeout-Mechanismus wie beim manuellen Gehen.
- **Konservative Geschwindigkeitslimits** gelten auch für die Autonomie.

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| Ziel gesetzt, aber kein Pfad | Liegt eine Pose vor? `ros2 topic echo /lidar_odometry/pose_fixed` — läuft die Lokalisierung (Sim: `sim_localization`; Real: MOLA/Livox)? |
| Pfad vorhanden, Roboter fährt nicht | `auto_enable` gesendet? Roboter laufbereit (Sim: WALK; Real: BALANCING)? |
| Roboter ruckelt / zwei Geschwindigkeitsquellen | Manuellen Joystick loslassen — bei aktiver Autonomie nicht gleichzeitig manuell fahren. |
| Fährt am Ziel vorbei / dreht unruhig | Regelparameter des Followers prüfen (siehe Technik-Dokument). |
| RViz zeigt den Roboter fest im Ursprung | Nur bei aktiver Navigation bewegt sich das TF-Modell; Fixed Frame in RViz muss `map` sein. |
| „Fährt in Hindernis" | Läuft die Szenen-Brücke? Objekt in der geladenen Umgebung enthalten? Ohne eigene Umgebung bzw. auf real ist die Karte weiterhin leer — Fläche freihalten. |
