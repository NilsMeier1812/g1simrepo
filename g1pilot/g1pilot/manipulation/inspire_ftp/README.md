# Inspire RH56DFTP-2 Hand-Bridge (Sim UND echte Haende)

Bringt den urspruenglich eigenstaendigen **ftp_hand_controller** in das
g1pilot-Setup — EIN Node, der je nach Backend mit der **Simulation** ODER per
**Modbus TCP** mit den **echten Haenden** spricht. Die beiden HTML-GUIs
bleiben in allen Modi identisch.

```
  hand_controller_viewer.html ──ws://…:8766──┐
                                             ├─► inspire_hand (ROS2-Node) ──► /joint_states ──► RViz
  inspire_hand_viewer.html    ──ws://…:8765──┘            │
                                                          └─► Backend (austauschbar)
```

## Die drei Backend-Stufen

| Stufe | Backend | Was passiert | Status |
|-------|---------|--------------|--------|
| **1** | `SimJointStateBackend` | Finger-Gelenke folgen der Steuerung in **RViz** (`/joint_states`). Kraft/Taktil = **0**. | ✅ |
| **2** | `MujocoContactBackend` | Finger als echte **MuJoCo-Aktuatoren** (Greifen/Kollision) via DDS `rt/inspire/cmd\|state`; **alle 17 Taktil-Zonen je Hand** aus MuJoCo-**Touch-Sensoren** (dieselben Zonen wie real). | ✅ Sim-Default |
| **3** | `InspireModbusBackend` | **ECHTE Haende** via Modbus TCP (eine IP je Hand im Roboter-LAN; Register-Map aus `reference/ftp_hand_controller/`). Liest Ist-Winkel, Kraefte (Gramm) und **alle 17 Taktil-Zonen**. | ✅ Real-Default |

Backend-Wahl: Parameter `backend` = `sim` / `mujoco` / `modbus`. Fuer
`modbus` zusaetzlich `left_host`/`right_host` (Default `192.168.123.210`/
`.211`) und `modbus_port` (6000); leere IP = Seite nicht ansteuern.
`bringup_real.launch.py` verdrahtet das ueber `G1_HAND_LEFT_HOST`/
`G1_HAND_RIGHT_HOST`/`G1_HAND_PORT` (start.sh-Real-Zweig fragt die IPs ab).

## Dateien

| Datei | Zweck |
|-------|-------|
| `joint_map.py` | **Single Source of Truth** fuer das Mapping 6 DOF (0..1000) → 24 URDF-Finger-Gelenke (rad), an URDF-Limits geklemmt. |
| `tactile.py`   | Taktil-Zonenlayout inkl. echter Modbus-Register-Adressen (3000–5123). |
| `model.py`     | `HandModel`: geteilter Soll-/Ist-Zustand einer Hand (threadsicher). |
| `backends.py`  | Die drei Backends (Stufe 1–3, s.o.). |
| `modbus_client.py` | Raw-Socket-Modbus-TCP-Client (stdlib-only, aus der Referenz uebernommen). |
| `bridge.py`    | ROS2-Node `inspire_hand`: beide WebSocket-Server + HTTP-GUI-Server + `/joint_states`. |
| `web/*.html`   | Die GUIs (Controller + Viewer), in allen Modi identisch. |

Test ohne Hardware: `g1pilot/test/test_inspire_modbus_backend.py` faehrt das
Modbus-Backend gegen einen In-Process-Fake-Server (FC03/FC16).

## Mapping-Konvention

Inspire-Winkel **1000 = offen** (Gelenk 0 rad), **0 = geschlossen**
(`CLOSED_RAD`), linear dazwischen. DOF-Reihenfolge wie in den GUIs:
`[Kleiner, Ring, Mittel, Zeige, Daumen-Beugung, Daumen-Rotation]`.

## Start

**Mit dem Sim-Stack** (Default an, via `bringup_sim.launch.py` / `start.sh`):

```bash
USE_HANDS=true ./start.sh --yes      # Hand-Bridge mit hochfahren (Default)
USE_HANDS=false ./start.sh --yes     # ohne; robot_state zeigt Finger als Default 0
```

Wenn die Bridge laeuft, gibt `robot_state` die Finger ab
(`publish_hand_joints:=false`) — sonst kollidieren beide `/joint_states`-Quellen.
Das ist in `bringup_sim.launch.py` schon verdrahtet.

**Eigenstaendig** (z.B. zum Testen, im laufenden g1pilot-Container):

```bash
ros2 launch g1pilot hand_launcher.launch.py
# manuell dann robot_state mit publish_hand_joints:=false starten!
```

**GUIs**: Die Bridge serviert beide GUIs selbst per **HTTP auf Port 8767**:

```
http://localhost:8767/hand_controller_viewer.html?autoconnect=1
http://localhost:8767/inspire_hand_viewer.html?autoconnect=1
```

Im `start.sh`-Menue kann man sie **automatisch oeffnen** lassen
(`OPEN_GUIS=true`). Da `start.sh` auf dem Host laeuft, oeffnet es nach dem
Hochfahren beide Seiten im Standard-Browser; es wartet dabei, bis die Bridge
auf `:8766` lauscht. Mit `?autoconnect=1` verbinden sich die Seiten selbst
und versuchen es alle 2 s erneut, bis die Bridge da ist (robust gegen
Reihenfolge/Neustart). file://-Oeffnen der Dateien aus `web/` geht weiterhin,
aber der `?autoconnect=1`-Query-String kommt dabei je nach Opener (xdg-open,
WSL) nicht an — darum der HTTP-Weg.

Die WebSockets bleiben unveraendert: `ws://localhost:8766` (Controller) bzw.
`ws://localhost:8765` (Viewer). Der Container nutzt `network_mode: host`,
daher ist `localhost` korrekt.

## Hinweise zur Sim

**Stufe 2 (`mujoco`, Default)** — echte Kontaktsensorik:
- Die echte Hand hat ZWEI unabhaengige Kraft-Ausgaben, beide in der Sim abgebildet:
  - **`force_act`** (Register 1582, 6/Hand, Gramm) = die **echte Kontaktkraft je
    Finger** (Summe seiner Taktil-Zonen), die die **Controller-GUI** als Kraftbalken
    zeigt. **UNGEDECKELT**: zeigt die tatsaechliche Kraft — auch wenn sie das Limit
    (`force_set`) beim Aufprall kurz uebersteigt (der Wert wird NICHT beschoenigt).
    Ohne Kontakt = 0 (ein Kraftsensor misst nur, was der Finger tatsaechlich drueckt).
  - **17 Taktil-Zonen je Hand** (Register 3000+) = die Kontakt-**Haut** (Viewer-
    Heatmap). Platziert an den `*_force_sensor_*`-Frames des URDF, palm + je Finger
    tip/nail/pad, Daumen auch mid. Echte, physikbasierte Kontaktkraefte — greift die
    Hand etwas Greifbares, leuchten die Zonen auf.
- **Greifbar = Kollisions-Bit 2**: Finger-Zonen kollidieren nur mit Objekten, die
  `contype`/`conaffinity` Bit 2 setzen (und untereinander), NICHT mit Boden/Koerper.
  Ohne so ein Objekt in Handnaehe bleiben die Kraefte 0 (nichts wird beruehrt).
- **Test-Objekt zum Ausprobieren** (Streamdeck-Button **"GRASP BOX"**, Toggle): legt
  eine kleine, fest mit jeder Handflaeche verbundene greifbare Kugel in die Griffzone.
  Beim Schliessen der Hand (~70 %) greifen alle Finger + Handflaeche zu -> man sieht
  sofort echte Griffkraefte in der GUI. Offen = kein Kontakt. Faellt nicht weg (starr
  an der Palme). Live schaltbar (kein Neustart): der Box-Koerper wird bei Inspire-
  Haenden immer eingefuegt, ist aber AUS unsichtbar+inert (~6 g, keine Kollision) und
  stoert Loco/Nav nicht. `G1_GRASP_TEST=1` startet ihn direkt AN, `G1_GRASP_BOX=0`
  laesst ihn ganz weg. Ohne ROS toggeln: `echo -n on|nc -u -w0 127.0.0.1 47901`.
- MuJoCo liefert je Zone EINEN Kraft-Skalar (Summe der Normal-Kontaktkraefte); die
  per-Taxel-Matrix des Viewers wird daraus verteilt. Der raeumliche Feindruck
  INNERHALB einer Zone ist also synthetisch, die Zonen-Gesamtkraft ist echt.
- **Kraft-Limit `force_set`** (GUI-Slider, Register 1498, Gramm) = echte
  **Kraftregelung** wie an der Hardware: die GEMESSENE Kontaktkraft (`force_act`,
  Gramm) wird laufend gegen `force_set` (gleiche Einheit) verglichen. Uebersteigt
  sie das Limit, **faehrt der Finger seinen Winkel aktiv wieder auf** (oeffnet,
  proportional zur Ueberschreitung), bis die Kraft drunter ist — man sieht den Winkel
  in der GUI zurueckgehen. Die dabei erreichte Halteposition wird **gelatcht**: der
  Finger bleibt dort und faehrt NICHT dauernd wieder ans Ziel (sonst Grenz-Zyklus
  "anfahren -> zu stark -> zurueck -> anfahren ..."). Der Latch loest erst, wenn man
  aktiv **enger** kommandiert (neuer Griff-Versuch) oder **aufmacht**. Der Servo hat
  die volle Modell-Antriebskraft; das Limit wirkt ueber die Winkel-Rueckfuehrung,
  NICHT als kuenstlicher Nm-Deckel. Die kurze Aufprall-Spitze in `force_act`
  (Stoss-Impuls) wird darum ehrlich angezeigt und dann weggeregelt — nicht auf das
  Limit geschoenigt. Regel-Parameter (`retract_open_rate`/`retract_close_rate`) im
  `MujocoContactBackend` tunebar.
- Griffkraft ueber `FKP`/`FFRC` im MJCF-Generator (`gen_inspire_ftp_hand.py`)
  tunebar; nach Aenderung das Modell neu generieren.

**Stufe 1 (`sim`, nur RViz)** — keine Kontaktphysik:
- Kraft/Taktil bleiben **0** (Platzhalter), nur die Ist-Winkel werden
  geschwindigkeitsbegrenzt nachgefuehrt (`speed`-Slider wirkt).
- `enabled` (Hauptschalter) gilt wie beim Original: ohne ihn bewegt sich nichts.
