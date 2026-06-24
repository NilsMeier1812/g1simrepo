# Inspire RH56DFTP-2 Hand-Bridge (Sim-Integration)

Bringt den urspruenglich eigenstaendigen **ftp_hand_controller** in das
g1pilot-MuJoCo-Setup. Statt per **Modbus TCP** mit den echten Haenden zu reden,
spricht die Bridge jetzt mit der **Simulation**. Die beiden HTML-GUIs bleiben
unveraendert.

```
  hand_controller_viewer.html ──ws://…:8766──┐
                                             ├─► inspire_hand (ROS2-Node) ──► /joint_states ──► RViz
  inspire_hand_viewer.html    ──ws://…:8765──┘            │
                                                          └─► Backend (austauschbar)
```

## Zwei-Stufen-Plan

| Stufe | Backend | Was passiert | Status |
|-------|---------|--------------|--------|
| **1** | `SimJointStateBackend` | Finger-Gelenke folgen der Steuerung in **RViz** (`/joint_states`). Kraft/Taktil = **0**. | ✅ aktiv |
| **2** | `MujocoContactBackend` | Finger als echte **MuJoCo-Aktuatoren** (Greifen/Kollision); Taktil/Kraft aus MuJoCo-Kontaktkraeften **gefaked**. | 🔧 vorbereitet (Stub) |

Der Umbau auf Stufe 2 ist ein reiner Backend-Tausch (`backend:=mujoco`) plus das
Einbringen der Inspire-Finger ins MuJoCo-Scene-XML — der Rest (GUIs, Mapping,
Node) bleibt gleich.

## Dateien

| Datei | Zweck |
|-------|-------|
| `joint_map.py` | **Single Source of Truth** fuer das Mapping 6 DOF (0..1000) → 24 URDF-Finger-Gelenke (rad), an URDF-Limits geklemmt. |
| `tactile.py`   | Taktil-Zonenlayout (fuer Viewer-Meta + Null-Platzhalter). |
| `model.py`     | `HandModel`: geteilter Soll-/Ist-Zustand einer Hand (threadsicher). |
| `backends.py`  | `SimJointStateBackend` (Stufe 1) + `MujocoContactBackend` (Stub, Stufe 2). |
| `bridge.py`    | ROS2-Node `inspire_hand`: beide WebSocket-Server + `/joint_states`. |
| `web/*.html`   | Die unveraenderten GUIs (Controller + Viewer). |

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

**GUIs**: Im `start.sh`-Menue kann man die GUIs **automatisch oeffnen** lassen
(`OPEN_GUIS=true`). Da `start.sh` auf dem Host laeuft, oeffnet es nach dem
Hochfahren beide HTML-Seiten im Standard-Browser und verbindet sie selbst (per
`?autoconnect=1`); es wartet dabei, bis die Bridge auf `:8766` lauscht.

Manuell: Die HTML-Dateien aus `web/` im Browser oeffnen und verbinden auf
`ws://localhost:8766` (Controller) bzw. `ws://localhost:8765` (Viewer). Der
Container nutzt `network_mode: host`, daher ist `localhost` korrekt. Optionaler
Auto-Connect auch manuell: `…/hand_controller_viewer.html?autoconnect=1`.

## Hinweise zur Sim (Stufe 1)

- **Kraft/Taktil sind 0** — es gibt keine Sensorik in der Sim. Der Viewer laeuft
  trotzdem (zeigt Nullen). Echte Werte kommen erst mit Stufe 2.
- Die Ist-Winkel werden geschwindigkeitsbegrenzt nachgefuehrt
  (`speed`-Slider wirkt), damit die GUI eine plausible Bewegung zeigt.
- `enabled` (Hauptschalter) gilt wie beim Original: ohne ihn bewegt sich nichts.
