# PREFLIGHT.md — Abnahme-Checkliste: erster echter Stand/Gang des G1

Eine **abhak-bare Gate-Liste** vor dem ersten Real-Betrieb. Sie ergaenzt das
ausfuehrliche Runbook [`REAL_TESTING.md`](REAL_TESTING.md): dort steht das *Wie*
(Schritt-fuer-Schritt), hier das *Ob-schon-sicher* (Gates, die ALLE gruen sein
muessen, bevor der naechste Block beginnt).

Architektur (bestaetigt): **Unterkoerper = Unitree-Onboard-Loco (`loco_client`),
Oberkoerper/Arme/Haende = dein Setup** — exakt der g1pilot-Original-Ansatz.

---

## A · Vor dem Einschalten (Hardware/Umgebung)

- [ ] **Gantry/Traggurt** oder ≥ 2 m freie Sturzflaeche in alle Richtungen.
- [ ] **Zweite Person** mit der **Unitree-Fernbedienung** — das ist der
      *uebergeordnete* Not-Aus, immer und ueber allem.
- [ ] **E-Stop-Semantik klar:** Streamdeck-EMERGENCY = `Damp()` = alle Motoren
      weich = **Roboter sackt zusammen**. Zum *sanften* Anhalten stattdessen
      **START BALANCING** (StopMove, Roboter bleibt stehen).
- [ ] Akku geladen, keine Kabel/Hindernisse in Fussnaehe.
- [ ] Erste Arm-Tests am **haengenden/knienden** Roboter, nicht am stehenden.

---

## B · Netzwerk & Verbindung (vor JEDEM Kommando)

- [ ] G1 per **Ethernet** am PC; PC-Interface hat eine **192.168.123.x**-IP.
      → `start.sh` markiert dieses NIC automatisch als Default.
- [ ] `ping -c3 192.168.123.161` (G1 Onboard-PC) kommt an.
- [ ] *Mit Haenden:* `ping` auf `.210`/`.211` **und** `nc -zv <ip> 6000`
      (Modbus-Port offen).
- [ ] Stack via `./start.sh` → **„ECHTER ROBOTER"**, Interface bestaetigen,
      Sicherheitsabfrage **`REAL`** getippt.
- [ ] **DDS-Check** im Container:
      `ros2 run g1pilot dds_check --interface $ROBOT_INTERFACE`
      → `rt/lowstate` ~**500 Hz**, `mode_machine` **≠ 0**, Arm-Winkel aendern
      sich, wenn du die Arme von Hand bewegst.

> **Abbruch-Kriterium B:** Kein `lowstate` → falsches Interface / nicht im
> 192.168.123.x-Netz / Kabel. Nicht weitermachen.

---

## C · „Bewegt-sich-nichts-von-selbst"-Garantien (im Code verifiziert)

Diese Punkte musst du nicht testen — sie sind im Code so gebaut. Zur Beruhigung
vor dem ersten Start:

- [x] **Kein Auto-Damp beim Start:** `loco_client` laeuft mit
      `damp_on_init=False` — der Node greift beim Hochfahren NICHT in den
      Roboter-Zustand ein (ein stehender G1 wuerde bei `Damp()` zusammensacken).
- [x] **Kein Auto-Start:** Die Streamdeck-UI publiziert von selbst NICHTS; der
      Roboter bewegt sich erst nach deinen Klicks (START → START BALANCING → …).
- [x] **WALK ist gesperrt, bis balanciert:** `start_walking` wird mit Warnung
      ignoriert, solange nicht `START → START BALANCING` gelaufen ist.
- [x] **Weiche Arm-Uebergabe:** arm_sdk-Gewicht rampt beim ENABLE 0→1 und beim
      DISABLE 1→0 ueber **2.0 s** (`arm_weight_ramp_up_s/down_s`) — kein Ruck.

---

## D · Erst-Stand-Abnahme (gesichert, Gantry!)

- [ ] **START** → Log `loco_client`: „Switched to FSM ID 4 (Standby)".
- [ ] **START BALANCING** → StandHeight-Rampe + `BalanceStand(1)` + `Start()`.
      **Erwartung:** Roboter steht selbststaendig und balanciert.
- [ ] **Stoer-Test:** leichtes Anstupsen wird ausgeglichen.
- [ ] **Sanft anhalten** funktioniert: nochmal **START BALANCING** (bleibt
      stehen), NICHT den EMERGENCY-Knopf zum „normalen" Stoppen benutzen.

> **FSM-ID-Vorbehalt:** Die Sequenz stammt aus dem g1pilot-Original. Bei neuerer
> G1-Firmware ggf. andere FSM-IDs → hier abbrechen und IDs in `loco_client.py`
> gegen die aktuelle Unitree-Doku pruefen (`loco_doc.md`).

---

## E · Geh-Abnahme (erst NACH erfolgreicher Stand-Abnahme)

- [ ] **Arme DISABLED lassen** fuer die ersten Geh-Tests. Grund: bei aktiver
      Manipulation haelt `arm_controller` ueber `rt/arm_sdk` auch die **Taille**
      fest — die will der Onboard-Loco beim Gehen aber mitbewegen. Natuerlicher
      Armschwung der Unitree-Steuerung ist fuer den ersten Gang besser.
- [ ] **PS4-Pfad zuerst** (physischer Deadman): Controller verbunden
      (`JOYSTICK_NAME` passt, Log „Joystick found"). **Deadman HALTEN** (Button 8)
      + Stick → Roboter geht; loslassen → StopMove.
- [ ] **Button-Indizes verifizieren:** Balance = Button 6, Deadman = Button 8
      (weichen bewusst vom Original ab). Beim ersten Verbinden mit
      `ros2 topic echo /g1pilot/joy` gegenchecken, dass DEIN Controller diese
      Indizes liefert.
- [ ] **Deadman-Test (Pflicht!):** Waehrend des Gehens das Streamdeck-Fenster
      schliessen → Roboter stoppt von selbst nach **≤ 0.5 s**
      (`cmd_vel_timeout`) und bleibt balanciert. (Darf NICHT weiterlaufen.)
- [ ] **Walk-Limits konservativ:** vx=0.4 / vy=0.3 / vyaw=0.4 m|rad/s
      (`G1_MAX_VX/VY/VYAW`) — fuer die ersten Gaenge so lassen.

---

## Not-Aus-Reflex (auswendig)

| Situation | Richtige Reaktion |
|---|---|
| „Sanft anhalten" beim Gehen | **START BALANCING** (StopMove, bleibt stehen) |
| Es ruckt/schwingt an den Armen | **ENABLE MANIPULATION aus** (Gewicht rampt 2 s runter) |
| Echte Gefahr / Sturz droht | **Unitree-Fernbedienung** (uebergeordnet) — Streamdeck-EMERGENCY nur, wenn Zusammensacken ok ist |

Detaillierter Ablauf & Troubleshooting: [`REAL_TESTING.md`](REAL_TESTING.md).
