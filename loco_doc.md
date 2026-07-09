# Wie die Locomotion in der Sim funktioniert

> Dieses Dokument beantwortet die Fragen **„Wie funktioniert das?"**, **„Wie hast du
> das gemacht?"** und **„Womit läuft das jetzt?"** für den Stehen → Laufen → Stehen-Teil
> des G1Pilot-MuJoCo-Stacks. Es ist bewusst erklärend geschrieben — man muss den Code
> nicht gelesen haben, um zu verstehen, was passiert und warum.

---

## TL;DR

Auf dem **echten** Unitree G1 läuft das Gehen/Balancieren auf einem **Onboard-Controller**
von Unitree, den man nur über eine High-Level-API (`LocoClient.BalanceStand/Move`)
anspricht. In **MuJoCo gibt es diesen Onboard-Controller nicht** — wir bekommen nur die
rohe Low-Level-Schnittstelle (Gelenke + IMU rein, Motor-Befehle raus).

Deshalb haben wir einen **sim-eigenen Ersatz-Controller** gebaut: den ROS-2-Node
**`loco_sim`**. Er kombiniert zwei Regler und schaltet je nach Situation um:

| Aufgabe | Womit gelöst |
|---|---|
| **Laufen** | eine **vortrainierte RL-Policy** (`motion.pt` aus `unitree_rl_gym`) — wir trainieren *nichts* selbst |
| **Stehen / Balancieren** | ein **modellbasierter PD-Regler** (Knöchel-/Hüft-Strategie auf IMU-Feedback) |
| **Übergang Laufen → Stehen** | ein **Stepping-Stop**: die Policy bremst aktiv ab, dann sanfter Handoff an den PD |

Beides spricht **dieselbe Unitree-DDS-Schnittstelle** wie der echte Roboter — derselbe
Code-Pfad, nur einmal gegen MuJoCo und einmal gegen die echte Hardware.

---

## 1 · Die Ausgangslage: was die Sim liefert (und was nicht)

MuJoCo simuliert die Physik des G1 (29 Gelenke, je ein Drehmoment-Motor). Eine **Bridge**
(`unitree_sdk2py_bridge.py`) übersetzt zwischen MuJoCo und der Unitree-DDS-Welt:

```
MuJoCo Physik  ──►  Bridge  ──►  rt/lowstate   (Gelenk q/dq/τ, IMU-Quaternion/Gyro/Acc)
MuJoCo Physik  ◄──  Bridge  ◄──  rt/lowcmd      (pro Motor: q, dq, τ, kp, kd)
```

Jeder Motor wird in MuJoCo als PD-Regler mit Vorsteuer-Moment angesteuert:

```
ctrl_i = τ_i + kp_i · (q_soll_i − q_ist_i) + kd_i · (dq_soll_i − dq_ist_i)
```

**Entscheidend — was es NICHT gibt:**
- **Keinen Onboard-Balance-/Loco-Controller.** Auf dem echten G1 macht den Unitree.
  Ohne ihn fällt der Roboter einfach um, wenn niemand die Beine regelt.
- **Keine Basis-Linear-Geschwindigkeit in `rt/lowstate`.** Die echte Schnittstelle liefert
  nur IMU (Orientierung, Drehrate, Beschleunigung) + Gelenkwerte — **nicht**, wie schnell
  der Körper durch den Raum gleitet. Das ist später wichtig (siehe Stepping-Stop).

Bisher „stand" der Roboter in der Sim nur, weil ein **Weld** (`HOLD_BASE`) den Oberkörper
künstlich an der Welt festhielt — das ist kein Balancieren, sondern „an einem Faden
aufgehängt". `loco_sim` ersetzt diesen Faden durch echte Beinregelung; der Weld wird
gelöst, sobald `loco_sim` übernimmt.

---

## 2 · Womit läuft das Laufen? — Die RL-Policy

### Was es ist
Eine **fertig trainierte** neuronale Policy aus dem offiziellen
[`unitree_rl_gym`](https://github.com/unitreerobotics/unitree_rl_gym)-Projekt, abgelegt als
TorchScript-Datei `g1pilot/policies/g1/motion.pt`. Wir **trainieren selbst nichts** — wir
*deployen* nur die fertige Policy, exakt wie Unitrees `deploy_real.py` es auf der echten
Hardware tut. Die zugehörige Konfiguration (`policies/g1/g1.yaml`) ist 1:1 aus dem
Original übernommen, damit Gelenk-Reihenfolge, Skalierungen und Gains **exakt** zu den
Gewichten passen — die häufigste Fehlerquelle beim Policy-Deploy.

### Wie sie funktioniert
Die Policy ist ein kleines **rekurrentes Netz (LSTM)**. Bei jedem Regelschritt (50 Hz):

1. **Observation bauen** (47 Zahlen) aus `rt/lowstate`:

   | Anteil | Größe | Quelle |
   |---|---|---|
   | Basis-Drehrate | 3 | IMU-Gyro |
   | projizierte Gravitation | 3 | aus IMU-Quaternion (zeigt die Neigung) |
   | Velocity-Kommando | 3 | Joystick: vx, vy, vyaw (normiert) |
   | Gelenkpositionen | 12 | Beine, relativ zur Standpose |
   | Gelenkgeschwindigkeiten | 12 | Beine |
   | letzte Action | 12 | das vorige Netz-Ausgabe |
   | Gait-Phase | 2 | sin/cos eines Lauf-Takts |

2. **Inferenz:** `action(12) = policy(observation)` — eine Ausgabe pro Bein-Gelenk.
3. **Auf Motoren abbilden:** `q_soll = Standpose + action · 0.25`, mit den Policy-Gains
   (kp/kd aus `g1.yaml`) auf `rt/lowcmd` geschrieben.

Das **Velocity-Kommando** ist die Steuerschnittstelle: Joystick → `/g1pilot/loco_cmd_vel`
→ fließt in die Observation → die Policy läuft in die gewünschte Richtung.

### Die eine wichtige Eigenheit
Diese Policy hat **keine Basis-Linear-Geschwindigkeit** in der Observation (gibt es ja nicht,
siehe §1). Sie ist damit „geschwindigkeitsblind": Bei Kommando = 0 **bleibt sie nicht
stehen**, sondern marschiert auf der Stelle weiter und driftet langsam weg. Sie *fällt*
nicht — aber sie *steht* auch nicht sauber am Platz. Genau deshalb gibt es einen zweiten
Regler fürs Stehen.

---

## 3 · Womit läuft das Stehen? — Der modellbasierte PD-Balancer

Fürs **saubere, driftfreie Stehen am Platz** nutzt `loco_sim` **nicht** die Policy, sondern
einen kleinen **modellbasierten Regler** (`_send_balance_pd`). Idee: eine klassische
**Knöchel-/Hüft-Balance-Strategie**, wie sie in der Biped-Regelung Standard ist.

**So funktioniert er:**
1. Aus dem IMU-Quaternion wird die **projizierte Gravitation** berechnet — aufrecht ist
   `[0, 0, −1]`; kippt der Roboter nach vorn, wird `gx > 0`, nach links `gy > 0`. Das ist
   das gemessene **Neigungs-Signal**.
2. Ein **Posture-PD** hält die Beine steif auf der festen Standpose (Steifigkeit
   ~10× die weichen Lauf-Gains — *der* Haupthebel; weiche Gains geben unter Last nach und
   der Körper kippt vornüber).
3. Zusätzlich legt der Regler ein **Vorsteuer-Drehmoment** auf Knöchel (primär) und Hüfte
   (sekundär), das proportional zur gemessenen Neigung + Drehrate **gegen** das Kippen
   wirkt — bis ans Motorlimit (±50 Nm Knöchel). Direkt als Moment, weil die weichen
   Positions-Gains allein zu wenig Autorität hätten (~20 Nm, nötig ~50 Nm).

**Warum dieser Regler statt der Policy fürs Stehen?**
- Er braucht **nur IMU + Gelenke** — also exakt das, was die echte Schnittstelle liefert.
- Er ist **station-keeping per Konstruktion**: Er regelt auf eine feste Pose, also driftet
  er nicht (validiert: ~15 mm Drift in 12 s).
- Er **reagiert auf gemessene Störungen** — bewegt man die Arme oder stößt den Roboter an,
  sieht er die Neigung über die IMU und gleicht aus. Das ist auch die Grundlage dafür,
  dass die **Arme frei** bleiben: Sie werden *nicht* zum Balancieren gebraucht, sondern
  können eine leichte Kiste tragen, während die Beine den Stand halten.

---

## 4 · Der schwierige Teil: Laufen → Stehen (Stepping-Stop)

Das ist der Übergang, der am meisten Arbeit gekostet hat. Das Problem: Ein **laufender**
Biped hat Schwung. Lässt man den Joystick los, kann ein **statischer** PD-Regler diesen
ankommenden Schwung **nicht fangen** — besonders nicht seitwärts/rückwärts. Setzt man die
steife Standpose zu früh, schlägt es den Roboter um.

**Die Lösung — aktiv abbremsen, statt hart umzuschalten:**

1. **Stick zentriert nach echtem Laufen** → `loco_sim` geht **nicht** sofort auf PD, sondern
   lässt die Policy **gegen die eigene Bewegung kommandieren**: Brems-Kommando
   ≈ `−brake_gain · Basis-Geschwindigkeit`. Der Roboter macht ein paar **Bremsschritte**.
2. **Handoff erst im richtigen Moment:** Übergabe an den PD-Stand erst, wenn der Roboter
   **langsam genug** ist *und* im **Doppelstütz** steht (beide Füße belastet). Dann ist er
   physikalisch in einem fangbaren Zustand.
3. **Sanfter Eintritt:** Beim Wechsel rampt der PD über ~0.4 s die Soll-Pose von der
   aktuellen Beinstellung → Standpose (sonst „reißt" der steife PD die Beine aus dem
   Schritt → Lurch).

**Das Henne-Ei-Problem dabei:** Zum Abbremsen braucht man die **Basis-Geschwindigkeit** —
die `rt/lowstate` aber gar nicht liefert (§1). Wie wir das gelöst haben, steht im nächsten
Abschnitt.

---

## 5 · Der Trick mit dem `reserve[]`-Feld (Sim-interne Sensorik)

Damit der Stepping-Stop funktioniert, braucht `loco_sim` zwei Größen, die die echte
Schnittstelle nicht hat: **Fuß-Kontaktkraft** (für „Doppelstütz?") und
**Basis-Geschwindigkeit** (fürs Bremsen). In der Sim kennt MuJoCo beide exakt.

Die `LowState_`-Nachricht hat aber **keine Felder** dafür. Lösung: Wir nutzen das sonst
ungenutzte **`reserve[4]`-Feld** der Nachricht als Schmuggelkanal. Die Bridge schreibt
beim Publishen:

```
reserve[0] = Fuß-Normalkraft links   [N]
reserve[1] = Fuß-Normalkraft rechts  [N]
reserve[2] = Basis-vx (Welt), kodiert als int((v+10)·1000)
reserve[3] = Basis-vy (Welt), kodiert als int((v+10)·1000)
```

`loco_sim` dekodiert das wieder. Ist das Feld leer (z. B. echte Hardware), fällt der Code
automatisch auf einen einfacheren **zeitbasierten** Handoff zurück.

> **Warum ist das ehrlich/legitim?** `loco_sim` ist ein **reiner Sim-Stellvertreter** und
> läuft **nie** auf der echten Hardware (dort übernimmt Unitrees Onboard-Loco). In der Sim
> ist die wahre Geschwindigkeit also eine zulässige Hilfsgröße — wir bauen ja gerade das
> nach, was der echte Roboter durch seinen internen Zustandsschätzer ohnehin kennt.
> Auf dem echten G1 schätzt ein EKF diese Geschwindigkeit aus IMU + Bein-Kinematik; das in
> der Sim mit verrauschtem Schätzer nachzubauen wurde getestet und **verschlechterte** das
> Verhalten — die Ground-Truth ist hier das pragmatisch bessere Mittel.

---

## 6 · Wie alles zusammenspielt: die Zustandsmaschine

`loco_sim` ist eine kleine FSM. Gesteuert wird sie über dieselben ROS-Topics, die auf dem
echten Roboter der `loco_client` abonniert — die Sim ist also drop-in-kompatibel zur
Bedienung.

| Zustand | Was läuft | Auslöser |
|---|---|---|
| **HOLD** | Standby: Beine steif auf Standpose, Bridge hält die Basis (Weld) | `…/start` |
| **BALANCE** | modellbasierter PD-Stand, Oberkörper/Arme frei | `…/start_balancing` |
| **RUN** | RL-Walking-Policy; Geschwindigkeit per Joystick | `…/start_walking` + `…/loco_cmd_vel` |
| **DAMP** | Not-Aus: alle Motoren weich (kp=0), sanftes Hinsetzen, Arme aus | `…/emergency_stop` |

**Typischer Zyklus:**

```
START BALANCING ─► [BALANCE]  steht frei, Arme manipulieren
       │
   START WALKING ─► [RUN]  Joystick = vorwärts/seitwärts/drehen
       │
 Joystick los ──► Stepping-Stop (Policy bremst) ──► Doppelstütz ──► [BALANCE]
```

Zwei Zusatzmechaniken laufen automatisch mit:
- **CATCH FALLS** (Default an): Droht im PD-Stand ein Sturz (Neigung zu groß), schaltet
  `loco_sim` kurz auf die **steppfähige Policy** um (fängt größere Stöße) und kehrt nach
  dem Auffangen zum PD zurück.
- **Bridge-Weld-Management:** `loco_sim` meldet der Bridge per Code (im `rt/lowcmd`), ob
  gerade balanciert wird — die Bridge löst dann den Basis-Weld bzw. hält ihn im Standby.

---

## 7 · Warum es zwei Regler sind (und nicht „einfach die Policy")

Eine naheliegende Frage: Warum nicht die RL-Policy für *alles* nehmen?

| | RL-Policy | Modellbasierter PD |
|---|---|---|
| **Laufen** | ✅ robust, alle Richtungen | ❌ kann nicht laufen |
| **Stehen am Platz** | ❌ driftet weg (geschwindigkeitsblind) | ✅ driftfrei, station-keeping |
| **Arme frei lassen** | bedingt | ✅ reagiert auf gemessene Last |
| **braucht** | IMU + Gelenke | IMU + Gelenke |

Jeder Regler ist gut in genau dem, worin der andere schwach ist. `loco_sim` nimmt deshalb
**das Beste aus beiden** und der Stepping-Stop ist die Brücke dazwischen. Das spiegelt
übrigens auch, wie der echte G1 intern arbeitet (geschätzter Zustand + Stepping-/Capture-
Strategie + steifer Stand-Modus) — nur dass Unitree das in einem einzigen, hochoptimierten
Onboard-Controller bündelt, den wir in der Sim nicht haben.

---

## 8 · Womit läuft das technisch?

- **Sprache/Framework:** Python, ROS 2 Humble, läuft im `g1pilot_sim`-Container.
- **Policy-Inferenz:** PyTorch (TorchScript, CPU, Single-Thread — das LSTM ist winzig,
  Multi-Threading wäre hier nur Overhead).
- **Physik:** MuJoCo (Python-Bindings), 1 kHz; `loco_sim` regelt mit 50 Hz (Decimation 20).
- **Kommunikation:** Unitree SDK2 über CycloneDDS (Domain 1, Loopback `lo`) — identisch
  zum echten Roboter.
- **Determinismus:** Im Lockstep-Modus taktet die Regelschleife auf den eingehenden
  `rt/lowstate`, sodass die Policy garantiert ihre trainierte 50-Hz-Rate sieht,
  unabhängig davon, wie schnell der PC ist.

---

## 9 · Grenzen & Ehrlichkeit

- **`loco_sim` ist sim-only.** Es ist ein Stellvertreter für den Onboard-Loco des echten
  G1, kein Controller für echte Hardware. Auf dem echten Roboter läuft weiter Unitrees
  Onboard-High-Level.
- **Die Walking-Policy** ist die kanonische `unitree_rl_gym`-Policy ohne explizites
  Stand-Verhalten — der driftfreie Stand kommt ausschließlich vom modellbasierten PD.
- **Sehr schnelle Diagonal-Stopps** bleiben grenzwertig: Während der Bremsschritte kann es
  bei extremem Schwung noch fallen. Geradeaus/seitwärts/rückwärts stoppt es zuverlässig.
- **Der Stepping-Stop hängt an der Sim-internen Geschwindigkeit** (`reserve[]`). Ohne sie
  (z. B. echte Hardware) bleibt nur der gröbere zeitbasierte Handoff.

---

## Weiterführend

- [`README.md`](README.md) — Überblick, Ersteinrichtung, Schnellstart, Architektur
- [`g1pilot/CHEATS.md`](g1pilot/CHEATS.md) — alle Steuer-Topics + Live-Tuning-Parameter
- [`g1pilot/TESTING_SIM.md`](g1pilot/TESTING_SIM.md) — Test-Ablauf
- Code: [`g1pilot/g1pilot/navigation/loco_sim.py`](g1pilot/g1pilot/navigation/loco_sim.py)
  (der Controller) und
  [`unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py`](unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py)
  (die Bridge mit dem `reserve[]`-Kanal)
