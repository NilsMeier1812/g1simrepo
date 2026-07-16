# Eigenes Projekt an die G1-MuJoCo-Sim anbinden

> **Für wen ist das?** Du hast ein **eigenes, komplett unabhängiges Projekt**
> (eigener Controller, eigene RL-Policy, eigenes Teleop, eigene Motion-Planung …)
> und willst es **vor dem Test am echten Unitree G1** erst in einer Simulation
> ausprobieren. Diese Anleitung erklärt, was du tun musst, um **nur die
> MuJoCo-Sim aus diesem Repo als G1-Ersatz** zu benutzen — ohne den g1pilot-Stack.
>
> Du brauchst **nichts** aus `g1pilot/` zu verstehen oder zu starten. Der ganze
> ROS-2-/RViz-/Teleop-Teil ist *mein* Projekt. Für dich ist nur der Ordner
> `unitree_mujoco/` relevant.

---

## 1 · Das mentale Modell (bitte zuerst lesen)

Die Sim in diesem Repo ist **kein ROS-Framework, an das du dich koppelst**. Sie
ist ein **Stellvertreter für den echten G1 auf der Low-Level-Schnittstelle**:

```
   DEIN PROJEKT                        DIESE SIM (unitree_mujoco)
 ┌───────────────┐                   ┌────────────────────────────┐
 │ dein Code     │  ── rt/lowcmd ──▶ │ unitree_sdk2py_bridge      │
 │ (was auch     │                   │   wandelt Befehle in Motor- │
 │  immer)       │  ◀─ rt/lowstate ─ │   Drehmoment, steppt MuJoCo │
 └───────────────┘                   │   publiziert Gelenke + IMU  │
                                     └────────────────────────────┘
        │                                        │
        └──────────── Unitree SDK2 / DDS ────────┘
              (CycloneDDS, GENAU wie am echten Roboter)
```

**Der entscheidende Punkt:** Die Sim spricht **exakt dieselbe DDS-Schnittstelle
wie der echte G1**. Alles, was dein Projekt können muss, um an die Sim
anzudocken, ist: **Unitree-SDK2-Nachrichten senden/empfangen** (`rt/lowcmd`,
`rt/lowstate`). Wenn dein Projekt ohnehin schon den echten G1 über das
Unitree SDK2 (Python oder C++) oder über `unitree_ros2` ansteuert, dann ist die
Anbindung an die Sim **nur eine Frage der DDS-Domain und des Netzwerk-Interface**
— kein Code-Umbau.

Die Konsequenz daraus:

- **Wenn dein Code schon G1-tauglich ist** (spricht `rt/lowcmd`/`rt/lowstate`
  im `unitree_hg`-Format): Du zeigst deine DDS-Verbindung auf **Domain 1 /
  Interface `lo`** statt auf das Roboter-Ethernet — fertig. Siehe [Abschnitt 5](#5--dein-projekt-anschließen).
- **Wenn dein Code noch gar keine G1-Schnittstelle hat**: Dann ist die Sim genau
  der richtige Ort, um sie zu bauen — was gegen die Sim funktioniert, funktioniert
  1:1 gegen die echte Hardware, weil es dieselbe API ist.

> ⚠️ **Was die Sim NICHT ist:** Sie ist **kein High-Level-Locomotion-System.**
> Am echten G1 läuft das Laufen/Balancieren auf Unitrees Onboard-Controller
> (`loco_client`, High-Level). Die Sim liefert dir **nur die Low-Level-Ebene**
> (Motoren + Sensoren). Wenn dein Projekt Laufen braucht, musst du den
> Balance-/Lauf-Regler **selbst mitbringen** (deine eigene RL-Policy o. Ä.) —
> die Sim gibt dir dafür einen frei stehenden Zweibeiner, den du über `rt/lowcmd`
> torque-/PD-regelst. (Mein `loco_sim` ist nur *mein* Sim-Stellvertreter dafür und
> für dich irrelevant.)

---

## 2 · Was du auf deiner Seite brauchst

Die Sim und dein Projekt reden über **CycloneDDS**. Am einfachsten ist es, wenn
**beide auf derselben Maschine** laufen und über das Loopback-Interface `lo`
kommunizieren (so ist die Sim per Default konfiguriert).

| Bereich | Anforderung |
|---|---|
| **OS** | Linux (getestet Ubuntu 22.04 / 24.04) oder Windows via **WSL2**. Kein GPU/CUDA nötig — reine CPU-Physik. |
| **Für die Sim selbst** | Python 3.8+, `mujoco` (pip), das Unitree-SDK2-Python aus diesem Repo (`unitree_sdk2_python/`). ODER: Docker (dann ist alles im Image). |
| **Für dein Projekt** | Irgendein Weg, Unitree-SDK2-DDS zu sprechen: **`unitree_sdk2_python`** (Python), **`unitree_sdk2`** (C++), oder **`unitree_ros2`** (ROS 2 Humble). |
| **DDS-Middleware** | CycloneDDS. Bei ROS 2: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. |
| **RAM / Disk** | ≥ 8 GB RAM, ~10 GB frei (v. a. wenn du den Docker-Weg gehst). |

> **Wichtig zu `lo`:** Das Loopback-Interface funktioniert **nur maschinen-intern**.
> Sim und dein Projekt müssen dann auf **demselben Rechner** (bzw. derselben
> WSL2-Distro / demselben Docker-Host-Netzwerk) laufen. Für getrennte Rechner
> siehe [Abschnitt 7](#7--netzwerk-varianten-gleiche-vs-getrennte-maschine).

---

## 3 · Die Sim starten — zwei Wege

Du brauchst **nur die MuJoCo-Sim**, nicht meinen g1pilot-Container.

### Weg A — Python direkt (am schlanksten, empfohlen zum Verstehen)

```bash
git clone https://github.com/nilsmeier1812/g1simrepo.git
cd g1simrepo

# Unitree-SDK2-Python installieren (liegt im Repo, keine Submodule)
cd unitree_sdk2_python
pip install -e .
cd ..

# MuJoCo installieren
pip install mujoco

# Sim starten
cd unitree_mujoco/simulate_python
python3 unitree_mujoco.py
```

Es öffnet sich das MuJoCo-Viewer-Fenster mit dem G1, und im Terminal werden die
Gelenke/Sensoren gelistet (`PRINT_SCENE_INFORMATION`). Ab jetzt publiziert die
Sim `rt/lowstate` und wartet auf `rt/lowcmd`.

> Damit `ROBOT = "g1"` aktiv ist: `unitree_mujoco/simulate_python/config.py`
> hat das bereits als Default gesetzt. Prüfe es dennoch (siehe [Abschnitt 4](#4--die-konfiguration-die-du-anfassen-musst)).

### Weg B — Docker (nur der Sim-Container)

Wenn du dir die lokale Installation sparen willst, kannst du **allein den
MuJoCo-Container** aus dem Compose starten (nicht den ganzen Stack):

```bash
cd g1simrepo/g1pilot
xhost +local:docker                       # X11 fürs Viewer-Fenster
HOLD_BASE_MODE=off SIM_LOCKSTEP=0 \
  docker compose --profile sim up mujoco-sim
```

`--profile sim ... mujoco-sim` startet **nur** den Sim-Service (der
`g1pilot-sim`-Container bleibt aus). Der Container läuft mit `network_mode: host`,
d. h. sein DDS liegt direkt auf dem Host — dein Projekt kann von außen auf
`lo`/Domain 1 andocken.

> Die Env-Variablen `HOLD_BASE_MODE`, `SIM_LOCKSTEP` etc. steuern das Verhalten —
> siehe nächster Abschnitt. Für einen eigenen Loco-/Ganzkörper-Controller willst
> du fast sicher `HOLD_BASE_MODE=off`.

---

## 4 · Die Konfiguration, die du anfassen musst

Alles in **`unitree_mujoco/simulate_python/config.py`** (bei Weg B teils per
Env-Variable überschreibbar).

| Parameter | Setzen auf | Warum |
|---|---|---|
| `ROBOT` | `"g1"` | Wählt das G1-Modell **und** das `unitree_hg`-Nachrichtenformat (29 DoF). Muss stimmen, sonst passen Message-Typen/Joint-Zahl nicht. |
| `DOMAIN_ID` | `1` | DDS-Domain. **Dein Projekt muss dieselbe Domain benutzen.** |
| `INTERFACE` | `"lo"` | DDS-Netzwerk-Interface. `lo` = gleiche Maschine. Für getrennte Rechner echtes Interface eintragen (Abschnitt 7). |
| `USE_JOYSTICK` | `0` | **Zwingend 0**, wenn kein Gamepad am Rechner/Container hängt — sonst stirbt der Sim-Thread. |
| `HOLD_BASE_MODE` | `"off"` **für Loco**, `"weld"` für Arm-only | `weld` schweißt den Oberkörper starr an die Welt (Basis fix) → gut, wenn du **nur die Arme** testest, ohne selbst balancieren zu müssen. `off` = Basis völlig frei → **das brauchst du, wenn dein Regler die Beine/Balance selbst macht.** |
| `SIMULATE_DT` | `0.001` | 1 kHz Physik. Nötig für Stabilität des extern gerechneten PD-Dämpfungsterms (`kd`). Bei Echtzeit-Problemen `0.002`. |
| `SIM_LOCKSTEP` | `0` oder `1` | `1` = deterministische Regelrate (genau *decimation* Physikschritte pro `rt/lowcmd`, dann ein `rt/lowstate`). Nur sinnvoll, wenn **dein Controller im Lockstep mitspielt** („ein Kommando pro State"). Wenn unsicher: mit `0` (Free-Running) anfangen. |
| `SIM_REALTIME_FACTOR` | `1.0` | `<1.0` bremst die Sim unter Echtzeit (hilft langsamen PCs mit ratenabhängiger RL-Policy). |

> **`HOLD_BASE_MODE` ist die wichtigste Entscheidung für dich:**
> - Du testest **Manipulation / Arme**, Roboter soll einfach stehen bleiben → `weld`.
> - Du testest **Laufen / Balance / Ganzkörper** mit deinem eigenen Regler → `off`
>   (die Sim gibt dir einen frei stehenden G1; **fällt ohne aktive Balance in 1–2 s um** —
>   das ist physikalisch korrekt und genau der Testfall).

---

## 5 · Dein Projekt anschließen

Die Sim macht **exakt das, was der echte G1 low-level macht**. Für dein Projekt
heißt das konkret:

### 5.1 DDS initialisieren — auf die Sim zeigen

**Python (`unitree_sdk2_python`):**

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
# Domain 1, Interface lo  ->  verbindet mit der Sim (statt Roboter-Ethernet)
ChannelFactoryInitialize(1, "lo")
```

**C++ (`unitree_sdk2`):**

```cpp
unitree::robot::ChannelFactory::Instance()->Init(1, "lo");
```

**ROS 2 (`unitree_ros2`):** DDS-Domain und Interface über die CycloneDDS-Config /
`ROS_DOMAIN_ID` bzw. `CYCLONEDDS_URI` auf Domain 1 / `lo` setzen (siehe die
`cyclonedds.xml` in `g1pilot/docker/` als Vorlage: nur `lo`, Multicast aus,
Peer `127.0.0.1`).

Am echten Roboter würdest du stattdessen `ChannelFactoryInitialize(0, "eth0")`
(bzw. dein Roboter-Interface) nehmen. **Nur diese eine Zeile unterscheidet Sim
und Real** — genau darum kannst du gefahrlos zuerst simulieren.

### 5.2 Die Topics & Nachrichten (der „Vertrag")

| Topic | Typ (G1) | Richtung | Inhalt |
|---|---|---|---|
| `rt/lowcmd` | `unitree_hg…LowCmd_` | **dein Code → Sim** | Motor-Sollwerte: `q, dq, tau, kp, kd` pro Gelenk |
| `rt/lowstate` | `unitree_hg…LowState_` | **Sim → dein Code** | Gelenk-`q/dq/tau_est`, IMU (Quaternion, Gyro, Accel) |
| `rt/sportmodestate` | `unitree_go…SportModeState_` | Sim → dein Code | Odometrie (optional) |
| `rt/wirelesscontroller` | `unitree_go…WirelessController_` | Sim → dein Code | Gamepad-State (optional) |

> **Wichtig:** Der G1 nutzt das **`unitree_hg`**-Message-Paket (nicht `unitree_go`
> wie Go2/B2). Die Bridge wählt das automatisch anhand `ROBOT="g1"` (29 Motoren >
> 20 → `hg`). Dein Code muss `LowCmd_`/`LowState_` also aus
> `unitree_sdk2py.idl.unitree_hg.msg.dds_` importieren.

Der Regelkreis, den die Sim je Physikschritt anwendet (identisch zum echten
Low-Level-Controller des G1):

```
ctrl_i  =  tau_i  +  kp_i · (q_i − q_ist_i)  +  kd_i · (dq_i − dq_ist_i)
```

Du schickst also pro Gelenk `q` (Ziel-Winkel), `dq` (Ziel-Geschwindigkeit),
`tau` (Feedforward-Drehmoment) sowie die Gains `kp`/`kd`. Reiner Torque-Betrieb:
`kp=kd=0` und nur `tau` setzen. Reiner Positions-PD: `tau=0`, `kp`/`kd`
sinnvoll wählen.

### 5.3 Joint-Reihenfolge (G1, 29 DoF)

Die Indizes in `motor_cmd[i]` / `motor_state[i]`:

| Index | Gruppe | Gelenke |
|---|---|---|
| 0–5 | Bein **links** | hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll |
| 6–11 | Bein **rechts** | (dito) |
| 12–14 | Taille | waist_yaw, waist_roll, waist_pitch |
| 15–21 | Arm **links** | shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw |
| 22–28 | Arm **rechts** | (dito) |

Vollständige Tabelle (inkl. 23-DoF- und Arm-only-Varianten) in
[`unitree_mujoco/unitree_robots/g1/g1_joint_index_dds.md`](unitree_mujoco/unitree_robots/g1/g1_joint_index_dds.md).

MuJoCo-`qpos` intern (`nq=36`): `[0:3]` Pelvis-Position, `[3:7]`
Pelvis-Quaternion `[w,x,y,z]`, `[7:36]` die 29 Gelenkwinkel. (Das siehst du nur,
wenn du die Sim selbst modifizierst — über DDS bekommst du die 29 Gelenke +
IMU.)

### 5.4 Minimalbeispiel als Vorlage

Im Repo liegt ein lauffähiges Sende-Beispiel:
[`unitree_mujoco/example/python/stand_go2.py`](unitree_mujoco/example/python/stand_go2.py).
Es zeigt das komplette Muster (Publisher `rt/lowcmd`, `motor_cmd[i]` füllen,
CRC setzen, 500-Hz-Loop). Es ist für den Go2 geschrieben (12 Beingelenke,
`unitree_go`) — für den G1 musst du auf `unitree_hg` und 29 Gelenke umstellen,
aber die Struktur ist identisch. Der DDS-Init dort zeigt bereits auf
`ChannelFactoryInitialize(1, "lo")` — also direkt auf die Sim.

---

## 6 · Verbindung testen (Sanity-Check)

**1 · Läuft die Sim?** Viewer-Fenster mit G1 offen, Terminal listet Gelenke.

**2 · Kommt State an?** In einem zweiten Terminal (mit installiertem
`unitree_sdk2_python`) ein kleines Skript, das `rt/lowstate` abonniert und den
ersten Gelenkwinkel druckt:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

ChannelFactoryInitialize(1, "lo")
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(lambda msg: print("q[0] =", msg.motor_state[0].q), 10)
import time; time.sleep(5)
```

Siehst du sich ändernde/plausible Werte → **DDS-Verbindung steht.** Kommt nichts:
Domain/Interface-Mismatch (Abschnitt 7) oder `unitree_hg` vs `unitree_go`
verwechselt.

**3 · Reagiert der Roboter auf Befehle?** Ein einfaches `rt/lowcmd` mit kleinem
`kp` auf ein Armgelenk schicken und im Viewer sehen, ob sich das Gelenk bewegt.
Fang **klein** an (niedrige Gains), damit nichts aufschwingt.

**4 · Für Loco (`HOLD_BASE_MODE=off`):** Ohne aktiven Regler kippt der G1 nach
1–2 s um — das ist erwartet. Erst dein Balance-Regler hält ihn aufrecht.

---

## 7 · Netzwerk-Varianten (gleiche vs. getrennte Maschine)

**Fall A — Sim und dein Projekt auf demselben Rechner (empfohlen):**
Nichts weiter nötig. Beide nutzen Domain 1 / `lo`. (Bei Docker sorgt
`network_mode: host` dafür, dass der Sim-Container dasselbe `lo` wie der Host
sieht.)

**Fall B — auf getrennten Rechnern:**
`lo` funktioniert **nicht** über Maschinengrenzen. Dann:

1. In der Sim-`config.py` `INTERFACE` auf das **echte Netzwerk-Interface**
   setzen (z. B. `"eth0"`), auf dem beide Rechner erreichbar sind.
2. Auf **deiner** Seite dasselbe Interface + **dieselbe Domain (1)** benutzen.
3. CycloneDDS-Discovery muss über dieses Interface laufen (Multicast erlauben
   **oder** den jeweils anderen Host als `<Peer>` eintragen — vgl. die
   `cyclonedds.xml`-Vorlage, die für `lo` das Gegenteil macht).

**Häufigste Fehlerquellen:**
- **Domain-Mismatch** (Sim auf 1, dein Code auf 0) → gar keine Verbindung.
- **`unitree_go` statt `unitree_hg`** beim G1 → Typen passen nicht, keine Daten.
- **Zweites DDS im selben Prozess** (z. B. ROS 2 mit FastDDS **und**
  unitree_sdk2): RMW auf `rmw_cyclonedds_cpp` zwingen, sonst „create domain
  error" / Multicast-Flut auf allen Interfaces. → ROS-Graph-Domain (`ROS_DOMAIN_ID`)
  **verschieden** von der Unitree-DDS-Domain (1) halten.

---

## 8 · Von der Sim auf den echten G1 wechseln

Wenn dein Projekt gegen die Sim sauber läuft, ist der Sprung zur Hardware minimal
— **weil es dieselbe DDS-API ist**:

| Aspekt | Simulation | Echter G1 |
|---|---|---|
| DDS-Init | `(1, "lo")` | `(0, "<eth-zum-roboter>")` |
| Low-Level (`rt/lowcmd`/`rt/lowstate`) | von der Sim bedient | vom Roboter bedient — **identisch** |
| Laufen/Balance | **dein** Regler (oder mein `loco_sim`) | Unitree-Onboard-High-Level (`loco_client`) **oder** dein Regler im Low-Level |
| Basis | frei/geschweißt (config) | real, frei |

> ⚠️ **Vor dem ersten Real-Start:** E-Stop erreichbar, ≥ 2 m Freifläche, erst in
> Sim validieren, Befehle klein anfangen, Drehmoment-Limits beachten. Der G1 ist
> ein 2-beiniger Roboter — ein Fehler im Regler heißt am echten Gerät „umfallen".
> Meine Checklisten dazu: [`g1pilot/PREFLIGHT.md`](g1pilot/PREFLIGHT.md),
> [`g1pilot/REAL_TESTING.md`](g1pilot/REAL_TESTING.md).

---

## 9 · Kurz-Checkliste

- [ ] Repo geklont, `unitree_sdk2_python` per `pip install -e .` installiert
- [ ] `mujoco` installiert (oder Docker-Weg gewählt)
- [ ] `config.py`: `ROBOT="g1"`, `DOMAIN_ID=1`, `INTERFACE="lo"`, `USE_JOYSTICK=0`
- [ ] `HOLD_BASE_MODE` gewählt: `weld` (Arme) vs. `off` (Loco/Ganzkörper)
- [ ] Sim gestartet, Viewer-Fenster + Gelenkliste sichtbar
- [ ] Dein Code: `ChannelFactoryInitialize(1, "lo")`, Import aus `unitree_hg`
- [ ] `rt/lowstate` empfangen getestet (Sanity-Check Abschnitt 6)
- [ ] `rt/lowcmd` mit kleinem `kp` getestet → Gelenk bewegt sich
- [ ] (Loco) eigener Balance-Regler hält den frei stehenden G1 aufrecht

---

### Weiterführend

- [`README.md`](README.md) — Gesamtüberblick des Repos (mein g1pilot-Stack)
- [`unitree_mujoco/unitree_robots/g1/g1_joint_index_dds.md`](unitree_mujoco/unitree_robots/g1/g1_joint_index_dds.md) — vollständige Joint-Tabellen
- [`unitree_mujoco/example/`](unitree_mujoco/example/) — Sende-Beispiele (Python / C++ / ROS 2)
- [`unitree_sdk2_python/`](unitree_sdk2_python/) — das Python-SDK inkl. IDL-Definitionen
