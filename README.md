# G1Pilot — MuJoCo-Simulation & Steuerungsstack für den Unitree G1

G1Pilot ist ein ROS-2-Steuerungsstack für den humanoiden Roboter Unitree G1
(29 Freiheitsgrade, optional Inspire-FTP-Hände). Er läuft **unverändert**
sowohl gegen eine MuJoCo-Simulation als auch gegen den echten Roboter — beide
sprechen dieselbe Unitree-DDS-Schnittstelle, sodass derselbe Code in Sim und
Real läuft.

| | |
|---|---|
| Roboter | Unitree G1, 29 DOF, optional Inspire-FTP-Hände |
| Simulator | MuJoCo (Python-Bindings), 1 kHz Physik |
| Middleware | Unitree SDK2 (CycloneDDS) + ROS 2 Humble |
| Laufzeit | Docker-Container, `network_mode: host` |

## Funktionsumfang

- **Arm-Manipulation** — kartesische Zielposen per interaktivem RViz-Marker,
  gelöst über eine Pinocchio-basierte inverse Kinematik, mit
  Kollisionsprüfung und Positionsspeicher.
- **Arm-API** — eine HTTP/JSON-Schnittstelle, über die auch Projekte ohne
  ROS Zielposen einspielen können.
- **Locomotion** — Stehen und Laufen; in der Simulation über einen
  RL-basierten Lauf-Regler plus modellbasierten Stand-Regler, auf echter
  Hardware über Unitrees Onboard-Controller.
- **Teleoperation** — grafische Bedienoberfläche (Streamdeck) und
  Unterstützung für einen physischen Gamepad.
- **Autonome Navigation** — Punkt-zu-Punkt-Fahrt mit Pfadplanung und
  Hindernisvermeidung.
- **Inspire-FTP-Hände** — Fingersteuerung mit Kraft- und Tastsinn-Rückmeldung,
  in Simulation und auf echter Hardware.
- **Sim ↔ Real ohne Code-Änderung** — Umschalten allein über
  Umgebungsvariablen.

## Schnellstart

```bash
git clone https://github.com/nilsmeier1812/g1simrepo.git
cd g1simrepo/g1pilot
make build-sim
make sim
```

Ausführliche Installationsanleitung (Linux, Windows/WSL2, Voraussetzungen):
[g1pilot/docs/01_installation.md](g1pilot/docs/01_installation.md).

Nach dem Start öffnet sich ein grafisches Startmenü (`./start.sh`), über das
sich sowohl die Simulation als auch der echte Roboter starten lassen, und
über das die gesamte Dokumentation erreichbar ist (Menüpunkt
„Dokumentation").

## Dokumentation

Die vollständige Dokumentation liegt in [g1pilot/docs/](g1pilot/docs/README.md)
und ist nach Themen gegliedert. Jedes Thema hat zwei Dokumente: eine
**Anleitung** für Anwender und ein **Technik**-Dokument für Entwickler.

| Thema | Anleitung | Technik |
|---|---|---|
| Installation | [g1pilot/docs/01_installation.md](g1pilot/docs/01_installation.md) | — |
| Architektur | — | [g1pilot/docs/02_architektur.md](g1pilot/docs/02_architektur.md) |
| Arm-Manipulation | [10_arm_manipulation_anleitung.md](g1pilot/docs/10_arm_manipulation_anleitung.md) | [11_arm_manipulation_technik.md](g1pilot/docs/11_arm_manipulation_technik.md) |
| Arm-API (externe Projekte) | [20_arm_api_anleitung.md](g1pilot/docs/20_arm_api_anleitung.md) | [21_arm_api_technik.md](g1pilot/docs/21_arm_api_technik.md) |
| Locomotion | [30_loco_anleitung.md](g1pilot/docs/30_loco_anleitung.md) | [31_loco_technik.md](g1pilot/docs/31_loco_technik.md) |
| Teleoperation | [40_teleoperation_anleitung.md](g1pilot/docs/40_teleoperation_anleitung.md) | [41_teleoperation_technik.md](g1pilot/docs/41_teleoperation_technik.md) |
| Navigation | [50_navigation_anleitung.md](g1pilot/docs/50_navigation_anleitung.md) | [51_navigation_technik.md](g1pilot/docs/51_navigation_technik.md) |
| Inspire-FTP-Hände | [60_inspire_haende_anleitung.md](g1pilot/docs/60_inspire_haende_anleitung.md) | [61_inspire_haende_technik.md](g1pilot/docs/61_inspire_haende_technik.md) |
| Echter Roboter (Sicherheit) | [70_echtroboter_anleitung.md](g1pilot/docs/70_echtroboter_anleitung.md) | — |

## Projektstruktur

```
.
├── g1pilot/                     ROS-2-Package + Docker/Compose/Makefile (Hauptprojekt)
│   ├── docs/                    vollständige Dokumentation
│   ├── g1pilot/                 Node-Quellcode (state, manipulation, navigation, teleoperation, utils)
│   ├── launch/                  ROS-2-Launchdateien
│   ├── policies/g1_wholebody/   RL-Lauf-Policy (ONNX)
│   └── docker/, docker-compose.yml
├── unitree_mujoco/               MuJoCo-Simulation (Unitree, angepasst)
├── unitree_ros2/                 Unitree-ROS-2-Abhängigkeiten
└── unitree_sdk2_python/          Unitree-SDK (Python-Bindings)
```

Das Repository ist eigenständig (keine Git-Submodule) — alle Abhängigkeiten
liegen mit im Baum.

## Bekannte Einschränkungen

- Der Simulations-Lauf-Regler (`loco_sim`) ist ein reiner Sim-Stellvertreter
  für den Onboard-Regler des echten G1 und nicht für den Realeinsatz gedacht
  — Details in [g1pilot/docs/31_loco_technik.md](g1pilot/docs/31_loco_technik.md).
- Autonome Navigation hat keine Live-Perzeption; Hindernisvermeidung gilt
  nur für die beim Start geladene Umgebung.

## Lizenz

Siehe [g1pilot/LICENSE](g1pilot/LICENSE).
