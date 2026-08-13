# Dokumentation

Dies ist der zentrale Einstiegspunkt in die G1Pilot-Dokumentation. Jedes
Thema hat zwei Dokumente:

- **Anleitung** — richtet sich an Anwender. Wie startet man die Funktion,
  wie bedient man sie, welche Einstellungen gibt es. Kein Code-Wissen
  vorausgesetzt.
- **Technik** — richtet sich an Entwickler, die den Code ändern wollen.
  Beschreibt Aufbau, Datenfluss, Topics/Parameter und die Stellen im
  Quellcode, an denen etwas passiert.

Alle Dokumente sind auch über das grafische Startmenü erreichbar
(`./start.sh` → Menüpunkt „Dokumentation").

## Einstieg

| Dokument | Für wen |
|---|---|
| [Installation & Ersteinrichtung](01_installation.md) | Alle — erster Start |
| [Architektur](02_architektur.md) | Entwickler — Gesamtüberblick über Container, DDS, Datenflüsse |

## Themen

| Thema | Anleitung (Anwender) | Technik (Entwickler) |
|---|---|---|
| Arm-Manipulation | [10_arm_manipulation_anleitung.md](10_arm_manipulation_anleitung.md) | [11_arm_manipulation_technik.md](11_arm_manipulation_technik.md) |
| Arm-API (externe Projekte) | [20_arm_api_anleitung.md](20_arm_api_anleitung.md) | [21_arm_api_technik.md](21_arm_api_technik.md) |
| Locomotion (Stehen/Laufen) | [30_loco_anleitung.md](30_loco_anleitung.md) | [31_loco_technik.md](31_loco_technik.md) |
| Teleoperation (Streamdeck/Joystick) | [40_teleoperation_anleitung.md](40_teleoperation_anleitung.md) | [41_teleoperation_technik.md](41_teleoperation_technik.md) |
| Navigation (autonomes Fahren) | [50_navigation_anleitung.md](50_navigation_anleitung.md) | [51_navigation_technik.md](51_navigation_technik.md) |
| Inspire-FTP-Hände | [60_inspire_haende_anleitung.md](60_inspire_haende_anleitung.md) | [61_inspire_haende_technik.md](61_inspire_haende_technik.md) |
| Echter Roboter (Sicherheit & Ablauf) | [70_echtroboter_anleitung.md](70_echtroboter_anleitung.md) | — (siehe die jeweiligen Technik-Dokumente oben) |

## Aufbau der Dokumente

Anleitungen folgen im Wesentlichen dieser Gliederung: Überblick, Voraussetzungen,
Bedienung Schritt für Schritt, Konfiguration, Fehlerbehebung.

Technik-Dokumente folgen: Überblick, beteiligte Dateien, Datenfluss/Architektur,
wichtige Funktionen im Detail, Konfiguration/Parameter, bekannte Einschränkungen.

Der Aufbau ist bewusst ähnlich, aber nicht starr — jedes Thema hat andere
Schwerpunkte.
