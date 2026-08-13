# Installation & Ersteinrichtung

Richtet sich an: alle Anwender. Ziel: von einem frischen Rechner zu einer
laufenden Simulation.

## Überblick

G1Pilot läuft komplett in Docker. Auf dem Host wird nur Docker, Git und ein
X11-Display gebraucht — der gesamte ROS-2-/MuJoCo-Stack steckt in den
Container-Images. Das Repository ist eigenständig (keine Git-Submodule): die
Unitree-Abhängigkeiten `unitree_mujoco`, `unitree_ros2`, `unitree_sdk2_python`
liegen mit im Baum.

Zwei Betriebsarten:

- **Simulation** — MuJoCo-Physik statt echtem Roboter, läuft auf jedem
  halbwegs aktuellen Rechner, keine GPU nötig.
- **Echter Roboter** — siehe zusätzlich
  [70_echtroboter_anleitung.md](70_echtroboter_anleitung.md), bevor der Stack
  gegen Hardware gestartet wird.

## Voraussetzungen

- Linux (getestet auf Ubuntu 22.04 / 24.04) oder Windows 10/11 mit WSL2.
- ≥ 8 GB RAM, ~10 GB freier Plattenplatz für die Docker-Images.
- Ein laufendes X11-Display (für RViz und die MuJoCo-/Teleop-Fenster).
  Unter Wayland hilft in der Regel `xhost` über den XWayland-Layer; über SSH
  mit `ssh -X` verbinden.
- Keine GPU/CUDA nötig — die Simulation ist rein CPU-basiert.

## Schritt für Schritt (Linux)

**1. System-Pakete**

```bash
sudo apt update
sudo apt install -y git x11-xserver-utils ca-certificates curl
```

**2. Docker Engine + Compose-Plugin** (offizielles Docker-Repository)

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
  sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
```

**3. Docker ohne `sudo` nutzbar machen**

```bash
sudo usermod -aG docker $USER
newgrp docker          # oder: ab-/wieder anmelden
docker run --rm hello-world   # muss ohne sudo durchlaufen
```

**4. Repository klonen**

```bash
git clone https://github.com/nilsmeier1812/g1simrepo.git
cd g1simrepo/g1pilot
```

**5. Images bauen** (erstmalig, ca. 8 Minuten; lädt ROS 2 Humble, MuJoCo,
Pinocchio usw.)

```bash
make build-sim         # baut g1pilot-sim:v1.1.0 + g1pilot-mujoco:v1.0
```

**6. Starten**

```bash
make sim               # baut bei Bedarf nach und startet den Stack
```

Beim ersten `make sim` wird zusätzlich `xhost +local:docker` gesetzt, damit
die Container auf das Display zugreifen dürfen. Erscheinen das MuJoCo-Fenster
und der Streamdeck, steht die Umgebung. Der Roboter steht dabei zunächst nur
— siehe [30_loco_anleitung.md](30_loco_anleitung.md) für die Bedienung.

## Windows (WSL2)

Läuft auf Windows, aber **innerhalb von WSL2**, nicht über „Docker Desktop für
Windows" pur. Zwei Dinge im Setup sind Linux-spezifisch: `network_mode: host`
(trägt die DDS-Kommunikation der Container über `lo`) und die X11-GUIs (RViz
+ MuJoCo-Viewer). Beides funktioniert in WSL2 sauber, in Docker Desktop pur
dagegen nicht zuverlässig. GPU/CUDA wird nicht gebraucht.

Empfohlen: Windows 11 (WSLg für die GUIs ist eingebaut).

```powershell
# In PowerShell (als Administrator): WSL2 + Ubuntu installieren, dann neu starten
wsl --install -d Ubuntu
```

Danach im Ubuntu-Terminal (WSL) weiter — ab hier identisch zur
Linux-Anleitung oben:

```bash
# Docker-Engine NATIV in der WSL-Distro installieren (Schritte 1-3 oben).
# Native Engine statt Docker-Desktop-Integration, damit Host-Networking
# ohne Tricks funktioniert.
sudo service docker start

# Repo INS WSL-Dateisystem klonen (NICHT nach /mnt/c/... — das ist langsam)
cd ~
git clone https://github.com/nilsmeier1812/g1simrepo.git
cd g1simrepo/g1pilot
make build-sim && make sim
```

WSLg setzt `DISPLAY` automatisch und stellt den X11-Socket bereit — das
MuJoCo-Fenster und RViz öffnen sich direkt auf dem Windows-Desktop.

Windows 10: geht ebenfalls über WSL2, aber WSLg ist nicht in jeder Version
dabei — dann einen X-Server (VcXsrv/X410) starten und `DISPLAY` von Hand
setzen. Docker Desktop statt nativer Engine ist möglich, `network_mode: host`
ist dort aber nur als (zu aktivierendes) Beta-Feature neuerer Versionen
verfügbar.

## Starten im Alltag

Der einfachste Einstieg ist `./start.sh`. Ohne Argumente öffnet sich ein
grafisches Startmenü (`g1_gui.py`, Tkinter): drei Karten — *Simulation
starten*, *Echten Roboter starten*, *Umgebungen bearbeiten*. Alles läuft in
einem Fenster; Menü, Optionsseiten und Log-Ansicht werden ausgetauscht.
Startet man einen Stack, erscheint dessen Docker-Ausgabe live im Fenster mit
einem Stop-Button. Über *‹ Menü* geht man zurück, ohne den Stack zu beenden —
er taucht unter *Laufende Prozesse* wieder auf. Fehlt Tkinter oder ein
Display, fällt `start.sh` automatisch auf ein klassisches Text-Menü zurück
(erzwingbar mit `--menu` oder `G1_NO_GUI=1`).

```bash
./start.sh            # grafisches Startmenü (Standard)
./start.sh --menu     # klassisches Text-Menü
# Nicht-interaktiv (Sim, Defaults/Env-Overrides):
USE_RVIZ=true ./start.sh --yes
# Nicht-interaktiv (Real, erfordert explizite Bestätigung):
G1_MODE=real ROBOT_INTERFACE=enp3s0 G1_REAL_CONFIRM=1 ./start.sh --yes
```

Alternativ direkt über `make` bzw. `docker compose`:

| Befehl | Wirkung |
|---|---|
| `make sim` | Stack im Vordergrund starten (Ctrl-C stoppt) |
| `make sim-bg` | Stack im Hintergrund |
| `make real ROBOT_INTERFACE=<nic>` | Echten Roboter starten (schlank: Arme + Hände + Loco) |
| `make real-full ROBOT_INTERFACE=<nic>` | Echten Roboter mit Livox/MOLA/Navigation starten |
| `make stop` | Stack stoppen |
| `make logs` / `make status` | Logs folgen / Container-Status |
| `make shell-sim` / `make shell-mujoco` / `make shell-real` | Shell im jeweiligen Container |
| `make clean` | Container + Images entfernen |

Jedes `make sim`/`make real`/`make real-full` baut das benötigte Image bei
Bedarf automatisch nach — ein separater `make build-*`-Aufruf ist nur nötig,
um das Bauen vom Starten zu trennen (z. B. um vorab zu bauen, ohne den Stack
gleich hochzufahren). Intern ist `make` nur ein dünner Wrapper um
`docker compose --profile <sim|real|real-full>` auf der einen
`docker-compose.yml` — es gibt bewusst keine separaten
`docker-compose.*.yaml`-Dateien mehr.

```bash
# Simulation: MuJoCo-G1 + g1pilot (robot_state, Arme, RViz, Teleop)
G1_SIM_MODE=true docker compose --profile sim up

# Echter Roboter (schlank: Arme + Hände + Unitree-Loco-Controller)
ROBOT_INTERFACE=<iface> docker compose --profile real up

# Echter Roboter mit Livox/MOLA/Navigation (großes Image, MID360 nötig)
ROBOT_INTERFACE=<iface> docker compose --profile real-full up
```

## Nach dem Start: Dokumentation

Alle weiteren Themen-Dokumente sind über [docs/README.md](README.md)
erreichbar, sowie aus dem grafischen Startmenü über den Menüpunkt
„Dokumentation".

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| `docker run --rm hello-world` verlangt `sudo` | Neu einloggen bzw. `newgrp docker` nach Schritt 3. |
| Kein MuJoCo-/RViz-Fenster | `DISPLAY` gesetzt? `xhost +local:docker` gelaufen (macht `make`/`start.sh` automatisch)? |
| Build bricht mit Netzwerkfehlern ab | Docker-Build lädt ROS-2-Pakete aus dem Internet — Firewall/Proxy prüfen. |
| `make sim` baut jedes Mal neu | Normal, sofern sich Quellcode/Dockerfile geändert haben; Docker cached unveränderte Layer. |
| Fenster öffnen sich, aber der Roboter reagiert auf nichts | Zunächst normal — siehe [30_loco_anleitung.md](30_loco_anleitung.md), der Roboter startet bewusst nicht automatisch. |
