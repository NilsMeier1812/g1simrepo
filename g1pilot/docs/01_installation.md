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
  halbwegs aktuellen Rechner. Eine GPU ist nicht nötig, macht die Fenster
  (MuJoCo, RViz) aber um ein Vielfaches flüssiger — siehe
  [Grafik-Performance](#grafik-performance-gpu--sparsame-grafik).
- **Echter Roboter** — siehe zusätzlich
  [70_echtroboter_anleitung.md](70_echtroboter_anleitung.md), bevor der Stack
  gegen Hardware gestartet wird.

## Voraussetzungen

- Linux (getestet auf Ubuntu 22.04 / 24.04) oder Windows 10/11 mit WSL2.
- ≥ 8 GB RAM, ~10 GB freier Plattenplatz für die Docker-Images.
- Ein laufendes X11-Display (für RViz und die MuJoCo-/Teleop-Fenster).
  Unter Wayland hilft in der Regel `xhost` über den XWayland-Layer; über SSH
  mit `ssh -X` verbinden.
- Keine GPU/CUDA nötig — die Physik ist rein CPU-basiert. Für flüssige
  Fenster wird eine vorhandene GPU automatisch genutzt (siehe
  [Grafik-Performance](#grafik-performance-gpu--sparsame-grafik)).

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

## Grafik-Performance (GPU + Sparsame Grafik)

MuJoCo-Viewer und RViz laufen in Docker-Containern. Ohne durchgereichte GPU
zeichnen beide **per Software auf der CPU** (llvmpipe) — das ist auf
schwächeren Rechnern der Hauptgrund für ruckelnde Fenster (1–2 fps) und nimmt
der Physik zusätzlich CPU-Kerne weg (die Sim läuft dann langsamer als
Echtzeit; dank Lockstep bleibt die Regelung trotzdem korrekt).

**GPU (automatisch).** `start.sh`, `run_sim*.sh` und `make sim` binden über
`docker/compose_gpu.sh` automatisch einen GPU-Zusatz ein:

| Host | Was passiert | Einmalig nötig |
|---|---|---|
| NVIDIA mit proprietärem Treiber | `docker/compose.gpu-nvidia.yml` | NVIDIA Container Toolkit (s.u.) |
| Intel/AMD (Mesa) oder NVIDIA mit `nouveau` | `docker/compose.gpu-dri.yml` (`/dev/dri`) | nichts |
| WSL2 / keine GPU | Software-Rendering wie bisher | — |

Welcher Modus aktiv ist, steht beim Start als `[gpu] ...`-Zeile im Terminal.
Erzwingen lässt er sich mit `G1_GPU=nvidia|dri|off`.

NVIDIA Container Toolkit (nur bei NVIDIA mit proprietärem Treiber, braucht
`sudo`; Paketquelle laut
[NVIDIA-Anleitung](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
einrichten):

```bash
nvidia-smi                       # muss die Karte zeigen (proprietärer Treiber aktiv)
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker info | grep -i runtimes   # muss "nvidia" enthalten
```

**Sparsame Grafik (Schalter).** Im Startmenü „Sparsame Grafik" bzw.
`G1_LOW_GFX=1`. Reine Anzeige-Optimierung, Physik und Kollision bleiben
bitgenau gleich:

- MuJoCo und RViz zeigen vereinfachte Meshes (Roboter ca. 1/5, Umgebungen
  ca. 1/5 der Dreiecke); Kollisionen rechnen weiter mit den Originalen.
- Kollisions-Meshes werden nicht mehr zusätzlich gezeichnet (am G1 lagen sie
  deckungsgleich unter dem sichtbaren Modell).
- Keine Schatten und keine Bodenspiegelung im MuJoCo-Fenster.
- RViz öffnet mit normaler Fenstergröße (1600×900).

Gemessen mit reinem Software-Rendering: Standardszene ca. 6× und
DC-Demonstrator-Zelle ca. 12× mehr Bilder pro Sekunde.

**Weitere Handgriffe ohne Code:**

- MuJoCo-Fenster: Taste `0` blendet Umgebung und Boden aus (Physik läuft
  weiter), `Tab`/`Shift+Tab` blendet die Seitenleisten aus, kleineres Fenster
  = weniger Pixel.
- RViz: Display „SceneMarkers" abhaken (Nav-Ansicht), wenn die Umgebung dort
  nicht gebraucht wird.
- Inspire-Hände, Navigation und RViz nur einschalten, wenn sie gebraucht
  werden.

## Windows (WSL2)

Läuft auf Windows, aber **innerhalb von WSL2**, nicht über „Docker Desktop für
Windows" pur. Zwei Dinge im Setup sind Linux-spezifisch: `network_mode: host`
(trägt die DDS-Kommunikation der Container über `lo`) und die X11-GUIs (RViz
+ MuJoCo-Viewer). Beides funktioniert in WSL2 sauber, in Docker Desktop pur
dagegen nicht zuverlässig. GPU/CUDA wird nicht gebraucht.

Empfohlen: Windows 11 (WSLg für die GUIs ist eingebaut).

```powershell
# In PowerShell: WSL2 + Ubuntu 24.04 installieren (ggf. danach neu starten)
wsl --install -d Ubuntu-24.04
```

Bewusst **Ubuntu-24.04** und nicht `Ubuntu`: `Ubuntu` ist immer die neueste
Version (26.04, Python 3.14) – damit laesst sich der Scene Editor nicht
einrichten (braucht Python < 3.13). Gestartet wird sie danach mit
`wsl -d Ubuntu-24.04` bzw. ueber „Ubuntu 24.04" im Startmenue.

Ist **Docker Desktop** installiert: es haengt sich standardmaessig in die
*Standard*-WSL-Distribution ein und kollidiert dort mit der nativen
Docker-Engine. Entweder Ubuntu nicht zur Standard-Distribution machen oder in
Docker Desktop unter *Settings → Resources → WSL integration* die Integration
fuer Ubuntu-24.04 ausschalten.

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

### Nur Umgebungen bauen: Scene Editor mit Docker Desktop

Für den Scene Editor (Umgebungen bauen, CAD/STEP importieren) reicht
**Docker Desktop** – er braucht weder Host-Networking noch X11, die
Oberfläche läuft im Browser. In PowerShell im Ordner `g1pilot/`:

```powershell
docker compose --profile editor build scene-editor         # einmalig, ~5 min
docker compose --profile editor run --rm --service-ports scene-editor
# -> Menü von launch.sh; Editor im Browser: http://127.0.0.1:8080
```

Details (CAD-Dateien importieren, Argumente für `launch.sh`) in
`unitree_mujoco/scene_editor/README.md`, Abschnitt „Setup unter Windows".

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
