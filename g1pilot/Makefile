# ═══════════════════════════════════════════════════════
# G1Pilot Makefile
#   make build-sim       → Sim-Images bauen (~8 min)
#   make build-real      → Real-Image bauen (schlank, ~10 min, kein Livox/MOLA)
#   make build-real-full → Real-Image MIT Livox/MOLA bauen (~45 min)
#   make sim             → Simulation starten
#   make real            → Echter Roboter starten (schlank)
#   make real-full       → Echter Roboter mit Livox/MOLA/Navigation starten
#   make stop            → Alles stoppen
#
#   Duenner Wrapper um `docker compose --profile <sim|real|real-full>` auf
#   der EINEN docker-compose.yml (siehe deren Kopfkommentar) -- es gibt
#   bewusst keine separaten docker-compose.*.yaml-Dateien mehr.
# ═══════════════════════════════════════════════════════

ROBOT_INTERFACE ?= eth0
DISPLAY         ?= :0

.PHONY: sim sim-bg real real-full stop \
        build-sim build-real build-real-full \
        build-mujoco build-g1pilot-sim build-g1pilot-real build-g1pilot-real-full \
        logs status clean shell-sim shell-mujoco shell-real

# ─── SIMULATION ─────────────────────────────────────────────
sim: build-g1pilot-sim build-mujoco
	@echo "🔵 Starte G1Pilot SIMULATION..."
	@xhost +local:docker 2>/dev/null || true
	DISPLAY=$(DISPLAY) docker compose --profile sim up

sim-bg: build-g1pilot-sim build-mujoco
	@echo "🔵 Starte SIMULATION (Hintergrund)..."
	@xhost +local:docker 2>/dev/null || true
	DISPLAY=$(DISPLAY) docker compose --profile sim up -d

# ─── ECHTER ROBOTER ─────────────────────────────────────────
real: build-g1pilot-real
	@echo "🔴 Starte G1Pilot ECHTER ROBOTER (schlank: Arme + Haende + Loco)..."
	@xhost +local:root 2>/dev/null || true
	ROBOT_INTERFACE=$(ROBOT_INTERFACE) DISPLAY=$(DISPLAY) \
	  docker compose --profile real up

real-full: build-g1pilot-real-full
	@echo "🔴 Starte G1Pilot ECHTER ROBOTER (voll: + Livox/MOLA/Navigation)..."
	@xhost +local:root 2>/dev/null || true
	ROBOT_INTERFACE=$(ROBOT_INTERFACE) DISPLAY=$(DISPLAY) \
	  docker compose --profile real-full up

# ─── BAUEN ──────────────────────────────────────────────────
build-sim: build-g1pilot-sim build-mujoco

build-real: build-g1pilot-real

build-real-full: build-g1pilot-real-full

build-g1pilot-sim:
	@echo "📦 Baue G1Pilot SIM Image (kein Livox/MOLA, ~8 min)..."
	docker build -f docker/Dockerfile.sim -t g1pilot-sim:v1.1.0 .

build-g1pilot-real:
	@echo "📦 Baue G1Pilot REAL Image (schlank, ~10 min)..."
	docker build -f docker/Dockerfile.real -t g1pilot-real:v1.2.0 .

build-g1pilot-real-full:
	@echo "📦 Baue G1Pilot REAL-FULL Image (+ Livox/MOLA, ~45 min)..."
	docker build -f docker/Dockerfile -t g1pilot:v1.1.0 .

build-mujoco:
	@echo "📦 Baue MuJoCo Simulator Image..."
	docker build -f docker/Dockerfile.mujoco -t g1pilot-mujoco:v1.0 ..

# ─── STOPPEN ────────────────────────────────────────────────
stop:
	docker compose --profile sim down --remove-orphans 2>/dev/null || true
	docker compose --profile real down --remove-orphans 2>/dev/null || true
	docker compose --profile real-full down --remove-orphans 2>/dev/null || true

# ─── HILFS-COMMANDS ─────────────────────────────────────────
logs:
	docker compose --profile sim logs -f

status:
	docker compose --profile sim --profile real --profile real-full ps

clean:
	docker compose --profile sim down --volumes --remove-orphans 2>/dev/null || true
	docker compose --profile real down --volumes --remove-orphans 2>/dev/null || true
	docker compose --profile real-full down --volumes --remove-orphans 2>/dev/null || true
	docker rmi g1pilot-sim:v1.1.0 g1pilot-mujoco:v1.0 g1pilot-real:v1.2.0 g1pilot:v1.1.0 2>/dev/null || true

# Shells ueber `docker compose exec` statt Name-Filter -- Compose ohne
# `container_name:` benennt Container nach <projekt>-<service>-<n>, das
# haengt vom Verzeichnisnamen/-COMPOSE_PROJECT_NAME ab.
shell-sim:
	docker compose --profile sim exec g1pilot-sim bash

shell-mujoco:
	docker compose --profile sim exec mujoco-sim bash

shell-real:
	docker compose --profile real exec g1pilot-real bash
