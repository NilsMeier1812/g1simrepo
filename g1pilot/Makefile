# ═══════════════════════════════════════════════════════
# G1Pilot Makefile
#   make build-sim   → Sim-Images bauen (~8 min)
#   make build-real  → Real-Image bauen (~45 min, Livox!)
#   make sim         → Simulation starten
#   make real        → Echter Roboter starten
#   make stop        → Alles stoppen
# ═══════════════════════════════════════════════════════

ROBOT_INTERFACE ?= eth0
DISPLAY         ?= :0

.PHONY: sim real stop build-sim build-real build-mujoco build-g1pilot-sim build-g1pilot-real logs status clean

# ─── SIMULATION ─────────────────────────────────────────────
sim: build-g1pilot-sim build-mujoco
	@echo "🔵 Starte G1Pilot SIMULATION..."
	@xhost +local:docker 2>/dev/null || true
	DISPLAY=$(DISPLAY) docker compose -f docker-compose.sim.yaml up

sim-bg: build-g1pilot-sim build-mujoco
	@echo "🔵 Starte SIMULATION (Hintergrund)..."
	@xhost +local:docker 2>/dev/null || true
	DISPLAY=$(DISPLAY) docker compose -f docker-compose.sim.yaml up -d

# ─── ECHTER ROBOTER ─────────────────────────────────────────
real: build-g1pilot-real
	@echo "🔴 Starte G1Pilot ECHTER ROBOTER..."
	ROBOT_INTERFACE=$(ROBOT_INTERFACE) DISPLAY=$(DISPLAY) \
	  docker compose -f docker-compose.real.yaml up

# ─── BAUEN ──────────────────────────────────────────────────
build-sim: build-g1pilot-sim build-mujoco

build-real: build-g1pilot-real

build-g1pilot-sim:
	@echo "📦 Baue G1Pilot SIM Image (kein Livox/MOLA, ~8 min)..."
	docker build -f docker/Dockerfile.sim -t g1pilot-sim:v1.1.0 .

build-g1pilot-real:
	@echo "📦 Baue G1Pilot REAL Image (~45 min)..."
	docker build -f docker/Dockerfile -t g1pilot-real:v1.1.0 .

build-mujoco:
	@echo "📦 Baue MuJoCo Simulator Image..."
	docker build -f docker/Dockerfile.mujoco -t g1pilot-mujoco:v1.0 ..

# ─── STOPPEN ────────────────────────────────────────────────
stop:
	docker compose -f docker-compose.sim.yaml down 2>/dev/null || true
	docker compose -f docker-compose.real.yaml down 2>/dev/null || true

# ─── HILFS-COMMANDS ─────────────────────────────────────────
logs:
	docker compose -f docker-compose.sim.yaml logs -f

status:
	docker compose -f docker-compose.sim.yaml ps

clean:
	docker compose -f docker-compose.sim.yaml down --volumes --remove-orphans 2>/dev/null || true
	docker compose -f docker-compose.real.yaml down --volumes --remove-orphans 2>/dev/null || true
	docker rmi g1pilot-sim:v1.1.0 g1pilot-real:v1.1.0 g1pilot-mujoco:v1.0 2>/dev/null || true

shell-sim:
	docker exec -it $$(docker ps -qf "name=g1pilot_sim") bash

shell-mujoco:
	docker exec -it $$(docker ps -qf "name=g1_mujoco_sim") bash
