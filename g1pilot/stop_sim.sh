#!/usr/bin/env bash
# Stoppt den Sim-Stack sauber und entfernt verwaiste/hintergruendige Container.
cd "$(dirname "$0")"
docker compose --profile sim down --remove-orphans
echo "[stop_sim] Sim-Stack gestoppt. Laufende Container:"
docker ps --format '  {{.Names}}\t{{.Status}}'
