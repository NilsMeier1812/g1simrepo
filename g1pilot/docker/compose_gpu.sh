#!/usr/bin/env bash
# Gibt die docker-compose "-f"-Argumente fuer die Sim aus: Basisdatei plus,
# falls moeglich, einen GPU-Zusatz (sonst rendern MuJoCo/RViz auf der CPU).
#
#   G1_GPU=auto   (Default) NVIDIA-Runtime -> nvidia, sonst /dev/dri -> dri
#   G1_GPU=nvidia | dri | off   erzwingen
#
# Nutzung:  docker compose $(docker/compose_gpu.sh) --profile sim up
# Hinweise (welcher Modus, was fehlt) gehen nach stderr.
cd "$(dirname "$0")/.." || exit 1

mode="${G1_GPU:-auto}"
base="-f docker-compose.yml"

is_wsl=0
grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null && is_wsl=1

has_nvidia_runtime() {
  docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'
}

if [ "$mode" = "auto" ]; then
  if has_nvidia_runtime; then
    mode=nvidia
  elif [ -d /dev/dri ] && [ "$is_wsl" = "0" ]; then
    mode=dri
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
      echo "[gpu] NVIDIA-Treiber gefunden, aber Docker kennt die NVIDIA-Runtime nicht ->" \
           "Container rendern per Software. Einmalig: sudo apt install nvidia-container-toolkit &&" \
           "sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker" >&2
    fi
  else
    mode=off
  fi
fi

case "$mode" in
  nvidia) echo "[gpu] Grafik im Container: NVIDIA-GPU (docker/compose.gpu-nvidia.yml)" >&2
          echo "$base -f docker/compose.gpu-nvidia.yml" ;;
  dri)    echo "[gpu] Grafik im Container: /dev/dri (Mesa; docker/compose.gpu-dri.yml)" >&2
          echo "$base -f docker/compose.gpu-dri.yml" ;;
  *)      echo "[gpu] Grafik im Container: Software-Rendering (CPU, keine GPU durchgereicht)" >&2
          echo "$base" ;;
esac
