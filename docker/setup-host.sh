#!/usr/bin/env bash
# Host prereqs for running Isaac Sim via docker compose.
# Run this yourself in a real terminal: bash setup-host.sh
# It will prompt for your sudo password once (cached for the rest of the run).

set -euo pipefail

echo "==> Installing docker compose v2 plugin"
sudo apt-get update
sudo apt-get install -y docker-compose-v2

echo "==> Installing NVIDIA Container Toolkit"
sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg2

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

echo "==> Verifying GPU passthrough"
docker run --rm --runtime=nvidia --gpus all nvcr.io/nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi

echo "==> Creating Isaac Sim cache/data directories"
mkdir -p ~/docker/isaac-sim/{cache/main,cache/computecache,config,data,logs,pkg}
mkdir -p ~/.cache/ov/hub
sudo chown -R 1234:1234 ~/docker/isaac-sim ~/.cache/ov/hub

echo "==> Done. docker compose version:"
docker compose version
