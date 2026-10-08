#!/bin/bash
# Optional app system dependencies. Docker runs this as root in build and runtime images.
# Keep this script self-contained; only dependency declaration files are mounted here.
# Do not install NVIDIA drivers: the host provides them.
set -euo pipefail

# Example system library (uncomment only when your app needs it):
# apt-get install -y --no-install-recommends libglib2.0-0

# Optional CUDA Toolkit example: Ubuntu 22.04, amd64, CUDA 12.4.
# Choose a version matching your app; PyTorch wheels can supply their runtime libraries.
# Installation reference: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/
# [ "$(dpkg --print-architecture)" = amd64 ] || { echo "This CUDA example requires amd64" >&2; exit 1; }
# apt-get install -y --no-install-recommends ca-certificates wget
# CUDA_KEYRING="$(mktemp --suffix=.deb)"
# wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -O "$CUDA_KEYRING"
# dpkg -i "$CUDA_KEYRING"
# rm -f "$CUDA_KEYRING"
# apt-get update
# apt-get install -y --no-install-recommends cuda-toolkit-12-4
# Use /usr/local/cuda/bin/nvcc when compiling CUDA code.
