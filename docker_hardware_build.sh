#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec docker build --progress=plain -f "$repo_dir/Dockerfile.hardware" -t tim_ur10_hardware_img "$repo_dir"
