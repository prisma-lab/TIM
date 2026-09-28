#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
    echo "Usage: $0 [image_name]" >&2
    exit 1
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
image_name="${1:-tim_img}"

exec docker build --progress=plain -f "$repo_dir/Dockerfile" \
    -t "$image_name" --build-arg USER_ID="$(id -u)" \
    --build-arg GROUP_ID="$(id -g)" "$repo_dir"
