#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 CONTAINER_NAME" >&2
    exit 1
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
auth_mount="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/run/tim"}}{{.Source}}{{end}}{{end}}' "$1")"
# Starting an already running container does not restart it.
docker start "$1" >/dev/null

# Simulation containers need the current desktop cookie after a new login.
# Ordinary TIM containers and shells without a display retain normal behavior.
if [[ "$auth_mount" == "$repo_dir/.runtime/simulation" && -n "${DISPLAY:-}" ]]; then
    "$repo_dir/docker_sim_auth.sh"
    exec docker exec -it --env "DISPLAY=$DISPLAY" --env XAUTHORITY=/run/tim/Xauthority "$1" bash
fi
exec docker exec -it "$1" bash
