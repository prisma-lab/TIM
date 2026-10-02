#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
container_name="${1:-tim_ur10}"
auth_dir="$repo_dir/.runtime/simulation"

"$repo_dir/docker_sim_auth.sh"

docker info --format '{{.ServerVersion}}' >/dev/null
if docker container inspect "$container_name" >/dev/null 2>&1; then
    docker start "$container_name" >/dev/null
else
    docker run -dit --init --name "$container_name" --shm-size=1g \
        --env "DISPLAY=$DISPLAY" --env XAUTHORITY=/run/tim/Xauthority \
        --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1 \
        --mount "type=bind,source=$auth_dir,target=/run/tim,readonly" \
        --mount 'type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly' \
        --mount "type=bind,source=$repo_dir/src,target=/home/user/ros2_ws/src" \
        --network host \
        tim_ur10_img bash >/dev/null

fi

exec docker exec -it --env "DISPLAY=$DISPLAY" --env XAUTHORITY=/run/tim/Xauthority "$container_name" bash
