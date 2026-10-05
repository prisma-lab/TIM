#!/usr/bin/env bash
# Open a hardware development shell; this does not launch drivers or move robots.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 /dev/serial/by-id/ADAPTER [container_name]" >&2
    exit 1
fi
serial_device="$(readlink -f -- "$1")"
container_name="${2:-tim_ur10_hardware}"
if [[ ! -c "$serial_device" ]]; then
    echo "Not a serial character device: $1" >&2
    exit 1
fi
if docker container inspect "$container_name" >/dev/null 2>&1; then
    echo "Container already exists. Use ./docker_attach.sh $container_name or choose a new name." >&2
    exit 1
fi
display_args=()
if [[ -n "${DISPLAY:-}" ]]; then
    "$repo_dir/docker_sim_auth.sh"
    display_args=(--env "DISPLAY=$DISPLAY" --env XAUTHORITY=/run/tim/Xauthority
        --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1
        --mount "type=bind,source=$repo_dir/.runtime/simulation,target=/run/tim,readonly"
        --mount 'type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly')
fi
exec docker run -it --init --net=host --ipc=host "${display_args[@]}" \
    --name "$container_name" \
    --device "$serial_device:/dev/robotiq" \
    --group-add "$(stat -c '%g' "$serial_device")" \
    --mount "type=bind,source=$repo_dir/src,target=/home/user/ros2_ws/src" \
    -e ROS_DOMAIN_ID=11 \
    tim_ur10_hardware_img bash
