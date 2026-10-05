#!/usr/bin/env bash
# Open a hardware development shell; this does not launch drivers or move robots.
# Service mode is the default; a serial device is mapped only when supplied.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    echo "Usage: $0 [--services [container_name]]"
    echo "       $0 /dev/serial/by-id/ADAPTER [container_name]"
    echo "With no arguments, creates the tim_ur10_services container without USB access."
    echo "ROS_DOMAIN_ID defaults to 11; set it to match the service provider."
}

if [[ $# -eq 1 && ( "$1" == --help || "$1" == -h ) ]]; then
    usage
    exit 0
fi
if [[ $# -gt 2 ]]; then
    usage >&2
    exit 1
fi

device_args=()
if [[ $# -eq 0 || "${1:-}" == --services ]]; then
    container_name="${2:-tim_ur10_services}"
else
    serial_device="$(readlink -f -- "$1")"
    container_name="${2:-tim_ur10_hardware}"
    if [[ ! -c "$serial_device" ]]; then
        echo "Not a serial character device: $1" >&2
        exit 1
    fi
    device_args=(--device "$serial_device:/dev/robotiq"
        --group-add "$(stat -c '%g' "$serial_device")")
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
    "${device_args[@]}" \
    --mount "type=bind,source=$repo_dir/src,target=/home/user/ros2_ws/src" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-11}" \
    tim_ur10_hardware_img bash
