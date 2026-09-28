#!/usr/bin/env bash
# Refresh the display cookie mounted by docker_sim_run.sh. Run on the host.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
auth_dir="$repo_dir/.runtime/simulation"
if [[ -z "${DISPLAY:-}" ]]; then
    echo 'Run this script from a terminal in your graphical desktop session.' >&2
    exit 1
fi
if ! command -v xauth >/dev/null; then
    echo 'The host needs xauth for the Gazebo GUI (sudo apt install xauth).' >&2
    exit 1
fi

mkdir -p "$auth_dir"
chmod 700 "$auth_dir"
umask 077
auth_temp="$(mktemp "$auth_dir/.Xauthority.XXXXXX")"
trap 'rm -f -- "$auth_temp" "$auth_temp-c" "$auth_temp-l"' EXIT
# FamilyWild makes the entry usable with the container's different hostname.
# Write a temporary file first: a failed refresh must preserve the working one.
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$auth_temp" nmerge -
if [[ ! -s "$auth_temp" ]]; then
    echo 'No X11 cookie was found for DISPLAY; check your desktop XAUTHORITY setting.' >&2
    exit 1
fi
mv -f -- "$auth_temp" "$auth_dir/Xauthority"
