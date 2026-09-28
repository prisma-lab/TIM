#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -gt 1 ]]; then
    echo "Usage: $0 [path/to/use_case_sim]" >&2
    exit 1
fi

# The default scene is bundled in TIM; an external package is optional.
sim_package_dir="${1:-$repo_dir/simulation/use_case_sim}"
if [[ ! -f "$sim_package_dir/package.xml" || ! -f "$sim_package_dir/CMakeLists.txt" ||
      ! -f "$sim_package_dir/launch/assembly_task.launch.py" ]]; then
    echo "Expected the use_case_sim package at: $sim_package_dir" >&2
    echo "Usage: $0 [path/to/use_case_sim]" >&2
    exit 1
fi

exec docker build --progress=plain -f "$repo_dir/Dockerfile.sim" \
    --build-context "use_case_sim=$sim_package_dir" \
    -t tim_ur10_img "$repo_dir"
