#!/usr/bin/env python3
"""Create an isolated source overlay using the production service interfaces.

Usage: python3 prepare_service_test_workspace.py /tmp/tim_service_tests
Build it after sourcing the normal workspace. Test services use separate ROS
domains and never contact the robot.
"""
from pathlib import Path
import shutil
import sys


def main():
    workspace = Path(sys.argv[1]).resolve()
    workspace.mkdir(parents=True, exist_ok=False)
    source = workspace / 'src'
    source.mkdir()
    repository_source = Path(__file__).resolve().parents[2]
    interfaces = source / 'inverse_msgs'
    shutil.copytree(repository_source / 'inverse_msgs', interfaces)
    for package in ('primitive_manager', 'ur10_hardware_primitives', 'task_planner'):
        (source / package).symlink_to(repository_source / package, target_is_directory=True)
    print(f'Test workspace prepared at {workspace}')


if __name__ == '__main__':
    main()
