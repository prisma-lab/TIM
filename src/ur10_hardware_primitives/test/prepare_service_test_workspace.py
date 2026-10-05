#!/usr/bin/env python3
"""Create an isolated source overlay containing test-only Pick/Place interfaces.

Usage: python3 prepare_service_test_workspace.py /tmp/tim_service_tests
Build it after sourcing the normal workspace. Never source this overlay when
connecting to a robot; the real provider must supply its production interfaces.
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
    cmake = interfaces / 'CMakeLists.txt'
    text = cmake.read_text()
    for name in ('Pick', 'Place'):
        relative = f'srv/motion_planner/{name}.srv'
        if not (interfaces / relative).exists():
            shutil.copyfile(Path(__file__).parent / 'service_interfaces' / f'{name}.srv',
                            interfaces / relative)
        if relative not in text:
            text = text.replace('    DEPENDENCIES', f'    {relative}\n    DEPENDENCIES')
    cmake.write_text(text)
    for package in ('primitive_manager', 'ur10_hardware_primitives'):
        (source / package).symlink_to(repository_source / package, target_is_directory=True)
    print(f'Test workspace prepared at {workspace}')


if __name__ == '__main__':
    main()
