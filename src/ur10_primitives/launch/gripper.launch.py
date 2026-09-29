"""Gazebo-only Robotiq 2F-85 gripper primitive configuration."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('ur10_primitives')) / 'config' / 'gripper.yaml'
    return LaunchDescription([
        Node(package='primitive_manager', executable='primitive_manager',
             name='ur10_primitive_manager', parameters=[str(config)], output='screen'),
    ])
