"""Start only service adapters; robot drivers and services run externally."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    enable_gripper = LaunchConfiguration('enable_gripper').perform(context).lower()
    if enable_gripper not in ('true', 'false'):
        raise ValueError('enable_gripper must be true or false')
    primitives = ['move']
    if enable_gripper == 'true':
        primitives.extend(['pick', 'place'])
    primitives.append('safe_stop')
    return [Node(
        package='primitive_manager', executable='primitive_manager',
        name='ur10_service_manager', output='screen',
        parameters=[LaunchConfiguration('config').perform(context), {
            'primitives': primitives,
        }],
    )]


def generate_launch_description():
    package = Path(get_package_share_directory('ur10_hardware_primitives'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(package / 'config/services.yaml')),
        DeclareLaunchArgument('enable_gripper', default_value='true',
                              description='Load pick/place service adapters; false loads only move'),
        OpaqueFunction(function=start),
    ])
