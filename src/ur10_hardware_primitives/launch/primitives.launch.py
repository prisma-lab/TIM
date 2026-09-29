"""Use an already running combined hardware model, UR driver, and MoveIt."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ur10_hardware_primitives.configuration import load_cell
from ur10_hardware_primitives.launch_helpers import primitive_nodes


def start(context):
    cell_path = LaunchConfiguration('cell_config').perform(context)
    config = LaunchConfiguration('primitives_config').perform(context)
    return primitive_nodes(cell_path, load_cell(cell_path), config or None)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cell_config', description='Absolute path to measured cell.yaml'),
        DeclareLaunchArgument('primitives_config', default_value=''),
        OpaqueFunction(function=start),
    ])
