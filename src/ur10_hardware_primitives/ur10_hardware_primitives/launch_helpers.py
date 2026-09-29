"""Shared startup of collision geometry and the hardware primitive manager."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from .configuration import primitive_parameters


def primitive_nodes(cell_path, cell, parameters_file=None):
    package = Path(get_package_share_directory('ur10_hardware_primitives'))
    scene = Node(package='ur10_hardware_primitives', executable='workcell_scene.py',
                 parameters=[{'cell_config': str(cell_path)}], output='screen')
    manager = Node(package='primitive_manager', executable='primitive_manager',
                   name='ur10_primitive_manager', output='screen',
                   parameters=[str(parameters_file or package / 'config/primitives.yaml'),
                               primitive_parameters(cell)])
    def after_scene(event, context):
        if event.returncode == 0:
            return [manager]
        return [EmitEvent(event=Shutdown(reason='Workcell setup failed; primitives not started'))]
    return [RegisterEventHandler(OnProcessExit(target_action=scene, on_exit=after_scene)), scene]
