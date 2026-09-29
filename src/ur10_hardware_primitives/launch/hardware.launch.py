"""UR10 CB3 + USB 2F-140, MoveIt, workcell geometry, and hardware primitives."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ur10_hardware_primitives.configuration import description, load_cell, merge, moveit_parameters
from ur10_hardware_primitives.launch_helpers import primitive_nodes


def start(context):
    cell_path = Path(LaunchConfiguration('cell_config').perform(context)).resolve()
    cell = load_cell(cell_path)
    package = Path(share('ur10_hardware_primitives'))
    driver = Path(share('ur_robot_driver'))
    robot = description(cell_path, cell)
    # Both processes receive exactly the same calibrated URDF. The official
    # launch still owns dashboard, external-control, and controller lifecycle.
    temporary = tempfile.TemporaryDirectory(prefix='tim-ur10-hardware-')
    folder = Path(temporary.name)
    model_path = folder / 'ur10_2f140.urdf'
    model_path.write_text(robot)
    controllers = merge(yaml.safe_load((driver / 'config/ur_controllers.yaml').read_text()),
                        yaml.safe_load((package / 'config/controllers.yaml').read_text()))
    controllers_path = folder / 'controllers.yaml'
    controllers_path.write_text(yaml.safe_dump(controllers))
    moveit = moveit_parameters(robot, package / 'config/moveit_controllers.yaml')
    def cleanup(context):
        temporary.cleanup()
    nodes = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver / 'launch/ur_control.launch.py')),
            launch_arguments={
                'ur_type': 'ur10', 'robot_ip': cell['robot_ip'],
                'runtime_config_package': 'ur10_hardware_primitives',
                'controllers_file': str(controllers_path),
                'description_package': 'ur_description', 'description_file': str(model_path),
                'kinematics_params_file': cell['kinematics_file'],
                'initial_joint_controller': 'scaled_joint_trajectory_controller',
                'activate_joint_controller': 'true', 'use_fake_hardware': 'false',
                'use_tool_communication': 'false', 'headless_mode': 'false', 'launch_rviz': 'false',
            }.items()),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['robotiq_gripper_controller', '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '60']),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[moveit], output='screen'),
        Node(package='rviz2', executable='rviz2', parameters=[moveit], output='screen',
             arguments=['-d', str(Path(share('ur_moveit_config')) / 'rviz/view_robot.rviz')],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ]
    nodes += primitive_nodes(cell_path, cell)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cell_config', description='Absolute path to measured cell.yaml'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        OpaqueFunction(function=start),
    ])
