import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    custom_pkg_dir = get_package_share_directory('use_case_sim')
    xacro_file = os.path.join(custom_pkg_dir, 'urdf', 'assembly_env.urdf.xacro')
    peg_urdf = os.path.join(custom_pkg_dir, 'urdf', 'peg.urdf')
    connector_urdf = os.path.join(custom_pkg_dir, 'urdf', 'connector.urdf')
    rviz_config = os.path.join(custom_pkg_dir, 'rviz', 'default.rviz')
    custom_controllers = os.path.join(custom_pkg_dir, 'config', 'task_controllers.yaml')

    robotiq_install_dir = get_package_prefix('robotiq_description')
    gazebo_mesh_fix = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.path.join(robotiq_install_dir, 'share'), ':']
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file,
        ' simulation_controllers:=', custom_controllers
    ])
    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(package='robot_state_publisher', executable='robot_state_publisher', output='both', parameters=[robot_description])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
    )

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', 'ur10_peg_task'], output='screen')

    # Dropped safely from Z=0.9
    spawn_connector = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', connector_urdf, '-entity', 'red_connector', '-x', '0.55', '-y', '0.0', '-z', '0.9'],
        output='screen'
    )

    # Placed on the far right (Y=-0.4). Rolled 90 degrees (-R) to lay flat and point at the hole.
    spawn_peg = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', peg_urdf, '-entity', 'blue_peg', '-x', '0.7', '-y', '-0.4', '-z', '0.9', '-R', '1.5708'],
        output='screen'
    )

    # Delay the spawn of the loose objects to ensure the table collision mesh is fully loaded
    delayed_spawns = TimerAction(
        period=4.0,
        actions=[spawn_connector, spawn_peg]
    )

    rviz_node = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config])
    
    joint_state_broadcaster = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    joint_trajectory_controller = Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'])
    robotiq_gripper_controller = Node(package='controller_manager', executable='spawner', arguments=['robotiq_gripper_controller', '--controller-manager', '/controller_manager'])

    return LaunchDescription([
        gazebo_mesh_fix,
        robot_state_pub_node,
        gazebo,
        spawn_robot,
        delayed_spawns,
        rviz_node,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[joint_trajectory_controller, robotiq_gripper_controller])),
    ])
