"""Gazebo-only UR10 / Robotiq 2F-85 demo; not a hardware bringup."""
from itertools import combinations
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def semantics(description):
    """Disable connected/fixed geometry and the gripper's internal linkage only."""
    robot = ET.fromstring(description)
    srdf = ET.Element('robot', name=robot.attrib['name'])
    group = ET.SubElement(srdf, 'group', name='ur_manipulator')
    ET.SubElement(group, 'chain', base_link='base_link', tip_link='tool0')
    links = [link.attrib['name'] for link in robot.findall('link')]
    parent = {name: name for name in links}

    def root(name):
        while parent[name] != name:
            name = parent[name]
        return name

    disabled = set()
    for joint in robot.findall('joint'):
        a, b = joint.find('parent').attrib['link'], joint.find('child').attrib['link']
        disabled.add(tuple(sorted((a, b))))
        if joint.attrib['type'] == 'fixed':
            parent[root(a)] = root(b)
    hand = [name for name in links if name.startswith('robotiq_85_')]
    for a, b in combinations(links, 2):
        if root(a) == root(b):
            disabled.add(tuple(sorted((a, b))))
    for a, b in combinations(hand + ['wrist_3_link'], 2):
        disabled.add(tuple(sorted((a, b))))
    for a, b in sorted(disabled):
        ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b, reason='Connected geometry')
    return ET.tostring(srdf, encoding='unicode')


def generate_launch_description():
    package = Path(get_package_share_directory('ur10_primitives'))
    scene = Path(get_package_share_directory('use_case_sim'))
    support = Path(get_package_share_directory('ur10_sim_support'))
    description = xacro.process_file(str(scene / 'urdf/assembly_env.urdf.xacro')).toxml()
    joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    moveit = {
        'use_sim_time': True,
        'robot_description': description,
        'robot_description_semantic': semantics(description),
        'robot_description_kinematics': {'ur_manipulator': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': 0.005,
            'kinematics_solver_timeout': 0.2}},
        'robot_description_planning': {'joint_limits': {
            name: {'has_velocity_limits': True, 'max_velocity': 1.0,
                   'has_acceleration_limits': True, 'max_acceleration': 1.0}
            for name in joints}},
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
            'path_tolerance': 0.01,
            'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.0}},
            'ur_manipulator': {'planner_configs': ['RRTConnect'], 'longest_valid_segment_fraction': 0.005}},
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_manage_controllers': False,
        'moveit_simple_controller_manager': {
            'controller_names': ['joint_trajectory_controller'],
            'joint_trajectory_controller': {'type': 'FollowJointTrajectory',
                                           'action_ns': 'follow_joint_trajectory',
                                           'default': True, 'joints': joints}},
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 4.0,
        'trajectory_execution.allowed_start_tolerance': 0.02,
        'publish_robot_description_semantic': True,
    }
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('target_publisher', default_value='true'),
        AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', str(Path(get_package_prefix('ur10_sim_support')) / 'lib')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(scene / 'launch/assembly_task.launch.py')),
            launch_arguments={'world': str(support / 'worlds/assembly_pick_place.world')}.items(),
            condition=IfCondition(LaunchConfiguration('simulation'))),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[moveit], output='screen'),
        Node(package='primitive_manager', executable='primitive_manager', name='ur10_primitive_manager',
             parameters=[str(package / 'config/pick_place.yaml')], output='screen'),
        Node(package='ur10_primitives', executable='target_publisher.py', name='pick_place_targets',
             parameters=[str(package / 'config/targets.yaml')], output='screen',
             condition=IfCondition(LaunchConfiguration('target_publisher'))),
    ])
