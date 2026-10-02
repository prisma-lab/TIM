"""Ground an image and plan a two-object task without rebuilding scene data."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('task_planner'))
    defaults = {
        'image': str(share / 'resource/example_problem_image.png'),
        'domain': str(share / 'pddl/serdar_two_objects_domain.pddl'),
        'mapping': str(share / 'config/serdar_two_objects_action_mapping.yaml'),
        'scene': str(share / 'resource/descriptions/scene.txt'),
        'goal': str(share / 'resource/descriptions/goal.txt'),
        'predicate_definitions': str(share / 'resource/descriptions/predicates.txt'),
        'model': 'qwen3-vl:8b-instruct', 'endpoint': 'http://127.0.0.1:11434/api/chat',
        'mode': 'full', 'context': '', 'output_directory': '', 'autostart': 'true',
        'vlm_timeout': '600.0', 'discovery_timeout': '15.0',
        'planner_executable': 'fast-downward.py', 'planner_build': '',
        'planner_search': 'astar(blind())', 'planner_timeout': '30.0',
        'execute': 'false', 'seed_topic': '/seed_ur10_two_objects/stream',
        'wait_for_seed': '10.0', 'planning_topic': '/two_objects/planning_request',
    }
    def parameters(names):
        return {name: ParameterValue(LaunchConfiguration(name), value_type=(
            bool if name in ('execute', 'autostart') else float if name in (
                'vlm_timeout', 'discovery_timeout', 'planner_timeout', 'wait_for_seed') else str))
                for name in names}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(package='task_planner', executable='two_objects_planner_node', output='screen',
             parameters=[parameters(('mapping', 'planner_executable', 'planner_build',
                         'planner_search', 'planner_timeout', 'execute', 'seed_topic',
                         'wait_for_seed', 'planning_topic'))]),
        Node(package='task_planner', executable='vlm_grounder_node', output='screen',
             parameters=[parameters(('image', 'autostart', 'domain', 'scene', 'goal', 'predicate_definitions',
                         'model', 'endpoint', 'mode', 'context', 'output_directory',
                         'vlm_timeout', 'discovery_timeout', 'planning_topic'))]),
    ])
