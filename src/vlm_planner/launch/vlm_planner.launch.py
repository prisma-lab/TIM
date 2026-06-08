"""
Launch file for VLM Task Planner.

Usage:
  ros2 launch vlm_task_planner vlm_planner.launch.py

Override defaults with:
  ros2 launch vlm_task_planner vlm_planner.launch.py \
      domain_path:=/absolute/path/to/domain.pddl \
      init_image_path:=/absolute/path/to/init.jpeg \
      goal_image_path:=/absolute/path/to/goal.jpeg \
      gemini_api_key:=AIza...
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("domain_path", default_value="domain.pddl"),
            DeclareLaunchArgument("init_image_path", default_value="init.jpeg"),
            DeclareLaunchArgument("goal_image_path", default_value="goal.jpeg"),
            DeclareLaunchArgument("gemini_api_key", default_value="YOUR_GEMINI_API_KEY"),
            DeclareLaunchArgument("gemini_model", default_value="gemini-2.0-flash"),
            DeclareLaunchArgument("topic_name", default_value="/planning_request"),
            Node(
                package="vlm_task_planner",
                executable="vlm_planner_node",
                name="vlm_task_planner_node",
                output="screen",
                parameters=[
                    {
                        "domain_path": LaunchConfiguration("domain_path"),
                        "init_image_path": LaunchConfiguration("init_image_path"),
                        "goal_image_path": LaunchConfiguration("goal_image_path"),
                        "gemini_api_key": LaunchConfiguration("gemini_api_key"),
                        "gemini_model": LaunchConfiguration("gemini_model"),
                        "topic_name": LaunchConfiguration("topic_name"),
                    }
                ],
            ),
        ]
    )
