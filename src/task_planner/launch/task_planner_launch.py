# #!/usr/bin/env python3

# # File: plansys2_simple_launch.py
# # Place this in your task_planner package's launch directory

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration


# def generate_launch_description():
    
#     # Declare launch arguments
#     model_file_arg = DeclareLaunchArgument(
#         'model_file',
#         default_value='',
#         description='Path to PDDL domain file'
#     )
    
#     return LaunchDescription([
#         model_file_arg,
        
#         # Domain Expert Node
#         Node(
#             package='plansys2_domain_expert',
#             executable='domain_expert_node',
#             name='domain_expert',
#             output='screen',
#             parameters=[
#                 {'model_file': LaunchConfiguration('model_file')},
#             ]
#         ),
        
#         # Problem Expert Node
#         Node(
#             package='plansys2_problem_expert', 
#             executable='problem_expert_node',
#             name='problem_expert',
#             output='screen'
#         ),
        
#         # Planner Node
#         Node(
#             package='plansys2_planner',
#             executable='planner_node', 
#             name='planner',
#             output='screen',
#             parameters=[
#                 {'plan_solver_plugins': ['plansys2_popf_plan_solver/POPFPlanSolver']},
#                 {'planner_timeout': 30.0}
#             ]
#         ),
        
#         # Executor Node
#         Node(
#             package='plansys2_executor',
#             executable='executor_node',
#             name='executor', 
#             output='screen'
#         ),
        
#         # Task Planner Node
#         Node(
#             package='task_planner',
#             executable='task_planner_node',
#             name='task_planner_node',
#             output='screen'
#         )
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_planner',
            executable='task_planner_node',
            name='task_planner_node',
            output='screen'
        )
    ])
