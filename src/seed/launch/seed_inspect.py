from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seed',
            executable='seed',
            name='mimic',
            output="screen",
            #output={"both","drone_ros2_log.txt"},
            arguments=["inspect_drone"],
            parameters=[
               #{"frames_to_explore": ["exp00","exp10","exp11","exp12","exp21","exp32","exp30","exp41","exp51","exp60"]} #Leonardo-simulation
               {"frames_to_explore": ["exp00","exp10","exp01","exp11"]} #UNINA-real
            ]
        )
    ])
