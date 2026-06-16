from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ln_to_crf_ros2_interface',
            executable='topic_manager',
            # name='topic_manager',
            output='screen',
        ),
        Node(
            package='ln_to_crf_ros2_interface',
            executable='service_manager',
            # name='service_manager',
            output='screen',
        ),
    ])
