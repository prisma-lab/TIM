from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    apriltag_config = os.path.join(
        get_package_share_directory('seed'),
        'launch',
        'apriltag.yaml'
    )

    return LaunchDescription([
        # Node(
        #     package='seed',
        #     executable='seed',
        #     name='mimic',
        #     output="screen",
        #     #output={"both","drone_ros2_log.txt"},
        #     arguments=["TIM"]
        # ),
        Node(
            package='task_planner',
            executable='planner_node',
            name='mimic',
            output="screen",
            #output={"both","drone_ros2_log.txt"},
            arguments=[]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="iiwa_to_world",
            arguments=["0", "0.0580", "-0.1210", "-1.5707963", "-1.5707963", "0", "marker_0", "iiwa_base_link"],
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="iiwa_to_home",
            arguments=["0.4", "0", "0.35", "0", "3.14", "0", "iiwa_base_link", "home"],
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="iiwa_to_adj",
            arguments=["0.8", "0", "0.3", "-2.6", "0.11", "-1.56", "iiwa_base_link", "adj"],
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="iiwa_to_test1",
            arguments=["0.4", "-0.2", "0.2", "0", "3.14", "0", "iiwa_base_link", "test"],
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="test1_to_test1_final",
            arguments=["0.15", "0", "0.0", "0", "0", "0", "robot_bottle.3", "test1_final"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="iiwa_to_cam2",
        #     #arguments=["0.186", "0.002", "-0.064", "0", "0", "0", "iiwa_base_link", "frontal_camera_link"], # NOTE: yaw, pitch, roll
        #     arguments=["0.186", "-0.055", "-0.015", "0", "0", "0", "iiwa_base_link", "frontal_camera_link"], # NOTE: yaw, pitch, roll
        #     # output="screen",
        # ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="marker_to_brick11",
            arguments=["0", "0.0", "-0.025", "1.57", "-1.57", "0", "marker_11", "brick11"], # NOTE: yaw, pitch, roll
            #arguments=["0", "0.05", "0", "0", "-4.71", "4.71", "marker_11", "brick11"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick11_to_over",
            arguments=["0", "0.0", "-0.03", "0", "0", "0", "brick11", "brick11.over"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick11_to_pre",
            arguments=["0", "0.0", "-0.057", "0", "0", "0", "brick11", "brick11.pre"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="marker_to_brick10",
            arguments=["0", "0.0", "-0.025", "1.57", "-1.57", "0", "marker_10", "brick10"], # NOTE: yaw, pitch, roll
            #arguments=["0", "0.05", "0", "0", "-4.71", "4.71", "marker_11", "brick11"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick10_to_over",
            arguments=["0", "0.0", "-0.03", "0", "0", "0", "brick10", "brick10.over"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick10_to_pre",
            arguments=["0", "0.0", "-0.057", "0", "0", "0", "brick10", "brick10.pre"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        
        
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="marker_to_brick20",
            arguments=["0", "0.0", "-0.025", "1.57", "-1.57", "0", "marker_20", "brick20"], # NOTE: yaw, pitch, roll
            #arguments=["0", "0.05", "0", "0", "-4.71", "4.71", "marker_11", "brick11"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick20_to_over",
            arguments=["0", "0.0", "-0.05", "0", "0", "0", "brick20", "brick20.over"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick20_to_pre",
            arguments=["0", "0.0", "-0.1", "0", "0", "0", "brick20", "brick20.pre"], # NOTE: yaw, pitch, roll
            # output="screen",
        ),
        
        
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="marker_to_brick21",
            arguments=["0", "0.0", "-0.025", "1.57", "-1.57", "0", "marker_21", "brick21"], # NOTE: yaw, pitch, roll
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick21_to_over",
            arguments=["0", "0.0", "-0.05", "0", "0", "0", "brick21", "brick21.over"], # NOTE: yaw, pitch, roll
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick21_to_pre",
            arguments=["0", "0.0", "-0.1", "0", "0", "0", "brick21", "brick21.pre"], # NOTE: yaw, pitch, roll
        ),
        
        
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="marker_to_brick22",
            arguments=["0", "0.0", "-0.025", "1.57", "-1.57", "0", "marker_22", "brick22"], # NOTE: yaw, pitch, roll
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick22_to_over",
            arguments=["0", "0.0", "-0.05", "0", "0", "0", "brick22", "brick22.over"], # NOTE: yaw, pitch, roll
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="brick22_to_pre",
            arguments=["0", "0.0", "-0.1", "0", "0", "0", "brick22", "brick22.pre"], # NOTE: yaw, pitch, roll
        ),
        
        
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="iiwa_to_gnd1",
            arguments=["0.57", "-0.144", "-0.050", "0", "3.14", "0", "iiwa_base_link", "gnd1"], #"gnd1.over"],
        ),
        
        #Node(
        #    package="tf2_ros",
        #    executable="static_transform_publisher",
        #    name="gnd1_to_pre",
        #    arguments=["0", "0", "-0.01", "0", "0", "0", "gnd1.over", "gnd1.pre"],
        #),
        
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="iiwa_to_gnd2",
            arguments=["0.50", "-0.030", "-0.050", "0", "3.14", "0", "iiwa_base_link", "gnd2"], #"gnd2.over"],
        ),
        
        #Node(
        #    package="tf2_ros",
        #    executable="static_transform_publisher",
        #    name="gnd2_to_pre",
        #    arguments=["0", "0", "-0.01", "0", "0", "0", "gnd2.over", "gnd2.pre"],
        #),
        
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="iiwa_to_gnd3",
            arguments=["0.57", "0.084", "-0.050", "0", "3.14", "0", "iiwa_base_link", "gnd3"], #"gnd3.over"],
        ),
        
        #Node(
        #    package="tf2_ros",
        #    executable="static_transform_publisher",
        #    name="gnd3_to_pre",
        #    arguments=["0", "0", "-0.01", "0", "0", "0", "gnd3.over", "gnd3.pre"],
        #),
        
        
        #Node(
        #    package="tf2_ros",
        #    executable="static_transform_publisher",
        #    name="world_to_b1",
        #    arguments=["0.1", "-0.15", "0.3", "0", "-2.2", "0", "marker_1", "bottle.1"], # NOTE: yaw, pitch, roll
        #    # output="screen",
        #),
        #Node(
        #    package="tf2_ros",
        #    executable="static_transform_publisher",
        #    name="world_to_b2",
        #    arguments=["0.1", "-0.15", "0.15", "0", "-2.2", "0", "marker_1", "bottle.2"], # NOTE: yaw, pitch, roll
        #    # output="screen",
        #),
        #Node(
        #    package="tf2_ros",
        #    executable="static_transform_publisher",
        #    name="world_to_b3",
        #    arguments=["0.0", "-0.15", "0.15", "0", "-2.2", "0", "marker_1", "bottle.3"], # NOTE: yaw, pitch, roll
        #    # output="screen",
        #),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('seed'), 'launch'), '/iiwa_upper_rs_launch.py'] ),
            launch_arguments = { 
                          "align_depth.enable" : "false",
            			  #"depth_module.depth_profile" : "640x480x30", #"1920x1080x30",
            			  "rgb_camera.color_profile" :   "1920x1080x15", #"640x480x30", #"1920x1080x30", #1280x720x30
            			  #"depth_qos": "SENSOR_DATA", #"SYSTEM_DEFAULT",  
            			  "color_qos": "SENSOR_DATA", #"SYSTEM_DEFAULT",  
            			  "serial_no": "'213322074516'",
            			  "camera_name": "frontal_camera",
            			  "camera_namespace": "",
                          "enable_depth": "false",  
                          #"enable_sync": "True",
            			  }.items(),
        ),

        # IncludeLaunchDescription(
        #     XMLLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('seed'), 'launch'),
        #     '/seed_aruco_4x4.launch.xml'])
        # ),


        # Apritag node for world marker 
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[apriltag_config],
            remappings=[
                ('image_rect',  '/frontal_camera/color/image_raw'),
                ('camera_info', '/frontal_camera/color/camera_info')
            ]
        ),

        # Aruco marker for objects in the scene
        IncludeLaunchDescription(
             XMLLaunchDescriptionSource([os.path.join(
             get_package_share_directory('seed'), 'launch'),
             '/seed_aruco_5x5.launch.xml'])
         ),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('seed'), 'launch'), '/iiwa_upper_rs_launch.py'] ),
        #     launch_arguments = { "align_depth.enable" : "True",
        #     			  "depth_module.depth_profile" : "640x480x30",
        #     			  "rgb_camera.color_profile" : "640x480x30", 
        #     			  "depth_qos": "SENSOR_DATA",
        #     			  "color_qos": "SENSOR_DATA",
        #     			  "serial_no": "'827112072041'",
        #     			  "camera_name": "camera",
        #     			  "camera_namespace": ""
        #     			  }.items(),
        # ),
        # IncludeLaunchDescription(
        #     XMLLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('seed'), 'launch'),
        #     '/seed_aruco_4x4.launch.xml'])
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('seed'), 'launch'), '/iiwa_frontal_rs_launch.py'] ),
        #     launch_arguments = { "align_depth.enable" : "True",
        #     			  "depth_module.depth_profile" : "640x480x30",
        #     			  "rgb_camera.color_profile" : "640x480x30", 
        #     			  "depth_qos": "SENSOR_DATA",
        #     			  "color_qos": "SENSOR_DATA",
        #     			  "serial_no": "'213322071559'",
        #     			  "camera_name": "frontal_camera",
        #     			  "camera_namespace": ""
        #     			  }.items(),
        # ),
        
    ])
