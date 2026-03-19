cmake_minimum_required(VERSION 3.10)
project(inverse_msgs)

# Build custom msgs
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    srv/motion_planner/HoldPosition.srv
    srv/motion_planner/MoveRelative.srv
    srv/motion_planner/PointToPointMotion.srv
    srv/motion_planner/ReachPosition.srv
    srv/motion_planner/LearnSkill.srv
    srv/motion_planner/ExecuteSkill.srv
    srv/recording/ListTopics.srv
    srv/recording/StartRecording.srv
    srv/recording/StopRecording.srv
    DEPENDENCIES 
    std_msgs
    geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
