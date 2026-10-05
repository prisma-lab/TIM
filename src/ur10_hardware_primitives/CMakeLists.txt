cmake_minimum_required(VERSION 3.8)
project(ur10_hardware_primitives)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
foreach(dep primitive_manager pluginlib rclcpp rclcpp_action control_msgs sensor_msgs
    geometry_msgs trajectory_msgs moveit_msgs tf2_ros tf2_geometry_msgs shape_msgs inverse_msgs std_msgs)
  find_package(${dep} REQUIRED)
endforeach()
add_library(ur10_hardware_plugins SHARED src/gripper_primitive.cpp src/manipulation_primitive.cpp
  src/move_a_b_primitive.cpp src/pick_primitive.cpp src/place_primitive.cpp)
target_compile_features(ur10_hardware_plugins PUBLIC cxx_std_17)
target_compile_options(ur10_hardware_plugins PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(ur10_hardware_plugins PRIVATE include)
ament_target_dependencies(ur10_hardware_plugins primitive_manager pluginlib rclcpp rclcpp_action
  control_msgs sensor_msgs geometry_msgs trajectory_msgs moveit_msgs tf2_ros tf2_geometry_msgs shape_msgs)
pluginlib_export_plugin_description_file(primitive_manager primitives_plugins.xml)
add_library(ur10_service_plugins SHARED
  src/services/motion_service_primitive.cpp
  src/services/move_service_primitive.cpp
  src/services/gripper_service_primitives.cpp)
target_compile_features(ur10_service_plugins PUBLIC cxx_std_17)
target_compile_options(ur10_service_plugins PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(ur10_service_plugins PRIVATE include)
ament_target_dependencies(ur10_service_plugins
  primitive_manager pluginlib rclcpp inverse_msgs std_msgs geometry_msgs tf2_ros)
pluginlib_export_plugin_description_file(primitive_manager services_plugins.xml)
install(TARGETS ur10_hardware_plugins ur10_service_plugins DESTINATION lib)
install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/workcell_scene.py scripts/target_publisher.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(hardware_protocol test/test_hardware_protocol.py TIMEOUT 240
    APPEND_ENV ROS_DOMAIN_ID=128 PYTHONDONTWRITEBYTECODE=1)
  ament_add_pytest_test(hardware_configuration test/test_configuration.py TIMEOUT 60)
  ament_add_pytest_test(service_protocol test/test_service_protocol.py TIMEOUT 180
    APPEND_ENV ROS_DOMAIN_ID=146 PYTHONDONTWRITEBYTECODE=1)
endif()
ament_package()
