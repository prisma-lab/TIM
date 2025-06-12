cmake_minimum_required(VERSION 3.5)
project(seed)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (was C++14)
if(NOT CMAKE_CXX_STANDARD)
  #set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD 17)
endif()

# -g gives information for the sigsegv-backtrace
#add_compile_options(-g ) #-no-pie)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -no-pie" )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# do not use seed_interfaces package
add_compile_definitions(SEED_INTERFACES=0)

# from WSG50
#add_definitions(-DOSNAME_LINUX)
set(WSG50_ETH_DRIVER_C_SRC
  BBS/IIWA/wsg_50_cpp_driver/src/cmd.c
  BBS/IIWA/wsg_50_cpp_driver/src/msg.c
  BBS/IIWA/wsg_50_cpp_driver/src/serial.c
  BBS/IIWA/wsg_50_cpp_driver/src/tcp.c
  BBS/IIWA/wsg_50_cpp_driver/src/udp.c
  )


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Charts REQUIRED)

#find_package(seed_interfaces REQUIRED)

find_package(tf2_ros REQUIRED)

#INVERSE-specific libraries:
find_package(plansys2_msgs REQUIRED)
find_package(task_planner_msgs REQUIRED)

# let the compiler search for headers in the include folder
include_directories(
  include
  BBS
  BBS/IIWA/wsg_50_cpp_driver/include
  /usr/lib/swi-prolog/include
  ${OpenCV_INCLUDE_DIRS}
)
message("opencv: " ${OpenCV_INCLUDE_DIRS})
message("Qt5Widgets: " ${Qt5Charts_INCLUDE_DIRS})
message("Qt5Widgets: " ${Qt5Charts_INCLUDE_DIRS})

#add_executable(seed
#  src/seed.cpp
#  src/behavior.cpp
#  src/LTM_swipl.cpp
#  src/seed_wakeUp.cpp
#  src/WM.cpp
#  src/GUI/gui_tree.cpp
#)

# add ALL source files from src/ and BBS/
message("SEED CPP DETECTOR, following source files are found:")
file(GLOB_RECURSE seed_SRC
  "src/*.cpp"
)
file(GLOB_RECURSE seed_BBS
  "BBS/*.cpp"
)
foreach(file_path ${seed_SRC})
    message(${file_path})
endforeach()
foreach(file_path ${seed_BBS})
    message(${file_path})
endforeach()
add_executable(seed
  ${seed_SRC}
  ${seed_BBS}
  ${WSG50_ETH_DRIVER_C_SRC}
)
message("NOTE: if source files are missing, please clean before build")
# use: rm -r build install

ament_target_dependencies(seed 
  rclcpp 
  std_msgs 
  sensor_msgs 
  OpenCV 
  image_transport 
  cv_bridge 
  #seed_interfaces
  tf2_ros
  #inverse-specific:
  plansys2_msgs
  task_planner_msgs
)

target_link_libraries(seed
  # SWI-Prolog
  /usr/lib/swi-prolog/lib/x86_64-linux/libswipl.so
  # Graphviz (GVC)
  cgraph
  gvc
  # openCV2
  ${OpenCV_LIBS}
  opencv_core 
	opencv_highgui
	opencv_imgproc
	opencv_features2d
  # QT5
  Qt5::Widgets
  Qt5::Charts
  # miscellaneous
  espeak
)

install(TARGETS 
  seed
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
