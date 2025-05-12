cmake_minimum_required(VERSION 2.8.3)
project(wsg_50_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# STANDALONE NOTES:
#  To build this code standalone, go/create the \build directory
#   run "cmake .."
#   then run "make" command
#
#  To run the code just run ./wsg50_eth from the build dir.


# INCLUDE NOTES:
# To include this driver into your code you have to add the following sources
#  as well as the "include" directory to your project

# WSG_50_ETH version
set(WSG50_ETH_DRIVER_SOURCES
  src/checksum.cpp include/wsg_50/checksum.h
  src/cmd.c include/wsg_50/cmd.h
  src/common.cpp include/wsg_50/common.h
  src/functions.cpp include/wsg_50/functions.h
  src/interface.cpp include/wsg_50/interface.h
  #src/main.cpp
  src/msg.c include/wsg_50/msg.h
  src/serial.c include/wsg_50/serial.h
  src/tcp.c include/wsg_50/tcp.h
  src/udp.c include/wsg_50/udp.h
  src/wsg50_eth.cpp
  )

# WSG_50_CAN version
set(WSG50_ETH_DRIVER_SOURCES_CAN
  src/checksum.cpp
  src/common.cpp
  src/functions_can.cpp
  src/msg.c)

include_directories(
  include
)

add_definitions(-DOSNAME_LINUX)
add_definitions(-g)

# C++11 Needed
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_executable(wsg50_eth src/main_WSG50_eth.cpp ${WSG50_ETH_DRIVER_SOURCES})
#target_link_libraries(wsg_50_ip )
#add_dependencies(wsg_50_ip wsg_50_common_generate_messages_cpp)

#add_executable(wsg_50_can src/main_can.cpp src/checksum.cpp src/msg.c src/common.cpp src/functions_can.cpp)
#add_executable(wsg_50_can src/main_can.cpp ${DRIVER_SOURCES_CAN})

#include_directories(/home/marc/peak-linux-driver-7.5/driver/src/)
#link_directories(/home/marc/peak-linux-driver-7.5/lib/)
#add_compile_flags(wsg_50_can -g -Wall)
#target_link_libraries(wsg_50_can pcan)
