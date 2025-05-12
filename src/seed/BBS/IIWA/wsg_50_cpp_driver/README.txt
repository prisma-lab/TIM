This code has been extracted and adapted from the following ROS1 repository:

https://github.com/nalt/wsg50-ros-pkg/tree/master

The idea is to isolate the cpp driver from the ROS1 code, so to have an interface with the WSG50
that does not depend on ROS.

The overall code is executed in the main_cpp.cpp file.

How to compile and run:

	1. go to the project directory

	2. mkdir build
	
	3. cd build
	
	4. cmake ../
	
	5. make
	
	6. ./wsg_50_ip
