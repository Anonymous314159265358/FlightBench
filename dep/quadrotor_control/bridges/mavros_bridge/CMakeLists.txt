cmake_minimum_required(VERSION 2.8.3)
project(mavros_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(vbat_thrust_calibration src/vbat_thrust_calibration.cpp)
cs_add_executable(mavros_bridge src/mavros_bridge_node.cpp src/mavros_bridge.cpp 
    src/thrust_mapping_params.cpp src/thrust_mapping.cpp)
target_link_libraries(mavros_bridge vbat_thrust_calibration)

cs_install()
cs_export()
