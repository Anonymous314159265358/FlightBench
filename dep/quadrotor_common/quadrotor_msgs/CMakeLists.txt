cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)
include_directories(${EIGEN3_INCLUDE_DIR})

add_dependencies(encode_msgs quadrotor_msgs_generate_messages_cpp)
add_dependencies(decode_msgs quadrotor_msgs_generate_messages_cpp)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES encode_msgs decode_msgs
  CATKIN_DEPENDS geometry_msgs nav_msgs
  #DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

cs_install()
cs_export()
