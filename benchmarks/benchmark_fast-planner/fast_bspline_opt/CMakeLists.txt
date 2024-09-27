cmake_minimum_required(VERSION 2.8.3)
project(fast_bspline_opt)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  fast_plan_env
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fast_bspline_opt
 CATKIN_DEPENDS fast_plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIR}
)

message(STATUS "Nlopt lib path: ${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( fast_bspline_opt 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( fast_bspline_opt
    ${catkin_LIBRARIES} 
    ${NLOPT_LIBRARIES}
    )  
