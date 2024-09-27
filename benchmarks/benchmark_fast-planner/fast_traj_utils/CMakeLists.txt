cmake_minimum_required(VERSION 2.8.3)
project(fast_traj_utils)

find_package(catkin REQUIRED COMPONENTS
  bspline
  fast_bspline_opt
  fast_path_searching
  poly_traj
  roscpp
  std_msgs
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fast_traj_utils
 CATKIN_DEPENDS fast_path_searching bspline fast_bspline_opt poly_traj
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( fast_traj_utils
    src/planning_visualization.cpp 
    )
target_link_libraries( fast_traj_utils
    ${catkin_LIBRARIES} 
    )  