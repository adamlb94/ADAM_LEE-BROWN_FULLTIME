cmake_minimum_required(VERSION 3.0.2)
project(multi_agent_planning)

## Compile as C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## catkin specific configuration
catkin_package()

## Build
include_directories(include ${catkin_INCLUDE_DIRS})
