cmake_minimum_required(VERSION 2.8.3)
project(robomagellan_bringup)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
