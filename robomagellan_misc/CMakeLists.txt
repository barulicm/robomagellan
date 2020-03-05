cmake_minimum_required(VERSION 2.8.3)
project(robomagellan_misc)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_subdirectory(nodes/sparkfun_razor_imu_node)
