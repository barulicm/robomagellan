cmake_minimum_required(VERSION 2.8.3)
project(robomagellan_hardware_interface)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  hardware_interface
  controller_manager
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robomagellan_hardware_interface
  CATKIN_DEPENDS hardware_interface controller_manager sensor_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/robomagellan_hardware_interface.cpp src/robomagellan_hardware_interface_node.cpp src/SerialPort.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
