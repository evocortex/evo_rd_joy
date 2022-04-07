cmake_minimum_required(VERSION 2.8.3)
project(evo_rd_joy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs std_srvs tf
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/JoyToVel.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)