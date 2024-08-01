cmake_minimum_required(VERSION 3.0.2)
project(deeporange14_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  can_dbc_parser
  sensor_msgs
  geometry_msgs
  deeporange14_msgs
  roslaunch
  message_generation
  novatel_oem7_msgs
  roslint
  roslaunch
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
  deeporange14_msgs
  nav_msgs
  roscpp
  rospy
  novatel_oem7_msgs

)

catkin_package(
  LIBRARIES
  ${PROJECT_NAME}
  INCLUDE
  include/
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/DeepOrangeStateSupervisor.cpp
  src/DeepOrangeVelocityController.cpp
  src/DeepOrangeDbwCan.cpp
  src/DataLogger.cpp
)
add_executable(${PROJECT_NAME}_dbwnode src/DeepOrangeInterfaceNode.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_dbwnode ${PROJECT_NAME}
)

install(DIRECTORY launch src dbc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-readability/todo,-whitespace/todo,-build/header_guard, -whitespace/braces, -runtime/references")
  roslint_cpp()
  roslint_add_test()

  roslaunch_add_file_check(launch/control.launch)
endif()