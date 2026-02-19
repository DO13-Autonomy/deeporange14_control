cmake_minimum_required(VERSION 3.0.2)
project(deeporange14_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PKG_DEPENDS
  actionlib_msgs
  can_dbc_parser
  can_msgs
  deeporange14_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  socketcan_bridge
  std_msgs
)

set(BUILD_DEPENDS
  roslaunch
  roslint
  ${PKG_DEPENDS}
)

set(RUN_DEPENDS
  message_runtime
  ${PKG_DEPENDS}
)

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUN_DEPENDS}
)

add_library(${PROJECT_NAME}
  src/DeepOrangeDbwCan.cpp
  src/DeepOrangeStateSupervisor.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/DeepOrangeInterfaceNode.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_SHARE_DIRECTORY}
)

install(DIRECTORY dbc/
  DESTINATION ${CATKIN_INSTALL_PREFIX}/dbc
  FILES_MATCHING PATTERN "*.dbc"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_INSTALL_PREFIX}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_INSTALL_PREFIX}/include
  FILES_MATCHING PATTERN "*.h"
)

if(CATKIN_ENABLE_TESTING)
  set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-readability/todo,-whitespace/todo,-build/header_guard, -whitespace/braces, -runtime/references")
  roslint_cpp()
  roslint_add_test()

  roslaunch_add_file_check(launch/control.launch)
endif()