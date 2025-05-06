cmake_minimum_required(VERSION 3.14.4)
project(deeporange14_control)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find all packages needed to build the code
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# TODO: add these as packages as needed, remove if not needed
#  roslaunch
#  message_generation
#  roslint

find_package(can_dbc_parser REQUIRED)

# message packages
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(deeporange14_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# define the target(s) for the build, with dependencies and links
add_library(${PROJECT_NAME}
  src/DeepOrangeStateSupervisor.cpp
  #src/DeepOrangeVelocityController.cpp
  src/DeepOrangeDbwCan.cpp
  #src/DataLogger.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# TODO: not really sure how these are suppoesd to be updated correctly
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  can_dbc_parser
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  novatel_oem7_msgs
  deeporange14_msgs
  can_msgs
  actionlib_msgs
  tf2_msgs
)

# TODO: is this target needed?
add_executable(${PROJECT_NAME}_dbwnode src/DeepOrangeInterfaceNode.cpp)
target_link_libraries(${PROJECT_NAME}_dbwnode ${PROJECT_NAME})

# install the target(s)
# TODO: do these also need to be exported? https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html#installing
install(
  DIRECTORY launch src dbc
  DESTINATION share/${PROJECT_NAME}
)

# TODO: is this needed?
#install(${PROJECT_NAME}_dbwnode DESTINATION lib/${PROJECT_NAME}_dbwnode)

# set up automated testing for launch files and code
# TODO: update this to make it work with ROS2
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()  
#  set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-readability/todo,-whitespace/todo,-build/header_guard, -whitespace/braces, -runtime/references")
#  roslint_cpp()
#  roslint_add_test()

#  roslaunch_add_file_check(launch/control.launch)
#endif()

ament_package()