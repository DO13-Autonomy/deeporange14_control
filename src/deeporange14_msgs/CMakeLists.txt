cmake_minimum_required(VERSION 3.14.4)
cmake_policy(SET CMP0057 NEW)
project(deeporange14_msgs)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/AuStatus.msg"
  "msg/CmdVelCntrl.msg"
  "msg/MissionStatus.msg"  # TODO: is this used?
  "msg/Mobility.msg"
  "msg/PIDComponents.msg"
  "msg/RaptorState.msg"
  "msg/TorqueCmdStamped.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()