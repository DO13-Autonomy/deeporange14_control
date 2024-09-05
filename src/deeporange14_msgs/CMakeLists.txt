cmake_minimum_required(VERSION 2.8.3)
project(deeporange14_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  roscpp
)

add_message_files(DIRECTORY msgs FILES
  MobilityMsg.msg
  RaptorStateMsg.msg
  MissionStatus.msg
  TorqueCmdStamped.msg
  AuStatusMsg.msg
  PIDComponentsMsg.msg
  CmdVelCntrl.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  message_runtime
  std_msgs
  geometry_msgs
)

install(DIRECTORY msgs
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
