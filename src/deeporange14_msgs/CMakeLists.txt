cmake_minimum_required(VERSION 3.5)
project(deeporange14_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(DIRECTORY msgs FILES
  AutonomyCommandMsg.msg
  AutonomyMeasurementMsg.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
)

install(DIRECTORY msgs
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
