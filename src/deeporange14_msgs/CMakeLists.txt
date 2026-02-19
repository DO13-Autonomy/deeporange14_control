cmake_minimum_required(VERSION 3.5)
project(deeporange14_msgs)

set(PKG_DEPENDS
  std_msgs
)

set(BUILD_DEPENDS
  message_generation
  ${PKG_DEPENDS}
)

set(RUN_DEPENDS
  message_runtime
  ${PKG_DEPENDS}
)

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDS})

add_message_files(DIRECTORY msgs
  FILES
    AutonomyCommandMsg.msg
    AutonomyMeasurementMsg.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS ${RUN_DEPENDS})

install(DIRECTORY msgs
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
