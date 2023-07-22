# Install script for directory: /users/vpurohi/deeporange_ws/deeporange14_control/src/deeporange14_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/users/vpurohi/deeporange_ws/deeporange14_control/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs/msgs" TYPE FILE FILES
    "/users/vpurohi/deeporange_ws/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg"
    "/users/vpurohi/deeporange_ws/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg"
    "/users/vpurohi/deeporange_ws/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg"
    "/users/vpurohi/deeporange_ws/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs/cmake" TYPE FILE FILES "/users/vpurohi/deeporange_ws/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/users/vpurohi/deeporange_ws/deeporange14_control/devel/include/deeporange14_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/users/vpurohi/deeporange_ws/deeporange14_control/devel/share/roseus/ros/deeporange14_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/users/vpurohi/deeporange_ws/deeporange14_control/devel/share/common-lisp/ros/deeporange14_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/users/vpurohi/deeporange_ws/deeporange14_control/devel/share/gennodejs/ros/deeporange14_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/users/vpurohi/deeporange_ws/deeporange14_control/devel/lib/python3/dist-packages/deeporange14_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/users/vpurohi/deeporange_ws/deeporange14_control/devel/lib/python3/dist-packages/deeporange14_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/users/vpurohi/deeporange_ws/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs/cmake" TYPE FILE FILES "/users/vpurohi/deeporange_ws/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs/cmake" TYPE FILE FILES
    "/users/vpurohi/deeporange_ws/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgsConfig.cmake"
    "/users/vpurohi/deeporange_ws/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs" TYPE FILE FILES "/users/vpurohi/deeporange_ws/deeporange14_control/src/deeporange14_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs" TYPE DIRECTORY FILES "/users/vpurohi/deeporange_ws/deeporange14_control/src/deeporange14_msgs/msgs")
endif()

