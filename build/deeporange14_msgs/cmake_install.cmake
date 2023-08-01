# Install script for directory: /home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/install")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs/msgs" TYPE FILE FILES
    "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg"
    "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg"
    "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg"
    "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg"
    "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg"
    "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs/cmake" TYPE FILE FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/devel/include/deeporange14_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/devel/share/roseus/ros/deeporange14_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/devel/share/common-lisp/ros/deeporange14_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/devel/share/gennodejs/ros/deeporange14_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/devel/lib/python3/dist-packages/deeporange14_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/devel/lib/python3/dist-packages/deeporange14_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs/cmake" TYPE FILE FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs/cmake" TYPE FILE FILES
    "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgsConfig.cmake"
    "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/build/deeporange14_msgs/catkin_generated/installspace/deeporange14_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs" TYPE FILE FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/deeporange14_msgs" TYPE DIRECTORY FILES "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs")
endif()

