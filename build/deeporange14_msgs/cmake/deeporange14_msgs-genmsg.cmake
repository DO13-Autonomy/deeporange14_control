# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "deeporange14_msgs: 6 messages, 0 services")

set(MSG_I_FLAGS "-Ideeporange14_msgs:/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(deeporange14_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg" NAME_WE)
add_custom_target(_deeporange14_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deeporange14_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg" NAME_WE)
add_custom_target(_deeporange14_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deeporange14_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg" NAME_WE)
add_custom_target(_deeporange14_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deeporange14_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg" NAME_WE)
add_custom_target(_deeporange14_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deeporange14_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg" NAME_WE)
add_custom_target(_deeporange14_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deeporange14_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg" NAME_WE)
add_custom_target(_deeporange14_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "deeporange14_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_cpp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_cpp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_cpp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_cpp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_cpp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deeporange14_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(deeporange14_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deeporange14_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(deeporange14_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(deeporange14_msgs_generate_messages deeporange14_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_cpp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_cpp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_cpp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_cpp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_cpp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_cpp _deeporange14_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deeporange14_msgs_gencpp)
add_dependencies(deeporange14_msgs_gencpp deeporange14_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deeporange14_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_eus(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_eus(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_eus(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_eus(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_eus(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deeporange14_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(deeporange14_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deeporange14_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(deeporange14_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(deeporange14_msgs_generate_messages deeporange14_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_eus _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_eus _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_eus _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_eus _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_eus _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_eus _deeporange14_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deeporange14_msgs_geneus)
add_dependencies(deeporange14_msgs_geneus deeporange14_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deeporange14_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_lisp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_lisp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_lisp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_lisp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_lisp(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deeporange14_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(deeporange14_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deeporange14_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(deeporange14_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(deeporange14_msgs_generate_messages deeporange14_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_lisp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_lisp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_lisp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_lisp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_lisp _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_lisp _deeporange14_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deeporange14_msgs_genlisp)
add_dependencies(deeporange14_msgs_genlisp deeporange14_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deeporange14_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_nodejs(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_nodejs(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_nodejs(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_nodejs(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_nodejs(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deeporange14_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(deeporange14_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deeporange14_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(deeporange14_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(deeporange14_msgs_generate_messages deeporange14_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_nodejs _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_nodejs _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_nodejs _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_nodejs _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_nodejs _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_nodejs _deeporange14_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deeporange14_msgs_gennodejs)
add_dependencies(deeporange14_msgs_gennodejs deeporange14_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deeporange14_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_py(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_py(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_py(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_py(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs
)
_generate_msg_py(deeporange14_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(deeporange14_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(deeporange14_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(deeporange14_msgs_generate_messages deeporange14_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MobilityMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_py _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/RaptorStateMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_py _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/MissionStatus.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_py _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/TorqueCmdStamped.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_py _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/AuStatusMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_py _deeporange14_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/deeporange14_msgs/msgs/PIDComponentsMsg.msg" NAME_WE)
add_dependencies(deeporange14_msgs_generate_messages_py _deeporange14_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(deeporange14_msgs_genpy)
add_dependencies(deeporange14_msgs_genpy deeporange14_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS deeporange14_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deeporange14_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/deeporange14_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(deeporange14_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(deeporange14_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deeporange14_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/deeporange14_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(deeporange14_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(deeporange14_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deeporange14_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/deeporange14_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(deeporange14_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(deeporange14_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deeporange14_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/deeporange14_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(deeporange14_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(deeporange14_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/deeporange14_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(deeporange14_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(deeporange14_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
