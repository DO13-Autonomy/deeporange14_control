# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pdu_msgs: 6 messages, 0 services")

set(MSG_I_FLAGS "-Ipdu_msgs:/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pdu_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg" NAME_WE)
add_custom_target(_pdu_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pdu_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg" "std_msgs/Header:pdu_msgs/FuseStatus"
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg" NAME_WE)
add_custom_target(_pdu_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pdu_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg" ""
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg" NAME_WE)
add_custom_target(_pdu_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pdu_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg" "pdu_msgs/RelayState"
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg" NAME_WE)
add_custom_target(_pdu_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pdu_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg" "std_msgs/Header:pdu_msgs/RelayStatus"
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg" NAME_WE)
add_custom_target(_pdu_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pdu_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg" ""
)

get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg" NAME_WE)
add_custom_target(_pdu_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pdu_msgs" "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_cpp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_cpp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_cpp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_cpp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_cpp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pdu_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(pdu_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pdu_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pdu_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pdu_msgs_generate_messages pdu_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_cpp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_cpp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_cpp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_cpp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_cpp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_cpp _pdu_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pdu_msgs_gencpp)
add_dependencies(pdu_msgs_gencpp pdu_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pdu_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pdu_msgs
)
_generate_msg_eus(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pdu_msgs
)
_generate_msg_eus(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pdu_msgs
)
_generate_msg_eus(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pdu_msgs
)
_generate_msg_eus(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pdu_msgs
)
_generate_msg_eus(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pdu_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(pdu_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pdu_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pdu_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pdu_msgs_generate_messages pdu_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_eus _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_eus _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_eus _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_eus _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_eus _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_eus _pdu_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pdu_msgs_geneus)
add_dependencies(pdu_msgs_geneus pdu_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pdu_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_lisp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_lisp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_lisp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_lisp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pdu_msgs
)
_generate_msg_lisp(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pdu_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(pdu_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pdu_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pdu_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pdu_msgs_generate_messages pdu_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_lisp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_lisp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_lisp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_lisp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_lisp _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_lisp _pdu_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pdu_msgs_genlisp)
add_dependencies(pdu_msgs_genlisp pdu_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pdu_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pdu_msgs
)
_generate_msg_nodejs(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pdu_msgs
)
_generate_msg_nodejs(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pdu_msgs
)
_generate_msg_nodejs(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pdu_msgs
)
_generate_msg_nodejs(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pdu_msgs
)
_generate_msg_nodejs(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pdu_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pdu_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pdu_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pdu_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pdu_msgs_generate_messages pdu_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_nodejs _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_nodejs _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_nodejs _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_nodejs _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_nodejs _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_nodejs _pdu_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pdu_msgs_gennodejs)
add_dependencies(pdu_msgs_gennodejs pdu_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pdu_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs
)
_generate_msg_py(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs
)
_generate_msg_py(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs
)
_generate_msg_py(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs
)
_generate_msg_py(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs
)
_generate_msg_py(pdu_msgs
  "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(pdu_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pdu_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pdu_msgs_generate_messages pdu_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_py _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/FuseStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_py _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayCommand.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_py _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayReport.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_py _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayState.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_py _pdu_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/administrator/Documents/DEEPORANGE14/deeporange14_control/src/raptor-dbw-ros/pdu_msgs/msg/RelayStatus.msg" NAME_WE)
add_dependencies(pdu_msgs_generate_messages_py _pdu_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pdu_msgs_genpy)
add_dependencies(pdu_msgs_genpy pdu_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pdu_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pdu_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pdu_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pdu_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pdu_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pdu_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pdu_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pdu_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pdu_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pdu_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pdu_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pdu_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pdu_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pdu_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pdu_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
