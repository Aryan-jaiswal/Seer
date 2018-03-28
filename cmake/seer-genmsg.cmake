# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "seer: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iseer:/home/pikachu/catkin_ws/src/Seer/msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(seer_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg" NAME_WE)
add_custom_target(_seer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "seer" "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg" ""
)

get_filename_component(_filename "/home/pikachu/catkin_ws/src/Seer/msg/AprilTagList.msg" NAME_WE)
add_custom_target(_seer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "seer" "/home/pikachu/catkin_ws/src/Seer/msg/AprilTagList.msg" "seer/AprilTag"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(seer
  "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/seer
)
_generate_msg_cpp(seer
  "/home/pikachu/catkin_ws/src/Seer/msg/AprilTagList.msg"
  "${MSG_I_FLAGS}"
  "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/seer
)

### Generating Services

### Generating Module File
_generate_module_cpp(seer
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/seer
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(seer_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(seer_generate_messages seer_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg" NAME_WE)
add_dependencies(seer_generate_messages_cpp _seer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pikachu/catkin_ws/src/Seer/msg/AprilTagList.msg" NAME_WE)
add_dependencies(seer_generate_messages_cpp _seer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(seer_gencpp)
add_dependencies(seer_gencpp seer_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS seer_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(seer
  "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/seer
)
_generate_msg_lisp(seer
  "/home/pikachu/catkin_ws/src/Seer/msg/AprilTagList.msg"
  "${MSG_I_FLAGS}"
  "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/seer
)

### Generating Services

### Generating Module File
_generate_module_lisp(seer
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/seer
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(seer_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(seer_generate_messages seer_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg" NAME_WE)
add_dependencies(seer_generate_messages_lisp _seer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pikachu/catkin_ws/src/Seer/msg/AprilTagList.msg" NAME_WE)
add_dependencies(seer_generate_messages_lisp _seer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(seer_genlisp)
add_dependencies(seer_genlisp seer_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS seer_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(seer
  "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seer
)
_generate_msg_py(seer
  "/home/pikachu/catkin_ws/src/Seer/msg/AprilTagList.msg"
  "${MSG_I_FLAGS}"
  "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seer
)

### Generating Services

### Generating Module File
_generate_module_py(seer
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seer
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(seer_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(seer_generate_messages seer_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pikachu/catkin_ws/src/Seer/msg/AprilTag.msg" NAME_WE)
add_dependencies(seer_generate_messages_py _seer_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pikachu/catkin_ws/src/Seer/msg/AprilTagList.msg" NAME_WE)
add_dependencies(seer_generate_messages_py _seer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(seer_genpy)
add_dependencies(seer_genpy seer_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS seer_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/seer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/seer
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(seer_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/seer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/seer
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(seer_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seer)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seer\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seer
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(seer_generate_messages_py sensor_msgs_generate_messages_py)
endif()
