# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gnss_pos: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ignss_pos:/home/leiyubiao/ll/src/message/gnss_pos/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gnss_pos_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg" NAME_WE)
add_custom_target(_gnss_pos_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gnss_pos" "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(gnss_pos
  "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gnss_pos
)

### Generating Services

### Generating Module File
_generate_module_cpp(gnss_pos
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gnss_pos
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gnss_pos_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gnss_pos_generate_messages gnss_pos_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg" NAME_WE)
add_dependencies(gnss_pos_generate_messages_cpp _gnss_pos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_pos_gencpp)
add_dependencies(gnss_pos_gencpp gnss_pos_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_pos_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(gnss_pos
  "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gnss_pos
)

### Generating Services

### Generating Module File
_generate_module_eus(gnss_pos
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gnss_pos
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gnss_pos_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gnss_pos_generate_messages gnss_pos_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg" NAME_WE)
add_dependencies(gnss_pos_generate_messages_eus _gnss_pos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_pos_geneus)
add_dependencies(gnss_pos_geneus gnss_pos_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_pos_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(gnss_pos
  "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gnss_pos
)

### Generating Services

### Generating Module File
_generate_module_lisp(gnss_pos
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gnss_pos
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gnss_pos_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gnss_pos_generate_messages gnss_pos_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg" NAME_WE)
add_dependencies(gnss_pos_generate_messages_lisp _gnss_pos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_pos_genlisp)
add_dependencies(gnss_pos_genlisp gnss_pos_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_pos_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(gnss_pos
  "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gnss_pos
)

### Generating Services

### Generating Module File
_generate_module_nodejs(gnss_pos
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gnss_pos
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gnss_pos_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gnss_pos_generate_messages gnss_pos_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg" NAME_WE)
add_dependencies(gnss_pos_generate_messages_nodejs _gnss_pos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_pos_gennodejs)
add_dependencies(gnss_pos_gennodejs gnss_pos_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_pos_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(gnss_pos
  "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss_pos
)

### Generating Services

### Generating Module File
_generate_module_py(gnss_pos
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss_pos
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gnss_pos_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gnss_pos_generate_messages gnss_pos_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leiyubiao/ll/src/message/gnss_pos/msg/pos_xy.msg" NAME_WE)
add_dependencies(gnss_pos_generate_messages_py _gnss_pos_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gnss_pos_genpy)
add_dependencies(gnss_pos_genpy gnss_pos_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gnss_pos_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gnss_pos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gnss_pos
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gnss_pos_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gnss_pos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gnss_pos
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gnss_pos_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gnss_pos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gnss_pos
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gnss_pos_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gnss_pos)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gnss_pos
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gnss_pos_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss_pos)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss_pos\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gnss_pos
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gnss_pos_generate_messages_py std_msgs_generate_messages_py)
endif()
