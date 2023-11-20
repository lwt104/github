# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "LfH: 1 messages, 0 services")

set(MSG_I_FLAGS "-ILfH:/home/lwt/jackal_ws/src/hallucination/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(LfH_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg" NAME_WE)
add_custom_target(_LfH_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "LfH" "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(LfH
  "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/LfH
)

### Generating Services

### Generating Module File
_generate_module_cpp(LfH
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/LfH
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(LfH_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(LfH_generate_messages LfH_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg" NAME_WE)
add_dependencies(LfH_generate_messages_cpp _LfH_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LfH_gencpp)
add_dependencies(LfH_gencpp LfH_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LfH_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(LfH
  "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/LfH
)

### Generating Services

### Generating Module File
_generate_module_eus(LfH
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/LfH
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(LfH_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(LfH_generate_messages LfH_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg" NAME_WE)
add_dependencies(LfH_generate_messages_eus _LfH_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LfH_geneus)
add_dependencies(LfH_geneus LfH_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LfH_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(LfH
  "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/LfH
)

### Generating Services

### Generating Module File
_generate_module_lisp(LfH
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/LfH
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(LfH_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(LfH_generate_messages LfH_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg" NAME_WE)
add_dependencies(LfH_generate_messages_lisp _LfH_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LfH_genlisp)
add_dependencies(LfH_genlisp LfH_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LfH_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(LfH
  "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/LfH
)

### Generating Services

### Generating Module File
_generate_module_nodejs(LfH
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/LfH
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(LfH_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(LfH_generate_messages LfH_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg" NAME_WE)
add_dependencies(LfH_generate_messages_nodejs _LfH_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LfH_gennodejs)
add_dependencies(LfH_gennodejs LfH_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LfH_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(LfH
  "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LfH
)

### Generating Services

### Generating Module File
_generate_module_py(LfH
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LfH
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(LfH_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(LfH_generate_messages LfH_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lwt/jackal_ws/src/hallucination/msg/Bspline.msg" NAME_WE)
add_dependencies(LfH_generate_messages_py _LfH_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LfH_genpy)
add_dependencies(LfH_genpy LfH_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LfH_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/LfH)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/LfH
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(LfH_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(LfH_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/LfH)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/LfH
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(LfH_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(LfH_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/LfH)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/LfH
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(LfH_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(LfH_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/LfH)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/LfH
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(LfH_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(LfH_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LfH)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LfH\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LfH
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(LfH_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(LfH_generate_messages_py std_msgs_generate_messages_py)
endif()
