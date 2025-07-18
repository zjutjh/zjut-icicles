# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "yahboomcar_msgs: 5 messages, 0 services")

set(MSG_I_FLAGS "-Iyahboomcar_msgs:/root/catkin_ws/src/yahboomcar_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(yahboomcar_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg" NAME_WE)
add_custom_target(_yahboomcar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "yahboomcar_msgs" "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg" NAME_WE)
add_custom_target(_yahboomcar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "yahboomcar_msgs" "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg" NAME_WE)
add_custom_target(_yahboomcar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "yahboomcar_msgs" "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg" NAME_WE)
add_custom_target(_yahboomcar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "yahboomcar_msgs" "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg" "yahboomcar_msgs/Target"
)

get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg" NAME_WE)
add_custom_target(_yahboomcar_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "yahboomcar_msgs" "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_cpp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_cpp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_cpp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_cpp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yahboomcar_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(yahboomcar_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yahboomcar_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(yahboomcar_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(yahboomcar_msgs_generate_messages yahboomcar_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_cpp _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_cpp _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_cpp _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_cpp _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_cpp _yahboomcar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yahboomcar_msgs_gencpp)
add_dependencies(yahboomcar_msgs_gencpp yahboomcar_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yahboomcar_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_eus(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_eus(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_eus(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_eus(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yahboomcar_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(yahboomcar_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yahboomcar_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(yahboomcar_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(yahboomcar_msgs_generate_messages yahboomcar_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_eus _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_eus _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_eus _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_eus _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_eus _yahboomcar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yahboomcar_msgs_geneus)
add_dependencies(yahboomcar_msgs_geneus yahboomcar_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yahboomcar_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_lisp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_lisp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_lisp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_lisp(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yahboomcar_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(yahboomcar_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yahboomcar_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(yahboomcar_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(yahboomcar_msgs_generate_messages yahboomcar_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_lisp _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_lisp _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_lisp _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_lisp _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_lisp _yahboomcar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yahboomcar_msgs_genlisp)
add_dependencies(yahboomcar_msgs_genlisp yahboomcar_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yahboomcar_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_nodejs(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_nodejs(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_nodejs(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_nodejs(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yahboomcar_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(yahboomcar_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yahboomcar_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(yahboomcar_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(yahboomcar_msgs_generate_messages yahboomcar_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_nodejs _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_nodejs _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_nodejs _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_nodejs _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_nodejs _yahboomcar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yahboomcar_msgs_gennodejs)
add_dependencies(yahboomcar_msgs_gennodejs yahboomcar_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yahboomcar_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_py(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_py(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_py(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg"
  "${MSG_I_FLAGS}"
  "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yahboomcar_msgs
)
_generate_msg_py(yahboomcar_msgs
  "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yahboomcar_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(yahboomcar_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yahboomcar_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(yahboomcar_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(yahboomcar_msgs_generate_messages yahboomcar_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Position.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_py _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_py _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/Target.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_py _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/TargetArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_py _yahboomcar_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/yahboomcar_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(yahboomcar_msgs_generate_messages_py _yahboomcar_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(yahboomcar_msgs_genpy)
add_dependencies(yahboomcar_msgs_genpy yahboomcar_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS yahboomcar_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yahboomcar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/yahboomcar_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(yahboomcar_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(yahboomcar_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(yahboomcar_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yahboomcar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/yahboomcar_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(yahboomcar_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(yahboomcar_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(yahboomcar_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yahboomcar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/yahboomcar_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(yahboomcar_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(yahboomcar_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(yahboomcar_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yahboomcar_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/yahboomcar_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(yahboomcar_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(yahboomcar_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(yahboomcar_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yahboomcar_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yahboomcar_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/yahboomcar_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(yahboomcar_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(yahboomcar_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(yahboomcar_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
