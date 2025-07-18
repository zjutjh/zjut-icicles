# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robot_localization: 0 messages, 3 services")

set(MSG_I_FLAGS "-Igeographic_msgs:/opt/ros/melodic/share/geographic_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iuuid_msgs:/opt/ros/melodic/share/uuid_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robot_localization_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv" NAME_WE)
add_custom_target(_robot_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_localization" "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv" "geometry_msgs/PoseWithCovarianceStamped:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv" NAME_WE)
add_custom_target(_robot_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_localization" "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv" "geometry_msgs/Quaternion:geographic_msgs/GeoPoint:geographic_msgs/GeoPose"
)

get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/GetState.srv" NAME_WE)
add_custom_target(_robot_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_localization" "/root/software/transbot_library/src/robot_localization/srv/GetState.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization
)
_generate_srv_cpp(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization
)
_generate_srv_cpp(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/GetState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization
)

### Generating Module File
_generate_module_cpp(robot_localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robot_localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robot_localization_generate_messages robot_localization_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_cpp _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_cpp _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/GetState.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_cpp _robot_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_localization_gencpp)
add_dependencies(robot_localization_gencpp robot_localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_localization_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_localization
)
_generate_srv_eus(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_localization
)
_generate_srv_eus(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/GetState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_localization
)

### Generating Module File
_generate_module_eus(robot_localization
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_localization
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robot_localization_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robot_localization_generate_messages robot_localization_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_eus _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_eus _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/GetState.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_eus _robot_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_localization_geneus)
add_dependencies(robot_localization_geneus robot_localization_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_localization_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization
)
_generate_srv_lisp(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization
)
_generate_srv_lisp(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/GetState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization
)

### Generating Module File
_generate_module_lisp(robot_localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robot_localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robot_localization_generate_messages robot_localization_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_lisp _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_lisp _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/GetState.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_lisp _robot_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_localization_genlisp)
add_dependencies(robot_localization_genlisp robot_localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_localization_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_localization
)
_generate_srv_nodejs(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_localization
)
_generate_srv_nodejs(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/GetState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_localization
)

### Generating Module File
_generate_module_nodejs(robot_localization
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_localization
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robot_localization_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robot_localization_generate_messages robot_localization_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_nodejs _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_nodejs _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/GetState.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_nodejs _robot_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_localization_gennodejs)
add_dependencies(robot_localization_gennodejs robot_localization_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_localization_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization
)
_generate_srv_py(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPoint.msg;/opt/ros/melodic/share/geographic_msgs/cmake/../msg/GeoPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization
)
_generate_srv_py(robot_localization
  "/root/software/transbot_library/src/robot_localization/srv/GetState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization
)

### Generating Module File
_generate_module_py(robot_localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robot_localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robot_localization_generate_messages robot_localization_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetPose.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_py _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/SetDatum.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_py _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/robot_localization/srv/GetState.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_py _robot_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_localization_genpy)
add_dependencies(robot_localization_genpy robot_localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_cpp)
  add_dependencies(robot_localization_generate_messages_cpp geographic_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(robot_localization_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robot_localization_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_localization
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_eus)
  add_dependencies(robot_localization_generate_messages_eus geographic_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(robot_localization_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robot_localization_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_lisp)
  add_dependencies(robot_localization_generate_messages_lisp geographic_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(robot_localization_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robot_localization_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_localization
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_nodejs)
  add_dependencies(robot_localization_generate_messages_nodejs geographic_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(robot_localization_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robot_localization_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_py)
  add_dependencies(robot_localization_generate_messages_py geographic_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(robot_localization_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robot_localization_generate_messages_py std_msgs_generate_messages_py)
endif()
