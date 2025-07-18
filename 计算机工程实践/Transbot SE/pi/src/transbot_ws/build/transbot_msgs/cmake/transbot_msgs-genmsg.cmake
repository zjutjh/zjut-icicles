# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "transbot_msgs: 14 messages, 6 services")

set(MSG_I_FLAGS "-Itransbot_msgs:/root/transbot_ws/src/transbot_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(transbot_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv" "transbot_msgs/Arm:transbot_msgs/Joint"
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/General.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/General.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg" "transbot_msgs/Joint"
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Position.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/Position.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg" ""
)

get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv" NAME_WE)
add_custom_target(_transbot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "transbot_msgs" "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/General.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)

### Generating Services
_generate_srv_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg;/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_cpp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
)

### Generating Module File
_generate_module_cpp(transbot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(transbot_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(transbot_msgs_generate_messages transbot_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/General.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Position.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_cpp _transbot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transbot_msgs_gencpp)
add_dependencies(transbot_msgs_gencpp transbot_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transbot_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/General.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_msg_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)

### Generating Services
_generate_srv_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg;/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_srv_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_srv_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_srv_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_srv_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)
_generate_srv_eus(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
)

### Generating Module File
_generate_module_eus(transbot_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(transbot_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(transbot_msgs_generate_messages transbot_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/General.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Position.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_eus _transbot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transbot_msgs_geneus)
add_dependencies(transbot_msgs_geneus transbot_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transbot_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/General.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_msg_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)

### Generating Services
_generate_srv_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg;/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)
_generate_srv_lisp(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
)

### Generating Module File
_generate_module_lisp(transbot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(transbot_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(transbot_msgs_generate_messages transbot_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/General.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Position.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_lisp _transbot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transbot_msgs_genlisp)
add_dependencies(transbot_msgs_genlisp transbot_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transbot_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/General.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_msg_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)

### Generating Services
_generate_srv_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg;/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_srv_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_srv_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_srv_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_srv_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)
_generate_srv_nodejs(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
)

### Generating Module File
_generate_module_nodejs(transbot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(transbot_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(transbot_msgs_generate_messages transbot_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/General.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Position.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_nodejs _transbot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transbot_msgs_gennodejs)
add_dependencies(transbot_msgs_gennodejs transbot_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transbot_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/General.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_msg_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)

### Generating Services
_generate_srv_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv"
  "${MSG_I_FLAGS}"
  "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg;/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_srv_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_srv_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_srv_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_srv_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)
_generate_srv_py(transbot_msgs
  "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
)

### Generating Module File
_generate_module_py(transbot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(transbot_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(transbot_msgs_generate_messages transbot_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/General.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/Position.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv" NAME_WE)
add_dependencies(transbot_msgs_generate_messages_py _transbot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(transbot_msgs_genpy)
add_dependencies(transbot_msgs_genpy transbot_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS transbot_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/transbot_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(transbot_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(transbot_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(transbot_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/transbot_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(transbot_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(transbot_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(transbot_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/transbot_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(transbot_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(transbot_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(transbot_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/transbot_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(transbot_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(transbot_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(transbot_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/transbot_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(transbot_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(transbot_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(transbot_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
