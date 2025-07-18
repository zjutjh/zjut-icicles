# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "opencv_apps: 31 messages, 1 services")

set(MSG_I_FLAGS "-Iopencv_apps:/root/software/transbot_library/src/opencv_apps/msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(opencv_apps_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg" "opencv_apps/Rect:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg" "opencv_apps/Point2D"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg" "opencv_apps/Point2D:opencv_apps/RotatedRect:opencv_apps/Size"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg" ""
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg" "opencv_apps/Size:opencv_apps/Point2D:opencv_apps/RotatedRect:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg" "opencv_apps/Point2D:opencv_apps/Line:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg" "opencv_apps/Point2D:opencv_apps/Contour:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg" "opencv_apps/Point2D"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg" ""
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg" "opencv_apps/Point2D:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg" "opencv_apps/Point2D"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg" "opencv_apps/Point2D:opencv_apps/Circle"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg" "opencv_apps/Point2D:opencv_apps/Circle:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg" "opencv_apps/Point2D:opencv_apps/Flow:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg" "opencv_apps/Rect:opencv_apps/Face:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg" "opencv_apps/Point2D:opencv_apps/Contour"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg" "opencv_apps/Size:opencv_apps/Point2D:opencv_apps/RotatedRect:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg" "opencv_apps/Rect:opencv_apps/Face"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg" "opencv_apps/Point2D:opencv_apps/Moment:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Line.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Line.msg" "opencv_apps/Point2D"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg" "opencv_apps/Point2D:opencv_apps/Size"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg" "opencv_apps/Point2D:opencv_apps/Moment"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv" "sensor_msgs/Image:opencv_apps/Rect:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg" "opencv_apps/Point2D:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Size.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Size.msg" ""
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg" "opencv_apps/Point2D:opencv_apps/Flow"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg" "opencv_apps/Point2D"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg" "opencv_apps/Rect"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg" "opencv_apps/Point2D:opencv_apps/Flow:std_msgs/Header"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg" "opencv_apps/Point2D"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Face.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/Face.msg" "opencv_apps/Rect"
)

get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg" NAME_WE)
add_custom_target(_opencv_apps_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "opencv_apps" "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg" "opencv_apps/Point2D:opencv_apps/Line"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)
_generate_msg_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)

### Generating Services
_generate_srv_cpp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
)

### Generating Module File
_generate_module_cpp(opencv_apps
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(opencv_apps_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(opencv_apps_generate_messages opencv_apps_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Line.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Size.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Face.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_cpp _opencv_apps_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(opencv_apps_gencpp)
add_dependencies(opencv_apps_gencpp opencv_apps_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS opencv_apps_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)
_generate_msg_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)

### Generating Services
_generate_srv_eus(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
)

### Generating Module File
_generate_module_eus(opencv_apps
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(opencv_apps_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(opencv_apps_generate_messages opencv_apps_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Line.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Size.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Face.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_eus _opencv_apps_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(opencv_apps_geneus)
add_dependencies(opencv_apps_geneus opencv_apps_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS opencv_apps_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)
_generate_msg_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)

### Generating Services
_generate_srv_lisp(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
)

### Generating Module File
_generate_module_lisp(opencv_apps
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(opencv_apps_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(opencv_apps_generate_messages opencv_apps_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Line.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Size.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Face.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_lisp _opencv_apps_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(opencv_apps_genlisp)
add_dependencies(opencv_apps_genlisp opencv_apps_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS opencv_apps_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)
_generate_msg_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)

### Generating Services
_generate_srv_nodejs(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
)

### Generating Module File
_generate_module_nodejs(opencv_apps
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(opencv_apps_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(opencv_apps_generate_messages opencv_apps_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Line.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Size.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Face.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_nodejs _opencv_apps_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(opencv_apps_gennodejs)
add_dependencies(opencv_apps_gennodejs opencv_apps_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS opencv_apps_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Circle.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Contour.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg;/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/root/software/transbot_library/src/opencv_apps/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Size.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Flow.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)
_generate_msg_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg"
  "${MSG_I_FLAGS}"
  "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg;/root/software/transbot_library/src/opencv_apps/msg/Line.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)

### Generating Services
_generate_srv_py(opencv_apps
  "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/root/software/transbot_library/src/opencv_apps/msg/Rect.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
)

### Generating Module File
_generate_module_py(opencv_apps
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(opencv_apps_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(opencv_apps_generate_messages opencv_apps_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Contour.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2D.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Flow.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Rect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/CircleArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/ContourArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRectArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FaceArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Line.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RotatedRect.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/MomentArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/srv/FaceRecognitionTrain.srv" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Point2DStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Size.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Circle.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/RectArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/FlowArrayStamped.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Moment.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/Face.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/software/transbot_library/src/opencv_apps/msg/LineArray.msg" NAME_WE)
add_dependencies(opencv_apps_generate_messages_py _opencv_apps_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(opencv_apps_genpy)
add_dependencies(opencv_apps_genpy opencv_apps_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS opencv_apps_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/opencv_apps
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(opencv_apps_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(opencv_apps_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/opencv_apps
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(opencv_apps_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(opencv_apps_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/opencv_apps
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(opencv_apps_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(opencv_apps_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/opencv_apps
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(opencv_apps_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(opencv_apps_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/opencv_apps
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(opencv_apps_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(opencv_apps_generate_messages_py std_msgs_generate_messages_py)
endif()
