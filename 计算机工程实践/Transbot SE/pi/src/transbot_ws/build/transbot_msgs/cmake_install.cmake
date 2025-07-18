# Install script for directory: /root/transbot_ws/src/transbot_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/transbot_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/transbot_msgs/msg" TYPE FILE FILES
    "/root/transbot_ws/src/transbot_msgs/msg/Adjust.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/SensorState.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/Position.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/General.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/PWMServo.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/Joint.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/Arm.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/LaserAvoid.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/Image_Msg.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/JoyState.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/PatrolWarning.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/PointArray.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/Battery.msg"
    "/root/transbot_ws/src/transbot_msgs/msg/Edition.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/transbot_msgs/srv" TYPE FILE FILES
    "/root/transbot_ws/src/transbot_msgs/srv/Buzzer.srv"
    "/root/transbot_ws/src/transbot_msgs/srv/RGBLight.srv"
    "/root/transbot_ws/src/transbot_msgs/srv/Headlight.srv"
    "/root/transbot_ws/src/transbot_msgs/srv/RobotArm.srv"
    "/root/transbot_ws/src/transbot_msgs/srv/CamDevice.srv"
    "/root/transbot_ws/src/transbot_msgs/srv/Patrol.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/transbot_msgs/cmake" TYPE FILE FILES "/root/transbot_ws/build/transbot_msgs/catkin_generated/installspace/transbot_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/root/transbot_ws/devel/include/transbot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/root/transbot_ws/devel/share/roseus/ros/transbot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/root/transbot_ws/devel/share/common-lisp/ros/transbot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/root/transbot_ws/devel/share/gennodejs/ros/transbot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/root/transbot_ws/devel/lib/python2.7/dist-packages/transbot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/root/transbot_ws/devel/lib/python2.7/dist-packages/transbot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/transbot_ws/build/transbot_msgs/catkin_generated/installspace/transbot_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/transbot_msgs/cmake" TYPE FILE FILES "/root/transbot_ws/build/transbot_msgs/catkin_generated/installspace/transbot_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/transbot_msgs/cmake" TYPE FILE FILES
    "/root/transbot_ws/build/transbot_msgs/catkin_generated/installspace/transbot_msgsConfig.cmake"
    "/root/transbot_ws/build/transbot_msgs/catkin_generated/installspace/transbot_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/transbot_msgs" TYPE FILE FILES "/root/transbot_ws/src/transbot_msgs/package.xml")
endif()

