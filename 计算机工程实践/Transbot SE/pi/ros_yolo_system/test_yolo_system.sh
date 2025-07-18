#!/bin/bash

# 测试ROS与YOLO系统的通信

# 启动新版本的Docker容器
echo "正在启动Docker容器..."
docker run -it --rm \
--net=host \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \
-v /home/pi/ros_yolo_system/shared:/shared \
-v /home/pi/ros_yolo_system/view_detect:/root/catkin_ws/src/view_detect \
-v /home/pi/ros_yolo_system/yolo_detector:/root/yolo_detector \
--device=/dev/ttyAMA0 \
--device=/dev/video0 \
--device=/dev/input \
--device=/dev/video1 \
yahboomtechnology/ros-melodic:Transbot_Yolo_V1.0 /bin/bash -c "
source /opt/ros/melodic/setup.bash &&
source /root/catkin_ws/devel/setup.bash &&
echo '启动ROS核心...' &&
roscore &
sleep 5 &&
echo '启动图像发布节点...' &&
rosrun view_detect image_publisher.py &
sleep 2 &&
echo '启动结果发布节点...' &&
rosrun view_detect results_publisher.py &
sleep 2 &&
echo '所有ROS节点已启动' &&
echo '您可以在另一个终端运行本地YOLO检测器' &&
/bin/bash
" 