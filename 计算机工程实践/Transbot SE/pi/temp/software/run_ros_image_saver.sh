#!/bin/bash

# 将脚本复制到ROS节点目录
cp /root/temp/software/ros_image_saver.py /root/catkin_ws/src/
chmod +x /root/catkin_ws/src/ros_image_saver.py

# 创建共享目录
mkdir -p /root/temp/shared_images

# 开始相机直播并保存图像
echo "启动相机和图像保存节点..."
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash

# 运行usb_cam节点和图像保存节点
roslaunch usb_cam usb_cam-test.launch &
sleep 5  # 等待相机启动

# 运行图像保存节点
echo "启动图像保存节点..."
rosrun rospy /root/catkin_ws/src/ros_image_saver.py
