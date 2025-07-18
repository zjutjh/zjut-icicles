#!/bin/bash

# 为脚本添加执行权限
chmod +x /home/pi/temp/software/ros_image_saver.py
chmod +x /home/pi/temp/software/yolov4-tiny-tf2/yolo_image_watcher.py

# 创建共享目录
mkdir -p /home/pi/temp/shared_images
mkdir -p /home/pi/temp/output_images

# 启动本地的YOLO观察者
echo "启动本地YOLO图像处理程序..."
gnome-terminal --title="YOLO观察者" -- python3 /home/pi/temp/software/yolov4-tiny-tf2/yolo_image_watcher.py &

# 等待YOLO程序启动
sleep 2

# 启动Docker并运行ROS节点
echo "启动Docker并运行ROS图像保存节点..."
cd /home/pi && ./run_docker.sh
