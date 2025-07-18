#!/bin/bash

# ROS-YOLO系统启动脚本

# 设置工作目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 设置路径
SHARED_DIR="$SCRIPT_DIR/shared"
YOLO_DIR="/home/pi/temp/software/yolov4-tiny-tf2"

# 确保共享目录存在
mkdir -p "$SHARED_DIR/images"
mkdir -p "$SHARED_DIR/results"

# 清空共享目录中的旧文件
echo "清理共享目录..."
rm -f "$SHARED_DIR/images"/*
rm -f "$SHARED_DIR/results"/*

# 启动本地YOLO检测器
echo "启动本地YOLO检测器..."
cd "$SCRIPT_DIR/yolo_detector"
./run_reg_connect.sh &
YOLO_PID=$!
echo "YOLO检测器进程ID: $YOLO_PID"
echo $YOLO_PID > /tmp/yolo_detector.pid

# 等待YOLO检测器启动
sleep 2

# 启动Docker容器中的ROS节点
echo "启动Docker容器中的ROS节点..."
docker run -it --rm \
    --name ros_yolo_system \
    -v "$SHARED_DIR:/shared" \
    -v "/dev/video0:/dev/video0" \
    --network host \
    --privileged \
    yahboom/ros-melodic:latest \
    roslaunch view_detect view_detect.launch

# 注意：当Docker容器停止时，脚本会继续执行
echo "Docker容器已停止"

# 停止本地YOLO检测器
if [ -f /tmp/yolo_detector.pid ]; then
    YOLO_PID=$(cat /tmp/yolo_detector.pid)
    echo "停止YOLO检测器进程 (PID: $YOLO_PID)..."
    kill $YOLO_PID 2>/dev/null || true
    rm -f /tmp/yolo_detector.pid
fi

echo "系统已停止" 