#!/bin/bash

# ROS-YOLO系统停止脚本

# 停止Docker容器
echo "停止Docker容器..."
docker stop ros_yolo_system 2>/dev/null || true

# 停止本地YOLO检测器
if [ -f /tmp/yolo_detector.pid ]; then
    YOLO_PID=$(cat /tmp/yolo_detector.pid)
    echo "停止YOLO检测器进程 (PID: $YOLO_PID)..."
    kill $YOLO_PID 2>/dev/null || true
    rm -f /tmp/yolo_detector.pid
fi

echo "系统已停止" 