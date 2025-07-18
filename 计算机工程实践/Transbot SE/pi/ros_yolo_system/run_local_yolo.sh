#!/bin/bash

# 运行本地YOLO检测器
echo "启动本地YOLO检测器..."
cd "$(dirname "$0")/yolo_detector"

# 确保Python环境正确
if command -v python3.11 &> /dev/null; then
    PYTHON_CMD="python3.11"
elif command -v python3 &> /dev/null; then
    PYTHON_CMD="python3"
else
    echo "错误：找不到Python3"
    exit 1
fi

# 运行检测器
$PYTHON_CMD predict_img.py

echo "YOLO检测器已退出" 