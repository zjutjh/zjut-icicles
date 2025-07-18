#!/bin/bash

# 启动reg_connect.py的脚本

# 设置工作目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 设置路径
SHARED_DIR="/home/pi/ros_yolo_system/shared"
YOLO_DIR="/home/pi/temp/software/yolov4-tiny-tf2"

# 确保共享目录存在
mkdir -p "$SHARED_DIR/images"
mkdir -p "$SHARED_DIR/results"

# 检查依赖
if ! pip3 list | grep -q watchdog; then
    echo "安装watchdog依赖..."
    pip3 install watchdog
fi

if ! pip3 list | grep -q tensorflow; then
    echo "警告: 未检测到TensorFlow，YOLO检测可能无法正常工作"
fi

if ! pip3 list | grep -q pillow; then
    echo "安装pillow依赖..."
    pip3 install pillow
fi

# 检查YOLO模型文件是否存在
if [ ! -f "$YOLO_DIR/logs/last1.weights.h5" ]; then
    echo "错误: YOLO模型文件不存在: $YOLO_DIR/logs/last1.weights.h5"
    exit 1
fi

if [ ! -f "$YOLO_DIR/model_data/CampusBot.txt" ]; then
    echo "错误: YOLO类别文件不存在: $YOLO_DIR/model_data/CampusBot.txt"
    exit 1
fi

# 启动监控程序
echo "启动YOLO图像监控程序..."
python3 reg_connect.py \
    --images_dir "$SHARED_DIR/images" \
    --results_dir "$SHARED_DIR/results" \
    --yolo_path "$YOLO_DIR" 