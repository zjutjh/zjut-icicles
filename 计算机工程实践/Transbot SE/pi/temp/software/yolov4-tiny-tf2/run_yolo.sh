#!/bin/bash
# 激活Python 3.8虚拟环境
source /root/temp/software/yolov4_env/bin/activate

# 进入yolov4-tiny-tf2目录
cd /root/temp/software/yolov4-tiny-tf2

# 运行predict_img.py，并传递所有命令行参数
python predict_img.py "$@"
