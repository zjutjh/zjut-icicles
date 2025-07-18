# YOLO与ROS通信系统

这是一个简化的YOLO与ROS通信系统，通过共享文件系统实现Docker内ROS与本地YOLO模型的通信。

## 系统架构

系统由两个主要部分组成：

1. **Docker内的ROS节点**：
   - 订阅摄像头图像并保存到共享目录
   - 读取检测结果并发布到ROS话题

2. **本地YOLO检测器**：
   - `reg_connect.py`：监控共享目录中的图像并调用YOLO进行检测
   - 检测结果保存到共享目录供ROS节点读取

## 使用方法

### 1. 启动Docker容器

```bash
cd /home/pi
./run_docker_pi1.sh
```

### 2. 在Docker容器中启动ROS节点

```bash
cd /root/catkin_ws
source devel/setup.bash
roscore &
roslaunch view_detect view_detect.launch
```

### 3. 在本地启动YOLO检测器

```bash
cd /home/pi/ros_yolo_system/yolo_detector
./run_reg_connect.sh
```

## 文件说明

- `reg_connect.py`：主程序，监控共享目录中的图像并调用YOLO进行检测
- `run_reg_connect.sh`：启动脚本，用于启动reg_connect.py
- `predict_img_modified.py`：修改版的YOLO检测脚本，支持单个图像路径作为参数

## 工作流程

1. Docker内的ROS节点订阅摄像头图像并保存到共享目录
2. `reg_connect.py`监控共享目录，当有新图像时调用YOLO进行检测
3. 检测结果保存到共享目录
4. Docker内的ROS节点读取检测结果并发布到ROS话题

## 依赖

- Python 3
- watchdog
- TensorFlow
- PIL (Pillow)
- ROS (在Docker容器中) 