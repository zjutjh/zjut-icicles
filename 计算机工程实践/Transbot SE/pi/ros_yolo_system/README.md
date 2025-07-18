# ROS-YOLO系统

这是一个基于Docker和本地YOLO的目标检测系统，通过共享文件系统实现Docker内ROS与本地YOLO模型的通信。

## 系统架构

系统由两个主要部分组成：

1. **Docker内的ROS节点** (`view_detect` 包)：
   - `image_saver.py`：订阅摄像头图像并保存到共享目录
   - `result_publisher.py`：读取检测结果并发布到ROS话题

2. **本地YOLO检测器**：
   - `reg_connect.py`：监控共享目录中的图像并调用YOLO进行检测
   - `predict_img.py`：简化版脚本，调用原始YOLO脚本处理图像

## 工作流程

1. Docker容器内的ROS节点将摄像头图像保存到共享目录
2. 本地YOLO检测器监控共享目录，检测到新图像时调用YOLO进行处理
3. 检测结果保存到共享目录中
4. Docker容器内的ROS节点读取结果并发布到ROS话题

## 目录结构

```
ros_yolo_system/
├── view_detect/            # Docker内的ROS包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/
│   │   └── YoloResult.msg  # 自定义消息类型
│   ├── src/
│   │   ├── image_saver.py  # 保存图像到共享目录
│   │   └── result_publisher.py  # 发布检测结果
│   └── launch/
│       └── view_detect.launch  # 启动文件
├── yolo_detector/
│   ├── reg_connect.py      # 监控图像目录
│   ├── predict_img.py      # 简化版检测脚本
│   └── run_reg_connect.sh  # 启动脚本
├── shared/                 # 共享目录
│   ├── images/             # 图像共享
│   └── results/            # 结果共享
├── start_system.sh         # 启动脚本
└── stop_system.sh          # 停止脚本
```

## 安装依赖

本地系统需要安装以下依赖：

```bash
pip3 install watchdog
```

## 使用方法

1. 确保YOLO模型已正确安装在 `/home/pi/temp/software/yolov4-tiny-tf2` 目录下

2. 确保Docker已安装并能够访问 `yahboom/ros-melodic:latest` 镜像

3. 启动系统：

```bash
cd /home/pi/ros_yolo_system
chmod +x start_system.sh stop_system.sh
./start_system.sh
```

4. 停止系统：

```bash
./stop_system.sh
```

## 配置

可以在启动脚本中修改以下配置：

- Docker镜像名称
- YOLO模型路径
- 共享目录路径
- 检测阈值

## 检测结果

检测结果以JSON格式保存在共享目录中，并通过ROS话题 `yolo_detection` 发布，使用自定义消息类型 `YoloResult`。

## 故障排除

如果系统无法正常工作，请检查：

1. 共享目录权限是否正确
2. Docker容器是否能够访问摄像头
3. YOLO模型文件是否存在
4. 日志输出是否有错误信息 