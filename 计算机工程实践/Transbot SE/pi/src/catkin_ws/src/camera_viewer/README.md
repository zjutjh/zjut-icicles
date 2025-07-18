# Camera Viewer ROS包

此ROS包用于：
1. 订阅USB相机的原始图像话题
2. 将图像转换为OpenCV格式
3. 使用OpenCV显示图像

## 依赖项
- ROS Melodic
- OpenCV
- usb_cam包
- cv_bridge
- image_transport

## 在Docker中编译和运行

### 编译
1. 进入Docker容器
```bash
# 使用已有的Docker容器或运行脚本启动新容器
docker exec -it <CONTAINER_ID> /bin/bash
# 或
./run_docker_pi1.sh
```

2. 编译包
```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

### 运行
运行启动文件：
```bash
roslaunch camera_viewer camera_viewer.launch
```

这将启动USB相机和查看器节点。

## 注意
确保Docker有适当的显示设置（如脚本run_docker_pi1.sh中设置的），以便能够显示OpenCV窗口。
