#!/bin/bash

# Wait for the Docker service to start
while true; do
    if systemctl is-active --quiet docker; then
        echo "Docker service has been started"
        break
    else
        echo "The Docker service has not started, waiting..."
        sleep 1
    fi
done

# 获取 SSH 客户端的 IP（Windows 机器的 IP）
WINDOWS_IP=$(echo $SSH_CLIENT | awk '{print $1}')
echo "Windows IP detected: $WINDOWS_IP"

# 设置 DISPLAY 指向 Windows 的 X11 服务器
export DISPLAY=$WINDOWS_IP:0.0

# 允许 X11 转发
xhost +$WINDOWS_IP

xhost +
echo "正在启动Docker容器..."
docker run -it \
--name ros_yolo_container \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \
-v /home/pi/temp:/root/temp \
-v /home/pi/src/catkin_ws:/root/catkin_ws \
-v /home/pi/src/transbot_ws:/root/transbot_ws \
-v /home/pi/src/Transbot:/root/Transbot \
-v /home/pi/ros_yolo_system/shared:/shared \
-v /home/pi/ros_yolo_system/view_detect:/root/catkin_ws/src/view_detect \
--device=/dev/ttyAMA0 \
--device=/dev/video0 \
--device=/dev/input \
--device=/dev/video1 \
yahboomtechnology/ros-melodic:Transbot_Se_V1.4 /bin/bash -c "
echo '正在编译ROS功能包...' &&
source /opt/ros/melodic/setup.bash &&
cd /root/catkin_ws &&
catkin_make &&
source /root/catkin_ws/devel/setup.bash &&
echo '编译完成，进入bash环境' &&
/bin/bash
"

# 如果容器正常退出，提交新镜像
if [ $? -eq 0 ]; then
    echo "正在保存容器为新的Docker镜像..."
    docker commit ros_yolo_container yahboomtechnology/ros-melodic:Transbot_Yolo_V1.0
    echo "新镜像已保存为 yahboomtechnology/ros-melodic:Transbot_Yolo_V1.0"
    # 清理容器
    docker rm ros_yolo_container
else
    echo "容器异常退出，未保存新镜像"
fi

# 清理
xhost -$WINDOWS_IP