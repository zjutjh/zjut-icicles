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
docker run -it \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \
-v /home/pi/temp:/root/temp \
-v /home/pi/src/catkin_ws:/root/catkin_ws \
-v /home/pi/src/transbot_ws:/root/transbot_ws \
-v /home/pi/src/Transbot:/root/Transbot \
--device=/dev/ttyAMA0 \
--device=/dev/video0 \
--device=/dev/input \
--device=/dev/video1 \
yahboomtechnology/ros-melodic:Transbot_Se_V1.4 /bin/bash 

# 清理
xhost -$WINDOWS_IP
