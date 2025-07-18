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

