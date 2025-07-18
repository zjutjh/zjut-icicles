#!/bin/bash

echo "=== Docker文件同步设置脚本 ==="

# 1. 查看当前运行的容器
echo "1. 查看当前运行的容器..."
docker ps

# 2. 创建共享目录
echo "2. 创建共享目录..."
mkdir -p /home/pi/docker_shared/catkin_ws
mkdir -p /home/pi/docker_shared/root_config
mkdir -p /home/pi/docker_shared/ros_packages

# 3. 复制Docker容器中的文件到主机（需要容器正在运行）
echo "3. 请先启动Docker容器，然后执行以下命令复制文件："
echo ""
echo "# 复制ROS工作空间"
echo "docker cp 容器名:/root/catkin_ws/. /home/pi/docker_shared/catkin_ws/"
echo ""
echo "# 复制配置文件"
echo "docker cp 容器名:/root/.config/. /home/pi/docker_shared/root_config/"
echo ""
echo "# 复制其他重要文件（根据需要）"
echo "docker cp 容器名:/opt/ros/melodic/share/. /home/pi/docker_shared/ros_packages/"

# 4. 设置权限
echo "4. 设置目录权限..."
sudo chown -R $USER:$USER /home/pi/docker_shared/
chmod -R 755 /home/pi/docker_shared/

echo ""
echo "=== 操作步骤 ==="
echo "1. 先运行旧的Docker容器: bash run_docker.sh"
echo "2. 在另一个终端执行文件复制命令（见上面的提示）"
echo "3. 退出Docker容器"
echo "4. 重新运行更新后的run_docker.sh脚本"
echo "5. 现在你可以在 /home/pi/docker_shared/ 目录下修改文件"
echo "6. 修改会实时同步到Docker容器内对应位置"

echo ""
echo "=== 同步的目录映射关系 ==="
echo "/home/pi/docker_shared/catkin_ws     <->  /root/catkin_ws"
echo "/home/pi/docker_shared/root_config   <->  /root/.config"
echo "/home/pi/temp                        <->  /root/temp" 