# Transbot ROS 工作区

本工作区包含了 Transbot 机器人的各种功能包，涵盖了机器人控制、视觉感知、AI识别、运动控制等完整功能。

## 功能包介绍

### 核心系统包

#### 1. **transbot_bringup**
- **功能**: 系统启动与集成
- **描述**: 负责机器人系统的统一启动，包含底层驱动、IMU校准、传感器融合、里程计等核心功能
- **依赖**: 支持 SLAM (gmapping)、导航 (move_base, amcl)、激光雷达等
- **用途**: 机器人基础功能的一键启动入口

#### 2. **transbot_msgs**
- **功能**: 自定义消息与服务定义
- **描述**: 定义机器人内部通信的消息类型，如机械臂控制、传感器数据、位置信息等
- **包含**: Arm, Joint, Position, SensorState, JoyState 等消息类型
- **用途**: 各功能包间的数据接口层

### 控制与交互包

#### 3. **transbot_ctrl**
- **功能**: 多种控制方式
- **描述**: 提供手柄、键盘等多种控制接口
- **主要脚本**:
  - `transbot_joy.py`: 手柄控制
  - `transbot_keyboard.py`: 键盘控制
- **用途**: 用户交互与远程控制

### 视觉感知包

#### 4. **transbot_track**
- **功能**: 视觉跟踪与控制
- **描述**: 基于视觉的目标跟踪、PID控制、机械臂联动
- **主要特性**:
  - HSV颜色跟踪
  - PID运动控制
  - 机械臂协调控制
- **用途**: 目标跟踪、视觉伺服

#### 5. **transbot_facetracker**
- **功能**: 人脸检测与跟踪
- **描述**: 基于 Haar 特征的人脸识别和跟踪
- **主要文件**:
  - `face_follow.py`: 人脸跟踪主程序
  - `haarcascade_frontalface_default.xml`: 人脸检测模型
- **用途**: 人脸识别、社交机器人应用

#### 6. **transbot_visual**
- **功能**: 通用视觉处理
- **描述**: 视觉相关的通用算法和工具
- **用途**: 图像处理、视觉算法支持

#### 7. **transbot_linefollow**
- **功能**: 循迹功能
- **描述**: 基于视觉的线条跟踪和路径规划
- **用途**: 自动导航、路径跟踪

### AI 与机器学习包

#### 8. **transbot_mediapipe**
- **功能**: MediaPipe 集成
- **描述**: 集成 Google MediaPipe 进行手势识别、人体姿态估计等
- **用途**: 手势控制、人体动作识别

#### 9. **arm_mediapipe**
- **功能**: 机械臂与 MediaPipe 联动
- **描述**: 结合 MediaPipe 手势识别控制机械臂运动
- **用途**: 手势控制机械臂、人机交互

### 专用功能包

#### 10. **transbot_mono**
- **功能**: 单目视觉处理
- **描述**: 单目摄像头的视觉算法和应用
- **用途**: 深度估计、单目SLAM

#### 11. **transbot_mulity**
- **功能**: 多任务/多模态处理
- **描述**: 多传感器融合或多任务并行处理
- **用途**: 复杂场景下的多功能集成

#### 12. **transbot_program**
- **功能**: 编程示例与工具
- **描述**: 机器人编程的示例代码和开发工具
- **用途**: 学习参考、快速开发

#### 13. **transbot_se_moveit_config**
- **功能**: MoveIt! 配置
- **描述**: 机械臂运动规划的 MoveIt! 配置文件
- **用途**: 机械臂路径规划、运动控制

### 自定义开发包

#### 14. **transbot_composite_app**
- **功能**: 复合主程序
- **描述**: 集成各功能包的主程序入口，用于复杂流程开发
- **用途**: 自定义复合功能、系统集成

## 使用说明

### 1. 环境准备
```bash
cd ~/src/transbot_ws
catkin_make
source devel/setup.bash
```

### 2. 基础启动
```bash
roslaunch transbot_bringup bringup.launch
```

### 3. 功能模块启动
- 视觉跟踪: `roslaunch transbot_track xxx.launch`
- 人脸跟踪: `rosrun transbot_facetracker face_follow.py`
- 手柄控制: `roslaunch transbot_ctrl transbot_joy.launch`

## 开发建议

1. **新功能开发**: 建议在 `transbot_composite_app` 包下进行
2. **消息扩展**: 在 `transbot_msgs` 包下添加新的消息类型
3. **集成测试**: 使用 `transbot_bringup` 进行系统级测试

---

**维护者**: yahboom
**ROS版本**: Melodic
**Python版本**: 2.7 (兼容部分 Python 3) 