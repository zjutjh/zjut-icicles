# 智能取外卖系统

这是一个基于Web的智能取外卖系统，集成了语音识别、语音合成和大模型API功能。系统可以处理外卖订单信息，支持二维码扫描，并提供智能对话功能。

## 功能特点

- **订单管理**：显示待取订单和已取订单，支持取餐确认功能
- **二维码扫描**：支持简化的二维码数据处理（A/B/C代表不同外卖信息）
- **语音交互**：集成语音识别和语音合成功能
- **大模型对话**：接入DeepSeek API，提供智能对话服务
- **集成聊天**：将语音输入、大模型和语音合成功能整合到一个聊天界面中

## 技术栈

- **后端**：Flask (Python)
- **前端**：HTML, CSS, JavaScript, Bootstrap 5
- **API**：DeepSeek API, 语音识别API, 语音合成API
- **二维码识别**：集成微信开源二维码引擎，通过图片识别二维码
- **数据存储**：JSON文件

## 网络配置

系统默认配置为192.168.144网段，访问地址为：
- 本地访问：http://localhost:5000
- 局域网访问：http://192.168.144.17:5000

如需修改网络配置，请更新以下文件：
1. webapp/templates/index.html中的网络连接信息
2. webapp/app.py中的host配置（如需更改）

## 安装与运行

1. 克隆项目到本地

```bash
git clone <项目地址>
cd 集成web
```

2. 创建并激活虚拟环境

```bash
# Windows
python -m venv new_venv
new_venv\Scripts\activate

# Linux/Mac
python -m venv new_venv
source new_venv/bin/activate
```

3. 安装依赖包

```bash
pip install -r requirements.txt
```
**注意**: 新增的二维码扫描功能依赖 `opencv-contrib-python-headless`。如果安装时遇到问题，可以尝试单独安装或查找适合您系统的版本。

4. 运行应用

```bash
cd webapp
python app.py
```

5. 在浏览器中访问 `http://localhost:5000` 或 `http://192.168.144.17:5000`

## 目录结构

```
集成web/
├── README.md           # 项目说明文档
├── 日志.txt            # 项目开发日志
├── new_venv/           # Python虚拟环境
└── webapp/             # Web应用主目录
    ├── app.py          # Flask应用主文件
    ├── simple_tts.py   # 语音合成模块
    ├── voice_recognition_simple.py  # 语音识别模块
    ├── deepseek_api.py # DeepSeek API调用模块
    ├── data/           # 数据存储目录
    │   ├── orders.json         # 待取订单数据
    │   └── taken_orders.json   # 已取订单数据
    ├── static/         # 静态资源目录
    │   ├── css/        # CSS样式文件
    │   ├── js/         # JavaScript脚本文件
    │   └── audio/      # 音频文件存储目录
    └── templates/      # HTML模板目录
        ├── index.html  # 主页面
        └── test_chat.html  # 聊天测试页面
└── 微信二维码扫描模块/ # 微信二维码扫描模块
    ├── wechat_qrcode_scanner.py # 扫描逻辑
    └── ... (模型文件)
```

## API接口

- `/api/orders` (GET): 获取所有当前订单
- `/api/taken_orders` (GET): 获取所有已取订单
- `/api/upload_order` (POST): 上传新订单
- `/api/take_order` (POST): 取走订单
- `/api/chat` (POST): 本地智能对话功能
- `/api/qrcode` (POST): 处理二维码数据
- `/api/scan_and_upload_order` (POST): **(新增)** 接收图片文件，扫描二维码并上传订单信息。这是为ROS机器人设计的接口。
- `/api/voice_recognition` (POST): 语音识别API
- `/api/text_to_speech` (POST): 文字转语音API
- `/api/ai_chat` (POST): 集成语音输入、大模型和语音合成的聊天功能

## 🤖 ROS机器人集成指南

本节将详细介绍如何配置一个ROS（机器人操作系统）节点，使其能够利用机器人的摄像头，持续地捕捉图像，并将其发送到本Web应用的API接口进行二维码扫描。

### 1. 核心思路

机器人端的程序需要完成以下任务：
1.  创建一个ROS节点。
2.  订阅机器人摄像头发布的图像话题（如 `/camera/image_raw`）。
3.  在回调函数中接收ROS图像消息，并使用 `cv_bridge` 将其转换为OpenCV图像格式。
4.  为了避免网络拥堵和服务器过载，**必须控制发送频率**（例如，每秒发送一张图片）。
5.  将OpenCV图像编码为JPEG格式。
6.  通过HTTP POST请求，将JPEG图像数据以 `multipart/form-data` 的形式发送到Web服务器的 `/api/scan_and_upload_order` 接口。
7.  （可选）在ROS控制台打印服务器的返回信息，以便调试。

### 2. 机器人端环境准备

请确保您的ROS工作环境中的Python环境中，安装了以下库：
```bash
pip install requests opencv-python
```
*   `requests`: 用于发送HTTP请求。
*   `opencv-python`: 用于图像处理和编码。（通常ROS环境中已自带，此命令可确保其存在）。
*   `rospy` 和 `cv_bridge` 是ROS自带的核心库，无需额外安装。

### 3. Python节点示例代码

以下是一个完整的Python脚本示例。您可以将其保存为 `qr_scanner_node.py` 并放置在您的ROS包的 `scripts` 目录下。

**重要提示：** 请务必将代码中的 `YOUR_SERVER_IP` 修改为您运行Web应用的电脑的实际IP地址。

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import requests
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

# --- 配置 ---
# Web服务器的地址和端口，请务必修改为您的实际IP地址！
SERVER_URL = "http://YOUR_SERVER_IP:5000/api/scan_and_upload_order"
# ROS摄像头话题
IMAGE_TOPIC = "/camera/image_raw" 
# 发送频率控制（秒）
SEND_INTERVAL = 2.0 

class QRScannerNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('qr_scanner_node', anonymous=True)
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 记录上一次发送的时间
        self.last_send_time = 0
        
        # 订阅摄像头话题
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
        
        rospy.loginfo("QR扫描节点已启动，正在监听话题: %s", IMAGE_TOPIC)

    def image_callback(self, data):
        # 检查是否到达发送时间间隔
        current_time = time.time()
        if current_time - self.last_send_time < SEND_INTERVAL:
            return # 未到时间，直接返回
            
        self.last_send_time = current_time
        rospy.loginfo("接收到图像，准备处理并发送...")

        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 将OpenCV图像编码为JPEG格式
        ret, jpeg_image = cv2.imencode('.jpg', cv_image)
        if not ret:
            rospy.logwarn("图像编码为JPEG失败")
            return

        # 发送图像到Web服务器
        try:
            files = {'file': ('image.jpg', jpeg_image.tobytes(), 'image/jpeg')}
            response = requests.post(SERVER_URL, files=files, timeout=5)
            
            # 打印服务器响应
            if response.status_code == 200:
                rospy.loginfo("服务器响应: %s", response.json())
            else:
                rospy.logwarn("服务器返回错误，状态码: %s, 内容: %s", response.status_code, response.text)
        
        except requests.exceptions.RequestException as e:
            rospy.logerr("请求Web服务器失败: %s", e)

def main():
    try:
        node = QRScannerNode()
        rospy.spin()
    except KeyboardInterrupt:
        print("节点已关闭")

if __name__ == '__main__':
    main()
```

### 4. 如何运行

1.  将上述代码保存为 `qr_scanner_node.py`。
2.  将其放入您的ROS包的 `scripts` 文件夹中。
3.  授予该文件可执行权限：`chmod +x qr_scanner_node.py`。
4.  启动ROS核心：`roscore`。
5.  启动您的机器人摄像头驱动。
6.  在一个新的终端中，运行该节点：`rosrun <your_package_name> qr_scanner_node.py`。

之后，您的机器人就会每隔2秒（可自行在代码中修改 `SEND_INTERVAL`）向Web服务器发送一次摄像头捕捉到的图像了。

## 备选方案：非ROS机器人集成

如果您不想使用ROS，或者您的机器人没有配置ROS环境，可以使用一个更简单的独立Python脚本来完成同样的功能。这种方法通用性更强，可以在任何装有Python和摄像头的设备上运行。

### 1. 机器人端环境准备

请确保您的机器人本地的Python环境中，安装了以下两个库：

```bash
pip install opencv-python requests
```
*   `opencv-python`: 用于打开摄像头和处理图像。
*   `requests`: 用于向Web服务器发送网络请求。

### 2. 拷贝并配置脚本

在本项目根目录下，我们已经为您准备好了一个名为 `simple_camera_sender.py` 的脚本。
1.  将 `simple_camera_sender.py` 文件拷贝到您的机器人上。
2.  使用文本编辑器打开这个文件。
3.  **务必修改 `SERVER_URL` 变量**，将其中的 `YOUR_SERVER_IP` 替换成您运行Web应用的电脑的局域网IP地址。
    ```python
    # 例如:
    SERVER_URL = "http://192.168.144.17:5000/api/scan_and_upload_order" 
    ```
4.  （可选）如果您的机器人有多个摄像头，默认的 `CAMERA_INDEX = 0` 可能无法正常工作。您可以尝试将其修改为 `1`, `2` ... 直到能成功打开正确的摄像头。

### 3. 如何运行

1.  在机器人上打开一个终端。
2.  进入 `simple_camera_sender.py` 所在的目录。
3.  运行脚本：
    ```bash
    python simple_camera_sender.py
    ```
程序启动后，您会看到它成功打开摄像头的提示。之后，它会按照设定的时间间隔（默认为2秒），不断地捕捉图像、发送并打印服务器的返回结果。按 `Ctrl+C` 即可停止程序。

## 特殊功能说明

1. **模型相关问题回复**：当用户询问关于模型、AI或身份的问题时，系统会返回指定的回答："我是由claude-4-sonnet模型支持的智能助手，专为Cursor IDE设计，可以帮您解决各类编程难题，请告诉我你需要什么帮助？"

2. **二维码数据映射**：系统预设了三种二维码数据（A/B/C），分别对应不同的外卖信息。

3. **机器人二维码扫描**：系统提供 `/api/scan_and_upload_order` 接口，可接收ROS机器人发送的包含二维码的图片。系统会自动识别图中的二维码（如A/B/C），映射为订单信息并存入系统，已存在的订单会自动忽略。

    **使用方法**:
    向 `http://<服务器IP>:5000/api/scan_and_upload_order` 发送 `POST` 请求，请求体为 `multipart/form-data`，其中包含一个名为 `file` 的文件字段。

4. **语音交互**：系统支持实时语音识别和语音合成，可以通过语音进行对话。

5. **大模型调用**：系统使用deepseek_api.py中的方式直接调用DeepSeek API，提高响应速度。

## 注意事项

- 确保系统安装了所有必要的依赖包
- 语音识别功能需要麦克风设备支持
- DeepSeek API需要有效的API密钥
- 系统默认监听所有网络接口，允许局域网内的其他设备访问

## 测试页面

访问 `http://localhost:5000/test` 或 `http://192.168.144.17:5000/test` 可以打开专门用于测试聊天功能的页面，该页面集成了语音输入、大模型调用和语音合成功能。 