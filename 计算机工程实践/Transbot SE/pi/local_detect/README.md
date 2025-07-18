# 视频检测脚本使用说明

## 文件说明

### 1. video_to_images.py
- **功能**: 从 `/dev/video2` 摄像头捕获2秒视频，以10FPS保存图像文件
- **帧率控制**: 每秒保存10帧，共约20张图像
- **输出**: 图像文件保存在 `captured_images/` 文件夹中
- **文件名格式**: `frame_YYYYMMDD_HHMMSS_XXXX.jpg`

### 2. scan_monitor.py
- **功能**: 监听 `scan.txt` 文件变化，根据内容执行不同操作
- **监听文件**: `/home/pi/src/transbot_ws/src/transbot_composite_app/temp/scan.txt`
- **触发条件**: 
  - 内容为 "1": 执行视频捕获
  - 内容为 "2": 执行图像检测
  - 其他非0值: 执行视频捕获

### 3. image_detector.py
- **功能**: 对 `captured_images/` 中的图像进行YOLO检测，统计识别结果
- **检测模型**: YOLOv4-tiny，可识别 fire, turn_left, turn_right, turn_round, construction, stop
- **输出结果**: 统计数量最多的类别，保存到 `left_right_fire_cons.txt`
- **执行后操作**: 自动清空 `captured_images/` 目录

### 4. qr_detector.py
- **功能**: 对 `captured_images/` 中的图像进行二维码检测，识别A、B、C值
- **检测服务**: 调用远程二维码识别服务
- **输出结果**: 识别到的二维码值（a、b、c），保存到 `a_b_c.txt`
- **默认值**: 如果未识别到有效二维码，默认保存 "a"
- **执行后操作**: 自动清空 `captured_images/` 目录

### 5. status_uploader.py
- **功能**: 监听配送状态变化，自动上传餐品信息和配送状态到服务器
- **监听文件**: `/home/pi/src/transbot_ws/src/transbot_composite_app/temp/end.txt`
- **餐品信息**: 从 `/home/pi/src/transbot_ws/src/transbot_composite_app/temp_in/a_b_c.txt` 读取
- **服务器地址**: `192.168.144.121:5000/api/delivery_status`
- **状态映射**: 0-正在拾取, 1-已拾取, 2-已送达

### 6. web_server_example.py
- **功能**: Web服务器端示例，接收和管理配送状态数据
- **端口**: 5000
- **API端点**: `/api/delivery_status` (POST/GET)
- **数据存储**: 自动保存配送记录到JSON文件
- **日志记录**: 记录所有配送状态变化

## 使用方法

### 单独运行视频捕获脚本
```bash
cd /home/pi/local_detect
python3 video_to_images.py
```

### 单独运行图像检测脚本
```bash
cd /home/pi/local_detect
python3 image_detector.py
```

### 单独运行二维码检测脚本
```bash
cd /home/pi/local_detect
python3 qr_detector.py
```

### 启动文件监听器
```bash
cd /home/pi/local_detect
python3 scan_monitor.py
```

### 后台运行监听器
```bash
cd /home/pi/local_detect
nohup python3 scan_monitor.py > monitor.log 2>&1 &
```

### 运行配送状态上传脚本
```bash
cd /home/pi/local_detect
python3 status_uploader.py
```

### 启动Web服务器（服务器端）
```bash
# 在服务器端 (192.168.144.121) 运行
cd /path/to/server
pip install -r server_requirements.txt
python3 web_server_example.py
```

## 工作流程（串行执行）

### 基本检测流程
1. 启动 `scan_monitor.py` 监听器
2. 监听器会实时检查 `scan.txt` 和 `end.txt` 文件的内容
3. 根据 `scan.txt` 内容执行不同操作：
   - **内容为 "1"**: 先执行视频捕获，完成后自动执行二维码检测
   - **内容为 "2"**: 先执行视频捕获，完成后自动执行图像检测
   - **其他非0值**: 只执行视频捕获

### 配送状态上传流程
4. 当 `end.txt` 文件内容发生变化时，自动触发配送状态上传：
   - **end.txt = "0"**: 正在拾取状态，上传到服务器
   - **end.txt = "1"**: 已拾取状态，上传到服务器
   - **end.txt = "2"**: 已送达状态，上传到服务器
5. 状态上传包含的信息：
   - 当前餐品编号（从 `a_b_c.txt` 读取）
   - 配送状态码和描述
   - 机器人ID和时间戳
4. **串行执行逻辑**:
   - 对于内容为 "1" 的情况，系统会：
     1. 首先执行视频捕获脚本（2秒，10FPS）
     2. 等待视频捕获完成
     3. 自动执行二维码检测脚本
     4. 完成后清空图像目录
   - 对于内容为 "2" 的情况，系统会：
     1. 首先执行视频捕获脚本（2秒，10FPS）
     2. 等待视频捕获完成
     3. 自动执行图像检测脚本
     4. 完成后清空图像目录
5. **视频捕获流程**:
   - 捕获时长: 2秒
   - 帧率控制: 每秒10帧，总共约20张图像
   - 图像文件保存在 `captured_images/` 目录
   - 文件名格式: `frame_YYYYMMDD_HHMMSS_XXXX.jpg`
6. **图像检测流程**:
   - 读取 `captured_images/` 中的所有图像
   - 使用YOLO模型检测每张图像
   - 统计所有检测结果，选择数量最多的类别
   - 将结果保存到 `left_right_fire_cons.txt`
   - 清空 `captured_images/` 目录

## 注意事项

- 确保摄像头 `/dev/video2` 可用
- 如果 `/dev/video2` 不可用，脚本会自动尝试其他摄像头设备
- 监听器使用0.5秒的检查间隔
- 如果视频脚本或检测脚本正在运行，不会重复执行
- 按 `Ctrl+C` 可以停止监听器
- 图像检测需要YOLO模型文件，确保路径正确
- 二维码检测需要远程识别服务，确保服务器可访问
- scan.txt = 1: 结果保存到 `a_b_c.txt`，默认值为 "a"
- scan.txt = 2: 结果保存到 `left_right_fire_cons.txt`，默认值为 "turn_left"
- 所有检测完成后都会自动清空 `captured_images/` 目录
- 配送状态上传需要服务器端运行Web服务器（端口5000）
- 确保网络连接正常，机器人能访问服务器 `192.168.144.121:5000`

## Web服务器配置

### 服务器端部署
1. 在服务器（192.168.144.121）上创建目录:
```bash
mkdir -p /opt/delivery_server
cd /opt/delivery_server
```

2. 复制服务器文件:
```bash
# 复制以下文件到服务器
- web_server_example.py
- server_requirements.txt
```

3. 安装依赖:
```bash
pip install -r server_requirements.txt
```

4. 启动服务器:
```bash
python3 web_server_example.py
```

### API端点说明

#### 1. 接收配送状态 (POST)
- **URL**: `http://192.168.144.121:5000/api/delivery_status`
- **方法**: POST
- **请求格式**: JSON
- **请求参数**:
  ```json
  {
    "food_item": "A",           // 餐品编号 (A/B/C)
    "status_code": "1",         // 状态码 (0/1/2)
    "status_text": "已拾取",    // 状态描述
    "timestamp": "2024-06-30 14:30:52",
    "robot_id": "transbot_01"   // 机器人ID
  }
  ```
- **响应格式**:
  ```json
  {
    "success": true,
    "message": "配送状态更新成功",
    "data": {
      "received_at": "2024-06-30 14:30:52",
      "processed": true
    }
  }
  ```

#### 2. 查询配送状态 (GET)
- **URL**: `http://192.168.144.121:5000/api/delivery_status`
- **方法**: GET
- **参数**: `robot_id` (可选)
- **响应**: 当前配送状态信息

#### 3. 查询配送历史 (GET)
- **URL**: `http://192.168.144.121:5000/api/delivery_history`
- **方法**: GET
- **参数**: `limit`, `robot_id` (可选)
- **响应**: 配送历史记录

#### 4. 健康检查 (GET)
- **URL**: `http://192.168.144.121:5000/health`
- **方法**: GET
- **响应**: 服务器状态信息

## 输出示例

### 视频捕获输出
视频捕获脚本运行后，会在 `captured_images/` 目录下生成类似以下的文件：
```
frame_20240630_143052_0001.jpg
frame_20240630_143052_0002.jpg
frame_20240630_143052_0003.jpg
...
```

### 图像检测输出（scan.txt = 2）
图像检测脚本运行后，会在 `left_right_fire_cons.txt` 中保存检测结果：
```
turn_left
```
或者
```
fire
```

检测过程会输出类似以下信息：
```
检测统计结果: {'turn_left': 15, 'fire': 3, 'stop': 1}
数量最多的类别: turn_left
```

### 二维码检测输出（scan.txt = 1）
二维码检测脚本运行后，会在 `a_b_c.txt` 中保存检测结果：
```
a
```
或者
```
b
```
或者
```
c
```

检测过程会输出类似以下信息：
```
找到 20 个图像文件
正在处理图像 1/20: frame_20240630_143052_0001.jpg
  识别到二维码: ['A']
找到有效二维码，停止继续扫描
最终识别结果: ['A']
二维码识别结果已保存到: a_b_c.txt
结果内容: a
```

## 故障排除

1. **摄像头无法打开**: 检查摄像头设备是否正确连接
2. **权限错误**: 确保脚本有执行权限 (`chmod +x *.py`)
3. **scan.txt 文件不存在**: 检查文件路径是否正确
4. **Python依赖**: 确保已安装 `opencv-python` (`pip install opencv-python`)
5. **YOLO模型无法加载**: 
   - 检查模型文件路径: `/home/pi/temp/software/yolov4-tiny-tf2/logs/last1.weights.h5`
   - 检查类别文件路径: `/home/pi/temp/software/yolov4-tiny-tf2/model_data/CampusBot.txt`
   - 确保已安装 `tensorflow` (`pip install tensorflow`)
6. **图像检测结果异常**: 
   - 检查 `captured_images/` 目录是否有图像文件
   - 查看检测脚本的详细输出信息
7. **二维码检测失败**:
   - 检查二维码识别服务器是否可访问（默认: 192.168.144.121:5000）
   - 确认网络连接正常
   - 查看二维码检测脚本的详细输出信息
8. **结果文件无法写入**: 检查文件路径的写入权限
   - `left_right_fire_cons.txt` (scan.txt = 2)
   - `a_b_c.txt` (scan.txt = 1)
9. **配送状态上传失败**:
   - 检查服务器是否运行在 `192.168.144.121:5000`
   - 验证网络连接: `ping 192.168.144.121`
   - 测试API端点: `curl http://192.168.144.121:5000/health`
   - 检查防火墙设置，确保端口5000开放
10. **Web服务器启动失败**:
    - 确认已安装Flask: `pip install flask`
    - 检查端口5000是否被占用: `netstat -tlnp | grep 5000`
    - 查看服务器日志文件: `delivery_logs.log`
11. **end.txt文件监听异常**:
    - 确认文件存在: `/home/pi/src/transbot_ws/src/transbot_composite_app/temp/end.txt`
    - 检查文件权限，确保可读
    - 验证监听器是否正常工作 