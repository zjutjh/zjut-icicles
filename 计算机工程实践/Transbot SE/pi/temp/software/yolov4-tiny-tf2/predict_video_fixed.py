#!/usr/bin/env python3
# encoding: utf-8
import os
from utils.yolo import YOLO
from PIL import Image
import numpy as np
import cv2
import time
import tensorflow as tf

# 获取当前脚本所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))

gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)
fps = 0.0

# 使用绝对路径初始化YOLO
yolo = YOLO(
    model_path=os.path.join(current_dir, 'logs/last1.weights.h5'),
    classes_path=os.path.join(current_dir, 'model_data/CampusBot.txt'),
    anchors_path=os.path.join(current_dir, 'model_data/CampusBot_anchors.txt'),
    score=0.5,
    iou=0.3
)

if __name__ == '__main__':
    print("正在初始化YOLO视频检测系统...")
    print(f"模型路径: {os.path.join(current_dir, 'logs/last1.weights.h5')}")
    print(f"类别文件: {os.path.join(current_dir, 'model_data/CampusBot.txt')}")
    
    # 使用 /dev/video2 摄像头（第二个USB摄像头）
    print("尝试打开摄像头 /dev/video2...")
    capture = cv2.VideoCapture(2)
    
    # 检查摄像头是否成功打开
    if not capture.isOpened():
        print("错误：无法打开摄像头 /dev/video2")
        print("尝试其他摄像头设备...")
        # 尝试使用其他设备
        for i in [0, 1, 3]:
            print(f"尝试摄像头设备 {i}...")
            capture = cv2.VideoCapture(i)
            if capture.isOpened():
                print(f"成功打开摄像头设备 /dev/video{i}")
                break
        else:
            print("无法找到可用的摄像头设备")
            exit(1)
    else:
        print("成功打开摄像头 /dev/video2")
    
    # 获取摄像头信息
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_camera = capture.get(cv2.CAP_PROP_FPS)
    print(f"摄像头分辨率: {width}x{height}, FPS: {fps_camera}")
    print("开始检测，按 Ctrl+C 退出...")
    
    try:
        while True:
            t1 = time.time()
            # 读取视频帧
            ret, frame = capture.read()
            if not ret:
                print("无法读取摄像头数据")
                break
                
            # 颜色空间转换：BGR转RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # 转换为PIL Image格式
            frame = Image.fromarray(np.uint8(frame))
            # YOLO检测
            frame, out_boxes, out_scores, out_classes = yolo.detect_image(frame)
            frame = np.array(frame)
            # RGB转BGR供opencv显示
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # 计算FPS
            fps = (fps + abs(1. / (time.time() - t1))) / 2
            frame = cv2.putText(frame, "FPS: %.2f" % (fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 显示检测信息
            if len(out_boxes) > 0:
                print(f"检测到 {len(out_boxes)} 个目标")
            
            # 显示视频
            cv2.imshow("YOLO Detection", frame)
            
            # 检查退出键
            key = cv2.waitKey(1) & 0xff
            if key == 27 or key == ord('q'):  # ESC或q键退出
                break
                
    except KeyboardInterrupt:
        print("用户中断检测")
    except Exception as e:
        print(f"检测过程中发生错误: {e}")
    finally:
        capture.release()
        cv2.destroyAllWindows()
        print("检测结束，资源已释放") 