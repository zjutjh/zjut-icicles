#!/usr/bin/env python3
# encoding: utf-8
"""
摄像头质量和YOLO检测测试工具 - SSH友好版本
不依赖GUI显示，通过保存图像文件来检测摄像头质量
"""

import os
import time
from datetime import datetime
from utils.yolo import YOLO
from PIL import Image
import numpy as np
import cv2
import tensorflow as tf

# 获取当前脚本所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))

def setup_gpu():
    """配置GPU内存增长"""
    gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
    for gpu in gpus:
        tf.config.experimental.set_memory_growth(gpu, True)

def initialize_yolo():
    """初始化YOLO模型"""
    print("正在初始化YOLO模型...")
    yolo = YOLO(
        model_path=os.path.join(current_dir, 'logs/last1.weights.h5'),
        classes_path=os.path.join(current_dir, 'model_data/CampusBot.txt'),
        anchors_path=os.path.join(current_dir, 'model_data/CampusBot_anchors.txt'),
        score=0.5,
        iou=0.3
    )
    print("YOLO模型初始化完成")
    return yolo

def open_camera():
    """打开摄像头设备"""
    print("尝试打开摄像头 /dev/video2...")
    capture = cv2.VideoCapture(2)
    
    if not capture.isOpened():
        print("错误：无法打开摄像头 /dev/video2")
        print("尝试其他摄像头设备...")
        for i in [0, 1, 3]:
            print(f"尝试摄像头设备 {i}...")
            capture = cv2.VideoCapture(i)
            if capture.isOpened():
                print(f"成功打开摄像头设备 /dev/video{i}")
                return capture, i
        print("无法找到可用的摄像头设备")
        return None, -1
    else:
        print("成功打开摄像头 /dev/video2")
        return capture, 2

def test_camera_quality(capture, test_duration=10):
    """测试摄像头视频质量"""
    print(f"\n📹 开始测试摄像头质量 ({test_duration}秒)...")
    
    # 获取摄像头信息
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_camera = capture.get(cv2.CAP_PROP_FPS)
    
    print(f"摄像头参数:")
    print(f"  - 分辨率: {width}x{height}")
    print(f"  - 理论FPS: {fps_camera}")
    
    # 创建输出目录
    output_dir = os.path.join(current_dir, 'camera_test_output')
    os.makedirs(output_dir, exist_ok=True)
    
    frame_count = 0
    start_time = time.time()
    last_save_time = start_time
    
    while time.time() - start_time < test_duration:
        ret, frame = capture.read()
        if not ret:
            print("⚠️ 无法读取摄像头数据")
            break
        
        frame_count += 1
        current_time = time.time()
        
        # 每2秒保存一张图像
        if current_time - last_save_time >= 2:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"camera_test_{timestamp}.jpg"
            filepath = os.path.join(output_dir, filename)
            
            cv2.imwrite(filepath, frame)
            print(f"✅ 保存测试图像: {filename}")
            last_save_time = current_time
        
        # 显示进度
        if frame_count % 30 == 0:
            elapsed = current_time - start_time
            actual_fps = frame_count / elapsed
            print(f"📊 测试进度: {elapsed:.1f}s/{test_duration}s, 实际FPS: {actual_fps:.1f}")
    
    # 计算最终统计
    total_time = time.time() - start_time
    average_fps = frame_count / total_time
    
    print(f"\n📈 摄像头质量测试结果:")
    print(f"  - 总帧数: {frame_count}")
    print(f"  - 测试时长: {total_time:.2f}秒")
    print(f"  - 平均FPS: {average_fps:.2f}")
    print(f"  - 理论FPS: {fps_camera}")
    print(f"  - FPS效率: {(average_fps/fps_camera)*100:.1f}%")
    print(f"  - 测试图像保存在: {output_dir}")
    
    return average_fps, width, height

def test_yolo_detection(capture, yolo, test_duration=30):
    """测试YOLO检测功能"""
    print(f"\n🎯 开始测试YOLO检测功能 ({test_duration}秒)...")
    
    # 创建输出目录
    output_dir = os.path.join(current_dir, 'yolo_test_output')
    os.makedirs(output_dir, exist_ok=True)
    
    detection_count = 0
    frame_count = 0
    total_objects = 0
    detection_stats = {}
    
    start_time = time.time()
    last_save_time = start_time
    
    try:
        while time.time() - start_time < test_duration:
            ret, frame = capture.read()
            if not ret:
                print("⚠️ 无法读取摄像头数据")
                break
            
            frame_count += 1
            
            # 颜色空间转换
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(np.uint8(frame_rgb))
            
            # YOLO检测
            detection_start = time.time()
            result_image, out_boxes, out_scores, out_classes = yolo.detect_image(pil_image)
            detection_time = time.time() - detection_start
            
            # 统计检测结果
            if len(out_boxes) > 0:
                detection_count += 1
                total_objects += len(out_boxes)
                
                # 统计各类别检测数量
                for cls in out_classes:
                    class_name = yolo.class_names[int(cls)]
                    detection_stats[class_name] = detection_stats.get(class_name, 0) + 1
                
                print(f"🔍 检测到 {len(out_boxes)} 个目标, 检测用时: {detection_time:.3f}s")
            
            # 定期保存检测结果图像
            current_time = time.time()
            if current_time - last_save_time >= 5:  # 每5秒保存一次
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                
                # 保存原图
                original_filename = f"original_{timestamp}.jpg"
                cv2.imwrite(os.path.join(output_dir, original_filename), frame)
                
                # 保存检测结果图
                result_array = np.array(result_image)
                result_bgr = cv2.cvtColor(result_array, cv2.COLOR_RGB2BGR)
                result_filename = f"detection_{timestamp}.jpg"
                cv2.imwrite(os.path.join(output_dir, result_filename), result_bgr)
                
                print(f"💾 保存检测结果: {result_filename}")
                last_save_time = current_time
            
            # 显示进度
            if frame_count % 60 == 0:
                elapsed = current_time - start_time
                print(f"📊 检测进度: {elapsed:.1f}s/{test_duration}s, 检测率: {detection_count/frame_count*100:.1f}%")
                
    except KeyboardInterrupt:
        print("用户中断测试")
    
    # 计算最终统计
    total_time = time.time() - start_time
    
    print(f"\n🎯 YOLO检测测试结果:")
    print(f"  - 总帧数: {frame_count}")
    print(f"  - 检测到目标的帧数: {detection_count}")
    print(f"  - 检测率: {detection_count/frame_count*100:.1f}%")
    print(f"  - 总检测目标数: {total_objects}")
    print(f"  - 平均每帧目标数: {total_objects/frame_count:.2f}")
    
    if detection_stats:
        print(f"  - 检测类别统计:")
        for class_name, count in detection_stats.items():
            print(f"    * {class_name}: {count}次")
    
    print(f"  - 检测结果保存在: {output_dir}")

def main():
    """主函数"""
    print("🚀 摄像头和YOLO检测测试工具")
    print("=" * 50)
    
    # 配置GPU
    setup_gpu()
    
    # 打开摄像头
    capture, device_id = open_camera()
    if capture is None:
        return
    
    try:
        # 1. 测试摄像头质量
        avg_fps, width, height = test_camera_quality(capture, test_duration=10)
        
        # 2. 初始化YOLO
        yolo = initialize_yolo()
        
        # 3. 测试YOLO检测
        test_yolo_detection(capture, yolo, test_duration=30)
        
        print("\n✅ 测试完成！")
        print("请查看生成的图像文件来评估摄像头质量和检测效果。")
        
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
    finally:
        capture.release()
        print("📸 摄像头资源已释放")

if __name__ == "__main__":
    main() 