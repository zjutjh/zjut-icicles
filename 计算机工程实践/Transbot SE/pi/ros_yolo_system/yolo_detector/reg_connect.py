#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import json
import shutil
import argparse
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import threading
from datetime import datetime

# 添加当前目录到系统路径
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# 添加YOLO路径到系统路径
YOLO_PATH = "/home/pi/temp/software/yolov4-tiny-tf2"
if YOLO_PATH not in sys.path:
    sys.path.append(YOLO_PATH)

# 导入YOLO检测接口
try:
    # 先尝试从当前目录导入
    from predict_img import detect_image, get_yolo_instance
except ImportError as e:
    print(f"警告：无法从当前目录导入YOLO检测接口: {e}")
    try:
        # 尝试直接从YOLO路径导入
        sys.path.insert(0, YOLO_PATH)
        from predict_img import detect_image, get_yolo_instance
    except ImportError as e:
        print(f"错误：无法导入YOLO检测接口，请确保路径正确: {e}")
        sys.exit(1)

class ImageWatcher(FileSystemEventHandler):
    """监控共享目录中的图像文件并调用YOLO进行检测"""
    
    def __init__(self, images_dir, results_dir, yolo_path):
        self.images_dir = images_dir
        self.results_dir = results_dir
        self.yolo_path = yolo_path
        
        # 确保结果目录存在
        os.makedirs(self.results_dir, exist_ok=True)
        
        # 初始化YOLO实例
        print(f"初始化YOLO模型...")
        model_path = os.path.join(self.yolo_path, "logs/last1.weights.h5")
        classes_path = os.path.join(self.yolo_path, "model_data/CampusBot.txt")
        anchors_path = os.path.join(self.yolo_path, "model_data/CampusBot_anchors.txt")
        self.yolo = get_yolo_instance(
            model_path=model_path,
            classes_path=classes_path,
            anchors_path=anchors_path,
            score=0.5,
            iou=0.3
        )
        
        # 记录已处理的文件
        self.processed_files = set()
        
        # 创建处理队列和线程
        self.process_queue = []
        self.queue_lock = threading.Lock()
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        print(f"初始化完成，监控目录: {self.images_dir}")
        print(f"YOLO路径: {self.yolo_path}")
        
    def on_created(self, event):
        """当新文件创建时调用"""
        if not event.is_directory and self._is_image_file(event.src_path):
            with self.queue_lock:
                # 避免重复添加
                if event.src_path not in self.process_queue and event.src_path not in self.processed_files:
                    self.process_queue.append(event.src_path)
                    print(f"添加新图像到处理队列: {os.path.basename(event.src_path)}")
    
    def _is_image_file(self, filepath):
        """检查文件是否为图像文件"""
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.gif']
        ext = os.path.splitext(filepath)[1].lower()
        return ext in image_extensions
    
    def process_images(self):
        """处理队列中的图像"""
        while True:
            # 从队列中获取图像
            image_path = None
            with self.queue_lock:
                if self.process_queue:
                    image_path = self.process_queue.pop(0)
            
            if image_path:
                # 避免重复处理
                if image_path in self.processed_files:
                    continue
                
                try:
                    # 等待文件写入完成
                    time.sleep(0.2)
                    
                    # 检查文件是否存在
                    if not os.path.exists(image_path):
                        print(f"文件不存在，跳过处理: {image_path}")
                        continue
                    
                    # 获取文件名
                    image_filename = os.path.basename(image_path)
                    print(f"处理图像: {image_filename}")
                    
                    # 直接调用YOLO检测接口
                    detections = detect_image(
                        image_path=image_path,
                        output_dir=self.results_dir,
                        show=False
                    )
                    
                    # 保存结果为JSON
                    result_filename = f"result_{os.path.splitext(image_filename)[0]}.json"
                    result_path = os.path.join(self.results_dir, result_filename)
                    
                    # 准备结果数据
                    result_data = {
                        "image_id": image_filename,
                        "timestamp": datetime.now().isoformat(),
                        "class_names": [],
                        "counts": []
                    }
                    
                    # 统计每个类别的数量
                    class_counts = {}
                    for item in detections:
                        class_name = item["class"]
                        if class_name in class_counts:
                            class_counts[class_name] += 1
                        else:
                            class_counts[class_name] = 1
                    
                    # 填充结果数据
                    for class_name, count in class_counts.items():
                        result_data["class_names"].append(class_name)
                        result_data["counts"].append(count)
                    
                    # 保存JSON结果
                    with open(result_path, 'w') as f:
                        json.dump(result_data, f)
                    
                    print(f"保存检测结果: {result_filename}")
                    print(f"检测到 {len(detections)} 个目标: {class_counts}")
                    
                    # 添加到已处理文件集合
                    self.processed_files.add(image_path)
                    
                    # 限制已处理文件集合的大小
                    if len(self.processed_files) > 1000:
                        self.processed_files = set(list(self.processed_files)[-500:])
                    
                except Exception as e:
                    print(f"处理图像时出错: {str(e)}")
            
            # 短暂休眠以减少CPU使用
            time.sleep(0.1)

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='监控共享目录中的图像并调用YOLO进行检测')
    parser.add_argument('--images_dir', type=str, default='/home/pi/ros_yolo_system/shared/images',
                        help='共享图像目录路径')
    parser.add_argument('--results_dir', type=str, default='/home/pi/ros_yolo_system/shared/results',
                        help='检测结果保存目录路径')
    parser.add_argument('--yolo_path', type=str, default='/home/pi/temp/software/yolov4-tiny-tf2',
                        help='YOLO目录路径')
    args = parser.parse_args()
    
    # 确保目录存在
    os.makedirs(args.images_dir, exist_ok=True)
    os.makedirs(args.results_dir, exist_ok=True)
    
    # 创建观察者
    observer = Observer()
    event_handler = ImageWatcher(args.images_dir, args.results_dir, args.yolo_path)
    observer.schedule(event_handler, args.images_dir, recursive=False)
    observer.start()
    
    # 处理已存在的图像
    existing_images = [os.path.join(args.images_dir, f) 
                      for f in os.listdir(args.images_dir) 
                      if event_handler._is_image_file(os.path.join(args.images_dir, f))]
    
    if existing_images:
        print(f"处理目录中已存在的 {len(existing_images)} 张图像...")
        for img_path in existing_images:
            with event_handler.queue_lock:
                event_handler.process_queue.append(img_path)
    
    try:
        print("开始监控图像目录，按Ctrl+C退出...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    
    observer.join()

if __name__ == "__main__":
    main() 