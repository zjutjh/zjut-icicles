#!/usr/bin/env python3
# coding: utf-8
import os
import time
from utils.yolo import YOLO
from PIL import Image
import tensorflow as tf
import numpy as np
import cv2

# 共享目录路径
SHARED_DIR = "/home/pi/temp/shared_images"
# 输出目录
OUTPUT_DIR = "/home/pi/temp/output_images"

def main():
    # 配置GPU内存增长
    gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
    for gpu in gpus:
        tf.config.experimental.set_memory_growth(gpu, True)
    
    # 创建YOLO实例
    print("初始化YOLO模型...")
    yolo = YOLO()
    
    # 确保输出目录存在
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"创建输出目录: {OUTPUT_DIR}")
    
    # 确保共享目录存在
    if not os.path.exists(SHARED_DIR):
        os.makedirs(SHARED_DIR)
        print(f"创建共享目录: {SHARED_DIR}")
    
    print("开始监视共享目录中的新图像...")
    processed_files = set()
    
    try:
        while True:
            flag_file = os.path.join(SHARED_DIR, "new_image.txt")
            if os.path.exists(flag_file):
                try:
                    # 读取新图像文件路径
                    with open(flag_file, "r") as f:
                        image_file = f.read().strip()
                    
                    if os.path.exists(image_file) and image_file not in processed_files:
                        print(f"发现新图像: {image_file}")
                        
                        # 打开并处理图像
                        image = Image.open(image_file)
                        r_image, boxes, scores, classes = yolo.detect_image(image)
                        
                        # 保存结果
                        base_name = os.path.basename(image_file)
                        output_path = os.path.join(OUTPUT_DIR, f"detected_{base_name}")
                        r_image.save(output_path)
                        
                        # 打印检测结果
                        print(f"检测到 {len(boxes)} 个有效目标:")
                        for i in range(len(boxes)):
                            cls = int(classes[i])
                            class_name = yolo.class_names[cls]
                            score = float(scores[i])
                            box = boxes[i]
                            print(f"- {class_name}: 置信度 {score:.2f}, 位置: {box}")
                        
                        processed_files.add(image_file)
                        print(f"结果已保存至: {output_path}")
                        
                        # 显示结果（可选）
                        try:
                            cv2_image = np.array(r_image)
                            cv2_image = cv2.cvtColor(cv2_image, cv2.COLOR_RGB2BGR)
                            cv2.imshow("YOLO Detection", cv2_image)
                            cv2.waitKey(1)  # 显示图像但不阻塞
                        except Exception as e:
                            print(f"显示图像时出错: {e}")
                        
                        # 处理完后删除标志文件
                        os.remove(flag_file)
                except Exception as e:
                    print(f"处理图像时出错: {e}")
                    # 如果处理出错，也删除标志文件，防止卡住
                    try:
                        os.remove(flag_file)
                    except:
                        pass
            
            # 清理过多的处理过的文件记录，防止内存增长
            if len(processed_files) > 100:
                oldest_files = list(processed_files)[:50]
                for old_file in oldest_files:
                    processed_files.remove(old_file)
            
            # 防止CPU占用过高
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("程序被用户中断")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
