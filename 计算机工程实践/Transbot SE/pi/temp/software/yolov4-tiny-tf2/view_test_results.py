#!/usr/bin/env python3
# encoding: utf-8
"""
测试结果查看工具
显示测试生成的图像文件信息和基本统计
"""

import os
import cv2
from datetime import datetime

def analyze_images(directory):
    """分析目录中的图像文件"""
    if not os.path.exists(directory):
        print(f"目录不存在: {directory}")
        return
    
    image_files = []
    for file in os.listdir(directory):
        if file.lower().endswith(('.jpg', '.jpeg', '.png')):
            image_files.append(file)
    
    if not image_files:
        print(f"目录中没有图像文件: {directory}")
        return
    
    print(f"\n📁 目录: {directory}")
    print(f"📸 图像文件数量: {len(image_files)}")
    
    # 显示文件列表和基本信息
    total_size = 0
    for i, filename in enumerate(sorted(image_files)[:10]):  # 只显示前10个
        filepath = os.path.join(directory, filename)
        file_size = os.path.getsize(filepath)
        total_size += file_size
        
        # 读取图像尺寸
        try:
            img = cv2.imread(filepath)
            if img is not None:
                height, width = img.shape[:2]
                print(f"  {i+1:2d}. {filename:<30} - {width}x{height} - {file_size/1024:.1f}KB")
            else:
                print(f"  {i+1:2d}. {filename:<30} - 无法读取")
        except:
            print(f"  {i+1:2d}. {filename:<30} - 读取错误")
    
    if len(image_files) > 10:
        print(f"  ... 还有 {len(image_files)-10} 个文件")
    
    print(f"📊 总大小: {total_size/1024/1024:.2f}MB")
    
    return image_files

def main():
    """主函数"""
    print("🔍 测试结果查看工具")
    print("=" * 50)
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 检查摄像头测试结果
    camera_test_dir = os.path.join(current_dir, 'camera_test_output')
    print("📹 摄像头质量测试结果:")
    camera_files = analyze_images(camera_test_dir)
    
    # 检查YOLO检测结果
    yolo_test_dir = os.path.join(current_dir, 'yolo_test_output')
    print("\n🎯 YOLO检测测试结果:")
    yolo_files = analyze_images(yolo_test_dir)
    
    print("\n💡 建议:")
    print("1. 通过SFTP或SCP下载图像文件到本地查看")
    print("2. 使用以下命令下载文件:")
    print(f"   scp -r pi@your_pi_ip:{camera_test_dir} ./")
    print(f"   scp -r pi@your_pi_ip:{yolo_test_dir} ./")
    print("3. 检查图像清晰度、亮度和检测框的准确性")

if __name__ == "__main__":
    main() 