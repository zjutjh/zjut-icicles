#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import argparse
from PIL import Image
import tensorflow as tf
import numpy as np

# 添加YOLO路径到系统路径
YOLO_PATH = "/home/pi/temp/software/yolov4-tiny-tf2"
if YOLO_PATH not in sys.path:
    sys.path.append(YOLO_PATH)

# 导入YOLO类
try:
    from utils.yolo import YOLO
except ImportError:
    print(f"错误：无法导入YOLO类，请确保路径正确: {YOLO_PATH}")
    sys.exit(1)

# 全局YOLO实例，避免重复加载模型
_yolo_instance = None

def get_yolo_instance(model_path=None, classes_path=None, anchors_path=None, score=0.5, iou=0.3):
    """
    获取或创建YOLO实例（单例模式）
    """
    global _yolo_instance
    
    if _yolo_instance is None:
        # 获取当前脚本所在的目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 使用默认路径或提供的路径
        if model_path is None:
            model_path = os.path.join(YOLO_PATH, 'logs/last1.weights.h5')
        if classes_path is None:
            classes_path = os.path.join(YOLO_PATH, 'model_data/CampusBot.txt')
        if anchors_path is None:
            anchors_path = os.path.join(YOLO_PATH, 'model_data/CampusBot_anchors.txt')
        
        # 配置GPU内存增长
        gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        
        print(f"加载YOLO模型...")
        print(f"模型路径: {model_path}")
        print(f"类别文件: {classes_path}")
        print(f"锚框文件: {anchors_path}")
        
        # 创建YOLO实例
        _yolo_instance = YOLO(
            model_path=model_path,
            classes_path=classes_path,
            anchors_path=anchors_path,
            score=score,
            iou=iou
        )
    
    return _yolo_instance

def detect_image(image_path, output_dir=None, show=False):
    """
    检测单个图像的接口函数
    
    参数:
        image_path: 图像文件路径
        output_dir: 输出目录，如果为None则不保存结果图像
        show: 是否显示结果图像
        
    返回:
        detections: 检测结果列表，每个元素是一个字典，包含类别、置信度和边界框
    """
    # 获取YOLO实例
    yolo = get_yolo_instance()
    
    # 打开图像
    try:
        image = Image.open(image_path)
    except FileNotFoundError:
        print(f"打开错误！找不到文件: {image_path}")
        return []
    except Exception as e:
        print(f"打开图片时发生未知错误 {image_path}: {e}")
        return []
    
    # 执行检测
    try:
        r_image, boxes, scores, classes = yolo.detect_image(image)
        
        # 构建检测结果
        detections = []
        for i in range(len(boxes)):
            cls = int(classes[i])
            class_name = yolo.class_names[cls]
            score = float(scores[i])
            box = boxes[i].tolist()  # 转换为普通列表
            
            detections.append({
                "class": class_name,
                "confidence": score,
                "box": box
            })
        
        # 保存结果图像（如果需要）
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
            base_name = os.path.basename(image_path)
            save_path = os.path.join(output_dir, f"output_{base_name}")
            r_image.save(save_path)
        
        # 显示图像（如果需要）
        if show:
            try:
                r_image.show()
            except:
                print("无法显示图像")
        
        return detections
    
    except Exception as e:
        print(f"处理图片时发生错误: {e}")
        return []

def main():
    """
    命令行入口函数
    """
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='YOLOv4-tiny目标检测')
    parser.add_argument('--model', type=str, default=os.path.join(YOLO_PATH, 'logs/last1.weights.h5'),
                        help='模型权重文件路径')
    parser.add_argument('--classes', type=str, default=os.path.join(YOLO_PATH, 'model_data/CampusBot.txt'),
                        help='类别名称文件路径')
    parser.add_argument('--anchors', type=str, default=os.path.join(YOLO_PATH, 'model_data/CampusBot_anchors.txt'),
                        help='锚框文件路径')
    parser.add_argument('--score', type=float, default=0.5,
                        help='置信度阈值')
    parser.add_argument('--iou', type=float, default=0.3,
                        help='IOU阈值')
    parser.add_argument('--input_dir', type=str, required=True,
                        help='输入图片目录')
    parser.add_argument('--output_dir', type=str, required=True,
                        help='输出图片保存目录')
    parser.add_argument('--show', action='store_true',
                        help='是否显示结果图片')
    args = parser.parse_args()
    
    # 确保输出目录存在
    os.makedirs(args.output_dir, exist_ok=True)
    
    # 打印实际使用的路径，便于调试
    print(f"使用模型文件: {args.model}")
    print(f"使用类别文件: {args.classes}")
    print(f"使用锚框文件: {args.anchors}")
    
    # 初始化YOLO实例
    get_yolo_instance(
        model_path=args.model,
        classes_path=args.classes,
        anchors_path=args.anchors,
        score=args.score,
        iou=args.iou
    )
    
    # 获取输入目录中的所有图像
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.gif']
    images_to_test = []
    for file in os.listdir(args.input_dir):
        ext = os.path.splitext(file)[1].lower()
        if ext in image_extensions:
            images_to_test.append(os.path.join(args.input_dir, file))
    
    if not images_to_test:
        print("错误：输入目录中没有找到图像文件")
        return
    
    print(f"将要处理 {len(images_to_test)} 张图片")
    
    # 处理每张图片
    for img_path in images_to_test:
        print(f"\n正在处理图片: {img_path}")
        
        # 使用接口函数进行检测
        detections = detect_image(img_path, args.output_dir, args.show)
        
        # 打印检测结果
        print(f"图片 {img_path} 中检测到 {len(detections)} 个有效目标:")
        for detection in detections:
            print(f"- {detection['class']}: 置信度 {detection['confidence']:.2f}, 位置: {detection['box']}")

if __name__ == "__main__":
    main()