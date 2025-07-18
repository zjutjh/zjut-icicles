#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import shutil
from collections import Counter
from datetime import datetime
from PIL import Image
import tensorflow as tf

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

class ImageDetector:
    def __init__(self):
        """初始化图像检测器"""
        self.yolo = None
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.captured_images_dir = os.path.join(self.current_dir, "captured_images")
        self.result_file = "/home/pi/src/transbot_ws/src/transbot_composite_app/temp_in/left_right_fire_cons.txt"
        
        # YOLO模型配置
        self.model_config = {
            'model_path': os.path.join(YOLO_PATH, 'logs/last1.weights.h5'),
            'classes_path': os.path.join(YOLO_PATH, 'model_data/CampusBot.txt'),
            'anchors_path': os.path.join(YOLO_PATH, 'model_data/CampusBot_anchors.txt'),
            'score': 0.5,
            'iou': 0.3
        }
    
    def initialize_yolo(self):
        """初始化YOLO模型"""
        if self.yolo is None:
            print("正在初始化YOLO模型...")
            
            # 配置GPU内存增长
            gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
            for gpu in gpus:
                tf.config.experimental.set_memory_growth(gpu, True)
            
            print(f"模型路径: {self.model_config['model_path']}")
            print(f"类别文件: {self.model_config['classes_path']}")
            print(f"锚框文件: {self.model_config['anchors_path']}")
            
            # 创建YOLO实例
            self.yolo = YOLO(
                model_path=self.model_config['model_path'],
                classes_path=self.model_config['classes_path'],
                anchors_path=self.model_config['anchors_path'],
                score=self.model_config['score'],
                iou=self.model_config['iou']
            )
            print("YOLO模型初始化完成")
    
    def detect_single_image(self, image_path):
        """检测单张图像"""
        try:
            # 打开图像
            image = Image.open(image_path)
            
            # 执行检测
            r_image, boxes, scores, classes = self.yolo.detect_image(image)
            
            # 构建检测结果
            detections = []
            for i in range(len(boxes)):
                cls = int(classes[i])
                class_name = self.yolo.class_names[cls]
                score = float(scores[i])
                
                detections.append({
                    "class": class_name,
                    "confidence": score,
                    "box": boxes[i].tolist()
                })
            
            return detections
        
        except Exception as e:
            print(f"处理图像 {image_path} 时发生错误: {e}")
            return []
    
    def get_image_files(self):
        """获取captured_images目录中的所有图像文件"""
        if not os.path.exists(self.captured_images_dir):
            print(f"警告: 图像目录不存在 {self.captured_images_dir}")
            return []
        
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.gif']
        image_files = []
        
        for file in os.listdir(self.captured_images_dir):
            ext = os.path.splitext(file)[1].lower()
            if ext in image_extensions:
                image_files.append(os.path.join(self.captured_images_dir, file))
        
        return sorted(image_files)
    
    def analyze_detections(self, all_detections):
        """分析所有检测结果，统计各类别数量"""
        class_counts = Counter()
        
        for detections in all_detections:
            for detection in detections:
                class_counts[detection['class']] += 1
        
        return class_counts
    
    def determine_result_class(self, class_counts):
        """确定最终结果类别"""
        if not class_counts:
            print("没有检测到任何目标，使用默认类别: turn_left")
            return "turn_left"
        
        # 获取数量最多的类别
        most_common_class = class_counts.most_common(1)[0][0]
        print(f"检测统计结果: {dict(class_counts)}")
        print(f"数量最多的类别: {most_common_class}")
        
        return most_common_class
    
    def save_result(self, result_class):
        """保存结果到文件"""
        try:
            # 确保目录存在
            result_dir = os.path.dirname(self.result_file)
            os.makedirs(result_dir, exist_ok=True)
            
            # 写入结果（覆盖模式）
            with open(self.result_file, 'w', encoding='utf-8') as f:
                f.write(result_class)
            
            print(f"检测结果已保存到: {self.result_file}")
            print(f"结果内容: {result_class}")
            
        except Exception as e:
            print(f"保存结果时发生错误: {e}")
    
    def clear_captured_images(self):
        """清空captured_images目录"""
        try:
            if os.path.exists(self.captured_images_dir):
                # 删除目录下的所有文件
                for file in os.listdir(self.captured_images_dir):
                    file_path = os.path.join(self.captured_images_dir, file)
                    if os.path.isfile(file_path):
                        os.remove(file_path)
                        print(f"删除文件: {file}")
                print(f"已清空目录: {self.captured_images_dir}")
            else:
                print(f"目录不存在，无需清空: {self.captured_images_dir}")
        
        except Exception as e:
            print(f"清空目录时发生错误: {e}")
    
    def run_detection(self):
        """运行完整的检测流程"""
        print("=" * 60)
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 开始图像检测流程")
        print("=" * 60)
        
        try:
            # 1. 初始化YOLO模型
            self.initialize_yolo()
            
            # 2. 获取图像文件列表
            image_files = self.get_image_files()
            if not image_files:
                print("没有找到图像文件，使用默认结果")
                self.save_result("turn_left")
                return True
            
            print(f"找到 {len(image_files)} 个图像文件")
            
            # 3. 逐个检测图像
            all_detections = []
            for i, image_path in enumerate(image_files):
                print(f"正在处理图像 {i+1}/{len(image_files)}: {os.path.basename(image_path)}")
                detections = self.detect_single_image(image_path)
                all_detections.append(detections)
                
                if detections:
                    print(f"  检测到 {len(detections)} 个目标: {[d['class'] for d in detections]}")
                else:
                    print("  未检测到目标")
            
            # 4. 分析检测结果
            class_counts = self.analyze_detections(all_detections)
            
            # 5. 确定最终结果
            result_class = self.determine_result_class(class_counts)
            
            # 6. 保存结果
            self.save_result(result_class)
            
            # 7. 清空图像目录
            self.clear_captured_images()
            
            print("=" * 60)
            print("图像检测流程完成！")
            print("=" * 60)
            
            return True
            
        except Exception as e:
            print(f"检测流程中发生错误: {e}")
            return False

def main():
    """主函数"""
    print("=== 图像检测与分析脚本 ===")
    
    detector = ImageDetector()
    success = detector.run_detection()
    
    if success:
        print("脚本执行成功！")
    else:
        print("脚本执行失败！")

if __name__ == '__main__':
    main() 