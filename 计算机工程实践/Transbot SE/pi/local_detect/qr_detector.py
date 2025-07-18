#!/usr/bin/env python3
# encoding: utf-8
import requests
import os
import time
from datetime import datetime

class QRDetector:
    def __init__(self):
        """初始化二维码检测器"""
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.captured_images_dir = os.path.join(self.current_dir, "captured_images")
        self.result_file = "/home/pi/src/transbot_ws/src/transbot_composite_app/temp_in/a_b_c.txt"
        
        # 服务器配置（需要根据实际情况修改）
        self.server_ip = "192.168.144.121"  # 请根据实际服务器IP修改
        self.server_url = f"http://{self.server_ip}:5000/api/scan_and_upload_order"
    
    def scan_qr_code(self, image_path):
        """
        二维码识别函数
        
        Args:
            image_path: 图像文件路径
            
        Returns:
            识别到的二维码列表，如 ['A', 'B'] 或 None
        """
        try:
            with open(image_path, 'rb') as f:
                files = {'file': f}
                response = requests.post(self.server_url, files=files, timeout=10)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success', False):
                    scanned_codes = result.get('scanned_codes', [])
                    return scanned_codes
        
        except Exception as e:
            print(f"识别失败 {image_path}: {e}")
        
        return None
    
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
    
    def extract_valid_codes(self, all_codes):
        """提取有效的二维码（A、B、C）"""
        valid_codes = []
        for code in all_codes:
            code_upper = str(code).upper().strip()
            if code_upper in ['A', 'B', 'C']:
                if code_upper not in valid_codes:  # 避免重复
                    valid_codes.append(code_upper)
        return valid_codes
    
    def save_result(self, qr_codes):
        """保存二维码识别结果到文件"""
        try:
            # 确保目录存在
            result_dir = os.path.dirname(self.result_file)
            os.makedirs(result_dir, exist_ok=True)
            
            # 确定要保存的值
            if qr_codes:
                # 如果识别到多个码，选择第一个，如果只有一个就用那个
                result_value = qr_codes[0].lower()  # 转为小写
            else:
                # 默认值
                result_value = "a"
            
            # 写入结果（覆盖模式）
            with open(self.result_file, 'w', encoding='utf-8') as f:
                f.write(result_value)
            
            print(f"二维码识别结果已保存到: {self.result_file}")
            print(f"结果内容: {result_value}")
            
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
        """运行完整的二维码识别流程"""
        print("=" * 60)
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 开始二维码识别流程")
        print("=" * 60)
        
        try:
            # 1. 获取图像文件列表
            image_files = self.get_image_files()
            if not image_files:
                print("没有找到图像文件，使用默认结果")
                self.save_result([])
                return True
            
            print(f"找到 {len(image_files)} 个图像文件")
            
            # 2. 逐个识别图像，直到找到二维码或处理完所有图像
            all_found_codes = []
            processed_files = 0
            
            for i, image_path in enumerate(image_files):
                print(f"正在处理图像 {i+1}/{len(image_files)}: {os.path.basename(image_path)}")
                qr_codes = self.scan_qr_code(image_path)
                processed_files += 1
                
                if qr_codes:
                    valid_codes = self.extract_valid_codes(qr_codes)
                    if valid_codes:
                        print(f"  识别到二维码: {valid_codes}")
                        all_found_codes.extend(valid_codes)
                        # 找到有效二维码后立即停止
                        print("找到有效二维码，停止继续扫描")
                        break
                    else:
                        print(f"  识别到二维码但不是A/B/C: {qr_codes}")
                else:
                    print("  未识别到二维码")
            
            # 3. 保存识别结果
            unique_codes = []
            for code in all_found_codes:
                if code not in unique_codes:
                    unique_codes.append(code)
            
            print(f"最终识别结果: {unique_codes if unique_codes else '无有效二维码'}")
            self.save_result(unique_codes)
            
            # 4. 清空图像目录
            self.clear_captured_images()
            
            print("=" * 60)
            print("二维码识别流程完成！")
            print("=" * 60)
            
            return True
            
        except Exception as e:
            print(f"识别流程中发生错误: {e}")
            # 即使出错也保存默认值
            self.save_result([])
            return False

def main():
    """主函数"""
    print("=== 二维码识别与分析脚本 ===")
    
    detector = QRDetector()
    success = detector.run_detection()
    
    if success:
        print("脚本执行成功！")
    else:
        print("脚本执行失败！")

if __name__ == '__main__':
    main() 