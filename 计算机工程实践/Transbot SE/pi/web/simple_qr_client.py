#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import requests
import os
import time

class SimpleQRClient:
    """简化的二维码识别客户端"""
    
    def __init__(self, server_ip="192.168.144.121", server_port=5000):
        """初始化客户端"""
        self.server_url = f"http://{server_ip}:{server_port}/api/scan_and_upload_order"
        print(f"🤖 连接服务器: {self.server_url}")
    
    def scan_image(self, image_path):
        """
        发送图像到服务器进行二维码识别
        
        Args:
            image_path: 图像文件路径
            
        Returns:
            识别到的二维码列表，如果失败返回None
        """
        try:
            with open(image_path, 'rb') as f:
                files = {'file': f}
                response = requests.post(self.server_url, files=files, timeout=10)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success', False):
                    codes = result.get('scanned_codes', [])
                    print(f"✅ 识别成功: {codes}")
                    return codes
                else:
                    print(f"❌ 识别失败: {result.get('message', '未知错误')}")
                    return None
            else:
                print(f"❌ 服务器错误: {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ 连接错误: {str(e)}")
            return None
    
    def scan_folder(self, folder_path):
        """
        扫描文件夹中的所有图像
        
        Args:
            folder_path: 图像文件夹路径
            
        Returns:
            所有识别结果的字典 {文件名: [识别码列表]}
        """
        results = {}
        
        # 支持的图像格式
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
        
        for filename in os.listdir(folder_path):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                image_path = os.path.join(folder_path, filename)
                print(f"\n📸 处理图像: {filename}")
                
                codes = self.scan_image(image_path)
                results[filename] = codes
                
                # 稍微等待一下，避免服务器压力过大
                time.sleep(0.1)
        
        return results

def main():
    """简单的使用示例"""
    # 创建客户端（修改IP地址为您的服务器IP）
    client = SimpleQRClient(server_ip="192.168.144.121")
    
    print("🤖 简化版二维码识别客户端")
    print("=" * 40)
    
    # 方式1: 识别单个图像文件
    # codes = client.scan_image("path/to/your/image.jpg")
    # if codes:
    #     print(f"识别到: {codes}")
    
    # 方式2: 识别整个文件夹
    # results = client.scan_folder("path/to/your/image/folder")
    # for filename, codes in results.items():
    #     if codes:
    #         print(f"{filename}: {codes}")
    
    # 交互式使用
    while True:
        print("\n选择操作:")
        print("1. 识别单个图像文件")
        print("2. 识别文件夹中所有图像") 
        print("3. 退出")
        
        choice = input("请输入选择 (1-3): ").strip()
        
        if choice == "1":
            image_path = input("请输入图像文件路径: ").strip()
            if os.path.exists(image_path):
                codes = client.scan_image(image_path)
                if codes:
                    print(f"✅ 识别结果: {codes}")
                    # 在这里添加您的处理逻辑
                    for code in codes:
                        if code == 'A':
                            print("🚀 执行A楼任务")
                        elif code == 'B':
                            print("🚀 执行B楼任务")
                        elif code == 'C':
                            print("🚀 执行C楼任务")
            else:
                print("❌ 文件不存在")
                
        elif choice == "2":
            folder_path = input("请输入图像文件夹路径: ").strip()
            if os.path.exists(folder_path):
                results = client.scan_folder(folder_path)
                print(f"\n📊 处理完成，共处理 {len(results)} 个文件")
                for filename, codes in results.items():
                    if codes:
                        print(f"✅ {filename}: {codes}")
            else:
                print("❌ 文件夹不存在")
                
        elif choice == "3":
            print("👋 退出")
            break
        else:
            print("❌ 无效选择")

if __name__ == "__main__":
    main() 