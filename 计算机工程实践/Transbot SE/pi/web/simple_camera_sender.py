#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import requests
import time
import os

# --- 配置 ---
# 请将 YOUR_SERVER_IP 修改为您Web服务器的实际局域网IP地址
SERVER_URL = "http://192.168.144.121:5000/api/scan_and_upload_order" 

# 摄像头索引号。0通常是默认的内置摄像头。如果机器人有多个摄像头，您可能需要尝试1, 2等。
CAMERA_INDEX = 2

# 发送图片的时间间隔（秒）
SEND_INTERVAL = 0.5 

def main():
    """
    主函数，循环捕捉摄像头图像并发送到服务器。
    """
    # 初始化摄像头
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    if not cap.isOpened():
        print(f"❌ 错误：无法打开索引为 {CAMERA_INDEX} 的摄像头。")
        print("请检查摄像头是否连接正常，或尝试更改 CAMERA_INDEX 的值。")
        return

    print(f"✅ 摄像头 {CAMERA_INDEX} 已成功打开。")
    print(f"将每隔 {SEND_INTERVAL} 秒向 {SERVER_URL} 发送图像。")
    print("按 Ctrl+C 可以停止程序。")

    try:
        while True:
            # 读取一帧图像
            ret, frame = cap.read()
            if not ret:
                print("⚠️ 无法从摄像头读取图像，跳过本次发送。")
                time.sleep(SEND_INTERVAL)
                continue

            # 将图像编码为JPEG格式
            ret, jpeg_image = cv2.imencode('.jpg', frame)
            if not ret:
                print("⚠️ 图像编码为JPEG格式失败。")
                continue
            
            print(f"📸 图像捕捉成功，准备发送...")

            # 发送图像到Web服务器
            try:
                files = {'file': ('image.jpg', jpeg_image.tobytes(), 'image/jpeg')}
                response = requests.post(SERVER_URL, files=files, timeout=10)
                
                # 打印服务器响应
                if response.status_code == 200:
                    response_data = response.json()
                    print(f"✅ 服务器响应: {response_data}")
                    # 检查是否成功扫描到代码并"返回"结果
                    if response_data.get('success') and response_data.get('scanned_codes'):
                        scanned_code = response_data['scanned_codes'][0]
                        print(f"---")
                        print(f"🤖 成功扫描到代码: {scanned_code} 🤖")
                        print(f"---")
                else:
                    print(f"❌ 服务器返回错误，状态码: {response.status_code}, 内容: {response.text}")
            
            except requests.exceptions.RequestException as e:
                print(f"💥 网络请求失败: {e}")

            # 等待指定的时间间隔
            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        print("\n程序被用户中断。正在关闭...")
    finally:
        # 释放摄像头资源
        cap.release()
        cv2.destroyAllWindows()
        print("摄像头已释放，程序退出。")


if __name__ == '__main__':
    main() 