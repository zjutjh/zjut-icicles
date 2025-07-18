#!/usr/bin/env python3
# coding: utf-8

import cv2
import json
import requests
import time

"""
此脚本用于在Transbot机器人上实现二维码扫描功能
需要将其部署到机器人上运行
"""

try:
    from Transbot_Lib import Transbot
    ROBOT_MODE = True
except ImportError:
    print("警告：未找到Transbot_Lib库，将以模拟模式运行")
    ROBOT_MODE = False

# Web平台地址
WEB_SERVER_URL = "http://192.168.144.17:5000/api/upload_order"

class QRCodeScanner:
    def __init__(self, robot=None):
        self.robot = robot
        self.detector = cv2.QRCodeDetector()
        # 尝试打开摄像头
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("错误：无法打开摄像头!")
                self.cap_opened = False
            else:
                self.cap_opened = True
        except Exception as e:
            print(f"初始化摄像头时出错: {e}")
            self.cap_opened = False
        
    def scan(self):
        """扫描二维码"""
        if not self.cap_opened:
            print("摄像头未打开，无法扫描")
            return None
            
        ret, frame = self.cap.read()
        if not ret:
            print("无法读取视频帧")
            return None
        
        # 显示摄像头图像（用于调试）
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)
        
        # 检测二维码
        data, bbox, _ = self.detector.detectAndDecode(frame)
        
        if data:
            try:
                # 解析二维码数据
                qr_data = json.loads(data)
                if 'order_id' in qr_data and 'name' in qr_data:
                    # 识别到二维码时绘制边框
                    if bbox is not None:
                        # 转换边界框为整数点
                        bbox = bbox.astype(int)
                        # 绘制边框
                        cv2.polylines(frame, [bbox], True, (0, 255, 0), 2)
                        cv2.imshow("QR Code Detected", frame)
                        cv2.waitKey(1000)  # 显示1秒
                    return qr_data
            except json.JSONDecodeError:
                print(f"无法解析二维码数据: {data}")
                pass
            except Exception as e:
                print(f"处理二维码时发生错误: {e}")
        
        return None
    
    def upload_to_server(self, data):
        """上传到Web服务器"""
        headers = {"Content-Type": "application/json"}
        
        try:
            response = requests.post(WEB_SERVER_URL, json=data, headers=headers)
            if response.status_code == 200:
                result = response.json()
                print(f"上传成功: {result['message']}")
                return True
            else:
                print(f"上传失败: HTTP状态码 {response.status_code}")
                print(response.text)
                return False
        except Exception as e:
            print(f"上传过程中发生错误: {str(e)}")
            return False
    
    def beep(self):
        """蜂鸣器提示音"""
        if self.robot and ROBOT_MODE:
            self.robot.set_beep(50)  # 蜂鸣器响50ms
        else:
            print("模拟蜂鸣器: 嘟...")
    
    def close(self):
        """释放资源"""
        if self.cap_opened:
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    # 初始化机器人
    robot = None
    if ROBOT_MODE:
        try:
            robot = Transbot()
            robot.create_receive_threading()
            print("成功初始化Transbot机器人")
        except Exception as e:
            print(f"初始化机器人失败: {e}")
    
    # 初始化扫描器
    scanner = QRCodeScanner(robot)
    
    try:
        print("-" * 50)
        print("开始扫描外卖二维码...")
        print(f"Web服务器地址: {WEB_SERVER_URL}")
        print("按Ctrl+C可以停止程序")
        print("-" * 50)
        
        while True:
            # 扫描二维码
            qr_data = scanner.scan()
            
            if qr_data:
                print(f"\n识别到订单: {qr_data}")
                
                # 上传到服务器
                if scanner.upload_to_server(qr_data):
                    # 提示音反馈
                    scanner.beep()
                    
                    # 等待一段时间再继续扫描，避免重复上传
                    time.sleep(3)
            
            # 短暂延时，减少CPU使用率
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        print("关闭摄像头和释放资源...")
        scanner.close()

if __name__ == "__main__":
    main() 