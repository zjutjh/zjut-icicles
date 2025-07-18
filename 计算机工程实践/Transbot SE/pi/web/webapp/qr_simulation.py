#!/usr/bin/env python3
# coding: utf-8

import requests
import json
import time
import random

"""
这个脚本模拟Transbot机器人识别外卖二维码并上传信息到Web平台
可以用来测试Web平台的功能
"""

# Web平台地址
WEB_SERVER_URL = "http://192.168.144.17:5000"

# 模拟订单数据
SAMPLE_FOODS = [
    "辣椒炒肉",
    "宫保鸡丁",
    "水煮鱼",
    "糖醋排骨",
    "麻婆豆腐",
    "红烧肉",
    "鱼香肉丝",
    "西红柿炒鸡蛋",
    "回锅肉",
    "酸菜鱼",
    "小炒肉",
    "干锅牛肉",
    "干锅土豆片",
    "蛋炒饭",
    "扬州炒饭",
]

def generate_order_id():
    """生成随机订单ID"""
    return ''.join(random.choices('0123456789', k=9))

def simulate_qr_recognition():
    """模拟识别二维码"""
    food_name = random.choice(SAMPLE_FOODS)
    order_id = generate_order_id()
    
    print(f"模拟识别到二维码: 订单号={order_id}, 外卖名称={food_name}")
    return {
        "order_id": order_id,
        "name": food_name
    }

def upload_to_web_server(order_data):
    """上传订单数据到Web服务器"""
    try:
        url = f"{WEB_SERVER_URL}/api/upload_order"
        headers = {"Content-Type": "application/json"}
        
        # 添加连接超时和重试机制
        for attempt in range(3):
            try:
                response = requests.post(url, json=order_data, headers=headers, timeout=5)
                
                if response.status_code == 200:
                    result = response.json()
                    print(f"上传成功: {result['message']}")
                    return True
                else:
                    print(f"上传失败: HTTP状态码 {response.status_code}")
                    print(response.text)
                    return False
            except requests.exceptions.RequestException as e:
                print(f"尝试 {attempt+1}/3 失败: {str(e)}")
                if attempt < 2:  # 如果不是最后一次尝试
                    print("等待2秒后重试...")
                    time.sleep(2)
                else:
                    print("所有重试失败")
                    return False
    except Exception as e:
        print(f"上传过程中发生错误: {str(e)}")
        return False

def main():
    """主函数"""
    print("开始模拟外卖二维码识别...")
    print(f"Web服务器地址: {WEB_SERVER_URL}")
    print("-" * 50)
    
    try:
        # 模拟多次识别
        for i in range(3):
            print(f"\n第 {i+1} 次识别:")
            order_data = simulate_qr_recognition()
            upload_to_web_server(order_data)
            time.sleep(2)  # 等待2秒
            
    except KeyboardInterrupt:
        print("\n模拟被用户中断")
    
    print("-" * 50)
    print("模拟结束")

if __name__ == "__main__":
    main() 