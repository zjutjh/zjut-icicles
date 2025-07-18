#!/usr/bin/env python3
# encoding: utf-8
"""
小车配送状态上传模块
用于向服务器上传配送状态信息
优化版本：包含重试机制、更好的错误处理和配置管理
"""

import requests
import os
import json
import time
from datetime import datetime
import logging

class StatusUploader:
    def __init__(self, config_file=None):
        """初始化状态上传器"""
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 配置文件路径（小车端需要根据实际路径调整）
        self.a_b_c_file = "/home/pi/src/transbot_ws/src/transbot_composite_app/temp_in/a_b_c.txt"
        self.end_file = "/home/pi/src/transbot_ws/src/transbot_composite_app/temp/end.txt"
        
        # 服务器配置 - 根据实际服务器IP调整
        self.server_ip = "192.168.144.121"  # 🔧 请根据实际服务器IP修改
        self.server_port = 5000
        self.upload_url = f"http://{self.server_ip}:{self.server_port}/api/delivery_status"
        
        # 重试配置
        self.max_retries = 3
        self.retry_delay = 2  # 秒
        self.timeout = 10     # 请求超时时间
        
        # 机器人ID - 可以根据实际机器人编号修改
        self.robot_id = "transbot_01"
        
        # 状态映射
        self.status_mapping = {
            "0": "正在拾取",
            "1": "已拾取", 
            "2": "已送达"
        }
        
        # 餐品编号到订单ID的映射（与服务器端保持一致）
        self.food_to_order_mapping = {
            "a": "15701809",  # A楼 - 辣椒炒肉
            "b": "50924357",  # B楼 - 黄焖鸡米饭
            "c": "11642704"   # C楼 - 肯德基
        }
        
        # 设置日志
        self.setup_logging()
    
    def setup_logging(self):
        """设置日志记录"""
        log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        logging.basicConfig(
            level=logging.INFO,
            format=log_format,
            handlers=[
                logging.FileHandler(f'{self.current_dir}/upload_status.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger('StatusUploader')
    
    def read_current_food_item(self):
        """读取当前配送的餐品编号"""
        try:
            if os.path.exists(self.a_b_c_file):
                with open(self.a_b_c_file, 'r', encoding='utf-8') as f:
                    content = f.read().strip().lower()
                    # 确保是有效的餐品编号
                    if content in ['a', 'b', 'c']:
                        self.logger.info(f"读取到餐品编号: {content.upper()}")
                        return content
            
            self.logger.warning(f"无法读取有效的餐品编号，使用默认值 'a'")
            return 'a'  # 默认值
        except Exception as e:
            self.logger.error(f"读取餐品编号时发生错误: {e}")
            return 'a'
    
    def read_delivery_status(self):
        """读取当前配送状态"""
        try:
            if os.path.exists(self.end_file):
                with open(self.end_file, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                    if content in ['0', '1', '2']:
                        self.logger.info(f"读取到配送状态: {content} ({self.status_mapping.get(content, '未知')})")
                        return content
            
            self.logger.warning(f"无法读取有效的配送状态，使用默认值 '0'")
            return "0"  # 默认状态
        except Exception as e:
            self.logger.error(f"读取配送状态时发生错误: {e}")
            return "0"
    
    def check_server_connection(self):
        """检查服务器连接"""
        try:
            # 先尝试ping服务器的健康检查端点
            health_url = f"http://{self.server_ip}:{self.server_port}/"
            response = requests.get(health_url, timeout=5)
            if response.status_code == 200:
                self.logger.info("✅ 服务器连接正常")
                return True
            else:
                self.logger.warning(f"⚠️ 服务器响应异常: HTTP {response.status_code}")
                return False
        except Exception as e:
            self.logger.error(f"❌ 无法连接到服务器: {e}")
            return False
    
    def upload_status_with_retry(self, food_item, status_code):
        """带重试机制的状态上传"""
        for attempt in range(1, self.max_retries + 1):
            self.logger.info(f"尝试上传状态 (第{attempt}/{self.max_retries}次)...")
            
            success = self.upload_status(food_item, status_code)
            if success:
                return True
            
            if attempt < self.max_retries:
                self.logger.info(f"等待 {self.retry_delay} 秒后重试...")
                time.sleep(self.retry_delay)
                self.retry_delay *= 1.5  # 指数退避
        
        self.logger.error(f"❌ 经过 {self.max_retries} 次尝试后仍然失败")
        return False
    
    def upload_status(self, food_item, status_code):
        """上传配送状态到服务器"""
        try:
            # 准备上传数据
            upload_data = {
                "food_item": food_item.upper(),  # A, B, C
                "status_code": status_code,      # 0, 1, 2
                "status_text": self.status_mapping.get(status_code, "未知状态"),
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "robot_id": self.robot_id
            }
            
            self.logger.info(f"准备上传数据: {upload_data}")
            
            # 发送POST请求
            headers = {
                'Content-Type': 'application/json',
                'User-Agent': 'TransBot-DeliverySystem/2.0',
                'Accept': 'application/json'
            }
            
            response = requests.post(
                self.upload_url, 
                json=upload_data,
                headers=headers,
                timeout=self.timeout
            )
            
            self.logger.info(f"HTTP响应状态码: {response.status_code}")
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success', False):
                    self.logger.info(f"✅ 状态上传成功!")
                    self.logger.info(f"   餐品: {food_item.upper()}")
                    self.logger.info(f"   状态: {self.status_mapping.get(status_code, '未知')}")
                    self.logger.info(f"   服务器响应: {result.get('message', '')}")
                    
                    # 保存成功上传的记录
                    self.save_upload_record(upload_data, result)
                    return True
                else:
                    self.logger.error(f"❌ 服务器处理失败: {result.get('message', '未知错误')}")
                    return False
            else:
                self.logger.error(f"❌ HTTP错误: {response.status_code}")
                self.logger.error(f"   响应内容: {response.text}")
                return False
                
        except requests.exceptions.Timeout:
            self.logger.error("❌ 上传超时: 服务器响应时间过长")
            return False
        except requests.exceptions.ConnectionError:
            self.logger.error(f"❌ 连接错误: 无法连接到服务器 {self.server_ip}:{self.server_port}")
            return False
        except requests.exceptions.RequestException as e:
            self.logger.error(f"❌ 请求错误: {e}")
            return False
        except Exception as e:
            self.logger.error(f"❌ 上传过程中发生未知错误: {e}")
            return False
    
    def save_upload_record(self, upload_data, server_response):
        """保存上传记录到本地文件"""
        try:
            record = {
                "timestamp": datetime.now().isoformat(),
                "upload_data": upload_data,
                "server_response": server_response,
                "status": "success"
            }
            
            record_file = f"{self.current_dir}/upload_records.json"
            
            # 读取现有记录
            records = []
            if os.path.exists(record_file):
                try:
                    with open(record_file, 'r', encoding='utf-8') as f:
                        records = json.load(f)
                except:
                    records = []
            
            # 添加新记录
            records.append(record)
            
            # 只保留最近100条记录
            if len(records) > 100:
                records = records[-100:]
            
            # 保存到文件
            with open(record_file, 'w', encoding='utf-8') as f:
                json.dump(records, f, ensure_ascii=False, indent=2)
            
            self.logger.debug(f"上传记录已保存到: {record_file}")
            
        except Exception as e:
            self.logger.warning(f"保存上传记录时出错: {e}")
    
    def get_order_id_for_food(self, food_item):
        """根据餐品编号获取订单ID"""
        return self.food_to_order_mapping.get(food_item.lower(), "unknown")
    
    def run_upload(self):
        """执行状态上传任务"""
        self.logger.info("=" * 60)
        self.logger.info(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 开始配送状态上传")
        self.logger.info("=" * 60)
        
        try:
            # 0. 检查服务器连接
            if not self.check_server_connection():
                self.logger.error("服务器连接失败，终止上传")
                return False
            
            # 1. 读取当前餐品编号
            food_item = self.read_current_food_item()
            order_id = self.get_order_id_for_food(food_item)
            self.logger.info(f"当前餐品编号: {food_item.upper()} (订单ID: {order_id})")
            
            # 2. 读取配送状态
            status_code = self.read_delivery_status()
            status_text = self.status_mapping.get(status_code, "未知状态")
            self.logger.info(f"当前配送状态: {status_code} ({status_text})")
            
            # 3. 上传到服务器（带重试机制）
            success = self.upload_status_with_retry(food_item, status_code)
            
            self.logger.info("=" * 60)
            if success:
                self.logger.info("配送状态上传完成!")
            else:
                self.logger.error("配送状态上传失败!")
            self.logger.info("=" * 60)
            
            return success
            
        except Exception as e:
            self.logger.error(f"上传流程中发生错误: {e}")
            return False

def main():
    """主函数"""
    print("=== 配送状态上传脚本 v2.0 ===")
    
    uploader = StatusUploader()
    success = uploader.run_upload()
    
    if success:
        print("脚本执行成功！")
        exit(0)
    else:
        print("脚本执行失败！")
        exit(1)

if __name__ == '__main__':
    main() 