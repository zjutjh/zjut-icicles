#!/usr/bin/env python3
# coding: utf-8

from flask import Flask, render_template, jsonify, request, send_file, send_from_directory
import json
import os
import time
import requests
import base64
from datetime import datetime
import logging
import re
import sys
from werkzeug.utils import secure_filename
# 原来的微信二维码扫描模块已替换为增强版模块

# 添加已安装包的路径
sys.path.append(r'c:\users\13181\desktop\pi5\new_venv\lib\site-packages')

# 添加项目根目录到系统路径，以便导入其他模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入语音合成和语音识别模块
from simple_tts import text_to_speech
from voice_recognition_simple import record_and_recognize, recognize_with_proxy

# 导入DeepSeek API客户端
from deepseek_api import ask_deepseek, check_api_key

# 导入增强版二维码扫描模块
from enhanced_qr_scanner import scan_qrcode as enhanced_scan_qrcode

app = Flask(__name__)

# 配置日志
logging.basicConfig(level=logging.INFO)

# 清理markdown格式的函数
def clean_markdown_response(text):
    """清理AI回复中的markdown格式标识符"""
    if not text:
        return text
    
    # 移除markdown格式标识符
    # 移除粗体标记 **text** 或 __text__
    text = re.sub(r'\*\*(.*?)\*\*', r'\1', text)
    text = re.sub(r'__(.*?)__', r'\1', text)
    
    # 移除斜体标记 *text* 或 _text_
    text = re.sub(r'\*(.*?)\*', r'\1', text)
    text = re.sub(r'_(.*?)_', r'\1', text)
    
    # 移除代码块标记 ```text```
    text = re.sub(r'```.*?\n(.*?)\n```', r'\1', text, flags=re.DOTALL)
    text = re.sub(r'`(.*?)`', r'\1', text)
    
    # 移除链接标记 [text](url)
    text = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', text)
    
    # 移除标题标记 # ## ### 等
    text = re.sub(r'^#{1,6}\s*(.*)$', r'\1', text, flags=re.MULTILINE)
    
    # 移除列表标记 - * +
    text = re.sub(r'^[\-\*\+]\s*(.*)$', r'\1', text, flags=re.MULTILINE)
    
    # 移除数字列表标记 1. 2. 等
    text = re.sub(r'^\d+\.\s*(.*)$', r'\1', text, flags=re.MULTILINE)
    
    # 移除引用标记 >
    text = re.sub(r'^>\s*(.*)$', r'\1', text, flags=re.MULTILINE)
    
    # 移除分隔线 --- *** ___
    text = re.sub(r'^[-\*_]{3,}$', '', text, flags=re.MULTILINE)
    
    # 移除表格标记
    text = re.sub(r'\|.*?\|', '', text)
    
    # 移除多余的空行
    text = re.sub(r'\n\s*\n', '\n', text)
    
    # 移除行首行尾空白
    text = text.strip()
    
    return text

# --- 路径配置修正 ---
# 获取app.py文件所在的绝对路径，确保路径引用的正确性
APP_ROOT = os.path.dirname(os.path.abspath(__file__))
# 数据存储路径，指向webapp/data
DATA_DIR = os.path.join(APP_ROOT, "data")
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)

ORDERS_FILE = os.path.join(DATA_DIR, "orders.json")
TAKING_ORDERS_FILE = os.path.join(DATA_DIR, "taking_orders.json")
TAKEN_ORDERS_FILE = os.path.join(DATA_DIR, "taken_orders.json")
DELIVERED_ORDERS_FILE = os.path.join(DATA_DIR, "delivered_orders.json")  # 新增已送达订单文件

# 配置一个专门用于存放机器人上传图片的文件夹
UPLOAD_FOLDER = os.path.join(APP_ROOT, 'robot_uploads') 
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# 确保上传文件夹存在
if not os.path.exists(UPLOAD_FOLDER):
    os.makedirs(UPLOAD_FOLDER)

# 检查DeepSeek API密钥是否有效
try:
    if check_api_key():
        logging.info("DeepSeek API密钥验证成功")
    else:
        logging.error("DeepSeek API密钥验证失败")
except Exception as e:
    logging.error(f"DeepSeek API密钥验证过程中出错: {str(e)}")

# 订单数据映射（基于订单号）
ORDER_DATA = {
    "15701809": {"送达地点": "A楼", "外卖名称": "辣椒炒肉"},
    "50924357": {"送达地点": "B楼", "外卖名称": "黄焖鸡米饭"},
    "11642704": {"送达地点": "C楼", "外卖名称": "肯德基"}
}

# 保留二维码数据映射（供图片扫描功能使用）
QR_CODE_DATA = {
    "A": {"送达地点": "A楼", "外卖名称": "辣椒炒肉", "order_id": "15701809"},
    "B": {"送达地点": "B楼", "外卖名称": "黄焖鸡米饭", "order_id": "50924357"},
    "C": {"送达地点": "C楼", "外卖名称": "肯德基", "order_id": "11642704"}
}

# 初始化数据文件
def initialize_data_files():
    if not os.path.exists(ORDERS_FILE):
        with open(ORDERS_FILE, 'w', encoding='utf-8') as f:
            json.dump([], f, ensure_ascii=False)
    
    if not os.path.exists(TAKING_ORDERS_FILE):
        with open(TAKING_ORDERS_FILE, 'w', encoding='utf-8') as f:
            json.dump([], f, ensure_ascii=False)
    
    if not os.path.exists(TAKEN_ORDERS_FILE):
        with open(TAKEN_ORDERS_FILE, 'w', encoding='utf-8') as f:
            json.dump([], f, ensure_ascii=False)
    
    if not os.path.exists(DELIVERED_ORDERS_FILE):
        with open(DELIVERED_ORDERS_FILE, 'w', encoding='utf-8') as f:
            json.dump([], f, ensure_ascii=False)

initialize_data_files()

# 加载订单数据
def load_orders():
    try:
        with open(ORDERS_FILE, 'r', encoding='utf-8') as f:
            return json.load(f)
    except:
        return []

# 加载正在取餐订单数据
def load_taking_orders():
    try:
        with open(TAKING_ORDERS_FILE, 'r', encoding='utf-8') as f:
            return json.load(f)
    except:
        return []

# 加载已取订单数据
def load_taken_orders():
    try:
        with open(TAKEN_ORDERS_FILE, 'r', encoding='utf-8') as f:
            return json.load(f)
    except:
        return []

# 加载已送达订单数据
def load_delivered_orders():
    try:
        with open(DELIVERED_ORDERS_FILE, 'r', encoding='utf-8') as f:
            return json.load(f)
    except:
        return []

# 保存订单数据
def save_orders(orders):
    with open(ORDERS_FILE, 'w', encoding='utf-8') as f:
        json.dump(orders, f, ensure_ascii=False, indent=2)

# 保存正在取餐订单数据
def save_taking_orders(taking_orders):
    with open(TAKING_ORDERS_FILE, 'w', encoding='utf-8') as f:
        json.dump(taking_orders, f, ensure_ascii=False, indent=2)

# 保存已取订单数据
def save_taken_orders(taken_orders):
    with open(TAKEN_ORDERS_FILE, 'w', encoding='utf-8') as f:
        json.dump(taken_orders, f, ensure_ascii=False, indent=2)

# 保存已送达订单数据
def save_delivered_orders(delivered_orders):
    with open(DELIVERED_ORDERS_FILE, 'w', encoding='utf-8') as f:
        json.dump(delivered_orders, f, ensure_ascii=False, indent=2)

@app.route('/')
def index():
    """渲染主页"""
    return render_template('index.html')

@app.route('/test')
def test_page():
    """渲染测试页面"""
    return render_template('test_chat.html')

@app.route('/api/orders', methods=['GET'])
def get_orders():
    """获取所有当前订单"""
    return jsonify(load_orders())

@app.route('/api/taking_orders', methods=['GET'])
def get_taking_orders():
    """获取所有正在取餐订单"""
    return jsonify(load_taking_orders())

@app.route('/api/taken_orders', methods=['GET'])
def get_taken_orders():
    """获取所有已取订单"""
    return jsonify(load_taken_orders())

@app.route('/api/delivered_orders', methods=['GET'])
def get_delivered_orders():
    """获取所有已送达订单"""
    return jsonify(load_delivered_orders())

@app.route('/api/upload_order', methods=['POST'])
def upload_order():
    """上传新订单（由小车调用）"""
    data = request.json
    if not data or 'order_id' not in data or 'name' not in data:
        return jsonify({"status": "error", "message": "缺少必要数据"}), 400
    
    orders = load_orders()
    
    # 检查订单是否已存在
    for order in orders:
        if order['order_id'] == data['order_id']:
            return jsonify({"status": "error", "message": "订单已存在"}), 400
    
    # 添加时间戳
    data['timestamp'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    orders.append(data)
    
    save_orders(orders)
    return jsonify({"status": "success", "message": "订单上传成功"})

@app.route('/api/take_order', methods=['POST'])
def take_order():
    """开始取餐流程（由用户在网页上操作，订单转移到正在取餐状态）"""
    data = request.json
    if not data or 'order_id' not in data or 'student_id' not in data or 'student_name' not in data:
        return jsonify({"status": "error", "message": "缺少必要数据"}), 400
    
    orders = load_orders()
    taking_orders = load_taking_orders()
    
    # 查找要取走的订单
    order_to_take = None
    for i, order in enumerate(orders):
        if order['order_id'] == data['order_id']:
            order_to_take = order
            del orders[i]
            break
    
    if order_to_take is None:
        return jsonify({"status": "error", "message": "未找到该订单"}), 404
    
    # 添加取餐人信息和开始取餐时间
    order_to_take['student_id'] = data['student_id']
    order_to_take['student_name'] = data['student_name']
    order_to_take['taking_start_timestamp'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    # 移动到正在取餐列表
    taking_orders.append(order_to_take)
    
    save_orders(orders)
    save_taking_orders(taking_orders)
    
    return jsonify({"status": "success", "message": "开始取餐流程，请等待小车配送"})

@app.route('/api/confirm_pickup', methods=['POST'])
def confirm_pickup():
    """确认取餐完成（由小车端调用，将订单从正在取餐转移到已取餐）"""
    data = request.json
    if not data or 'order_id' not in data:
        return jsonify({"status": "error", "message": "缺少订单编号"}), 400
    
    order_id = data['order_id']
    taking_orders = load_taking_orders()
    taken_orders = load_taken_orders()
    
    # 查找正在取餐的订单
    order_to_confirm = None
    for i, order in enumerate(taking_orders):
        if order['order_id'] == order_id:
            order_to_confirm = order
            del taking_orders[i]
            break
    
    if order_to_confirm is None:
        return jsonify({"status": "error", "message": "未找到该正在取餐的订单"}), 404
    
    # 添加取餐完成时间
    order_to_confirm['taken_timestamp'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    # 移动到已取餐列表
    taken_orders.append(order_to_confirm)
    
    save_taking_orders(taking_orders)
    save_taken_orders(taken_orders)
    
    return jsonify({
        "status": "success", 
        "message": "取餐确认成功",
        "order": order_to_confirm
    })

# 配送状态处理函数（仅处理送达状态）
def handle_delivered_status(order_id, food_item, timestamp, robot_id):
    """处理已送达状态（状态码2）- 将订单从配送中移动到已送达"""
    try:
        taken_orders = load_taken_orders()  # 配送中订单
        delivered_orders = load_delivered_orders()  # 已送达订单
        
        # 查找配送中的订单
        order_to_deliver = None
        for i, order in enumerate(taken_orders):
            if order['order_id'] == order_id:
                order_to_deliver = order
                del taken_orders[i]
                break
        
        if order_to_deliver is None:
            # 如果在配送中找不到，可能订单不存在或已经送达
            # 检查是否已经在送达列表中
            for order in delivered_orders:
                if order['order_id'] == order_id:
                    return {
                        "success": True, 
                        "message": f"订单 {order_id}({food_item}) 已经在送达列表中",
                        "order": order
                    }
            
            return {"success": False, "message": f"未找到配送中的订单 {order_id}，可能订单不存在或状态异常"}
        
        # 更新送达信息
        order_to_deliver['delivered_timestamp'] = timestamp
        order_to_deliver['delivery_status'] = "已送达"
        order_to_deliver['robot_id'] = robot_id  # 更新配送机器人信息
        
        # 移动到已送达列表
        delivered_orders.append(order_to_deliver)
        
        save_taken_orders(taken_orders)
        save_delivered_orders(delivered_orders)
        
        logging.info(f"✅ 订单 {order_id}({food_item}) 送达完成，由机器人 {robot_id} 配送")
        
        return {
            "success": True, 
            "message": f"订单 {order_id}({food_item}) 送达完成",
            "order": order_to_deliver
        }
        
    except Exception as e:
        logging.error(f"处理送达状态时出错: {str(e)}")
        return {"success": False, "message": f"处理送达状态时出错: {str(e)}"}

@app.route('/api/delivery_status', methods=['POST'])
def receive_delivery_status():
    """接收小车配送状态信息"""
    try:
        data = request.json
        if not data:
            return jsonify({"success": False, "message": "缺少请求数据"}), 400
        
        # 检查必要字段
        required_fields = ['food_item', 'status_code']
        for field in required_fields:
            if field not in data:
                return jsonify({"success": False, "message": f"缺少必要字段: {field}"}), 400
        
        food_item = data['food_item'].upper()  # A, B, C
        status_code = str(data['status_code'])  # 0, 1, 2
        timestamp = data.get('timestamp', datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        robot_id = data.get('robot_id', 'unknown')
        
        logging.info(f"收到配送状态更新: 餐品={food_item}, 状态={status_code}, 机器人={robot_id}")
        
        # 根据餐品编号找到对应订单ID
        if food_item not in QR_CODE_DATA:
            return jsonify({"success": False, "message": f"无效的餐品编号: {food_item}"}), 400
        
        order_id = QR_CODE_DATA[food_item]['order_id']
        
        # 只处理送达消息（状态码2）
        if status_code == "2":
            # 已送达：将订单从配送中移动到已送达状态
            result = handle_delivered_status(order_id, food_item, timestamp, robot_id)
        else:
            # 忽略其他状态码，记录日志但返回成功
            logging.info(f"忽略状态码 {status_code}（非送达状态），餐品={food_item}, 机器人={robot_id}")
            return jsonify({
                "success": True, 
                "message": f"状态码 {status_code} 已收到但未处理（仅处理送达状态）"
            }), 200
        
        if result['success']:
            logging.info(f"配送状态更新成功: {result['message']}")
            return jsonify(result), 200
        else:
            logging.error(f"配送状态更新失败: {result['message']}")
            return jsonify(result), 400
        
    except Exception as e:
        logging.error(f"处理配送状态时发生错误: {str(e)}")
        return jsonify({"success": False, "message": f"服务器内部错误: {str(e)}"}), 500

@app.route('/api/chat', methods=['POST'])
def chat():
    """统一的智能对话功能 - 支持文本和语音输入"""
    data = request.json
    if not data:
        return jsonify({"status": "error", "message": "缺少必要数据"}), 400
    
    # 支持两种输入方式：直接文本输入 或 语音输入标识
    user_message = ""
    if 'message' in data:
        user_message = data['message']
    elif 'use_voice_input' in data and data['use_voice_input']:
        # 语音输入模式
        try:
            text, confidence = record_and_recognize()
            if text:
                user_message = text
                logging.info(f"语音识别结果: {text}, 置信度: {confidence}")
            else:
                return jsonify({"status": "error", "message": "语音识别失败，请重试"}), 400
        except Exception as e:
            logging.error(f"语音识别错误: {str(e)}")
            return jsonify({"status": "error", "message": f"语音识别错误: {str(e)}"}), 500
    else:
        return jsonify({"status": "error", "message": "缺少输入内容"}), 400
    
    if not user_message.strip():
        return jsonify({"status": "error", "message": "输入内容不能为空"}), 400
    
    orders = load_orders()
    taking_orders = load_taking_orders()
    taken_orders = load_taken_orders()
    response_text = ""
    
    # 第二层：优先处理订单查询 (查询特定订单号或餐名的外卖是否到达)
    
    # 首先检测订单号
    order_id_match = re.search(r'订单编号是?(\d+)', user_message) or re.search(r'(\d{6,})', user_message)
    if order_id_match:
        order_id = order_id_match.group(1)
        
        # 检查待取订单
        for order in orders:
            if order['order_id'] == order_id:
                response_text = f"您的订单编号{order_id}（{order['name']}）已到达，可以前来取餐了。该订单上传时间是{order['timestamp']}。"
                break
        
        # 如果待取订单中没找到，检查正在取餐订单
        if not response_text:
            for order in taking_orders:
                if order['order_id'] == order_id:
                    response_text = f"您的订单编号{order_id}（{order['name']}）正在取餐中，取餐人是{order['student_name']}（学号:{order['student_id']}），于{order['taking_start_timestamp']}开始取餐。请耐心等待配送完成。"
                    break
        
        # 如果还没找到，检查配送订单
        if not response_text:
            for order in taken_orders:
                if order['order_id'] == order_id:
                    response_text = f"您的订单编号{order_id}（{order['name']}）正在配送中，配送给{order['student_name']}（学号:{order['student_id']}），于{order['taken_timestamp']}开始配送。"
                    break
        
        # 如果都没找到
        if not response_text:
            response_text = f"抱歉，未找到订单编号{order_id}的相关信息。请确认订单号是否正确。"
    
    # 如果没有匹配到订单号，则检测餐名
    if not response_text:
        # 获取所有可能的餐名（从三个状态的订单中）
        all_food_names = set()
        for order in orders:
            all_food_names.add(order['name'])
        for order in taking_orders:
            all_food_names.add(order['name'])
        for order in taken_orders:
            all_food_names.add(order['name'])
        
        # 检查用户输入是否包含任何餐名
        matched_food_name = None
        for food_name in all_food_names:
            if food_name in user_message:
                matched_food_name = food_name
                break
        
        if matched_food_name:
            # 在三个状态的订单中查找匹配的餐名
            waiting_orders = []
            taking_order_info = []
            taken_order_info = []
            
            for order in orders:
                if order['name'] == matched_food_name:
                    waiting_orders.append(order)
            
            for order in taking_orders:
                if order['name'] == matched_food_name:
                    taking_order_info.append(order)
            
            for order in taken_orders:
                if order['name'] == matched_food_name:
                    taken_order_info.append(order)
            
            # 生成回复
            response_parts = []
            if waiting_orders or taking_order_info or taken_order_info:
                response_parts.append(f"关于{matched_food_name}的订单信息：")
                
                # 处理待取订单
                if waiting_orders:
                    if len(waiting_orders) == 1:
                        order = waiting_orders[0]
                        response_parts.append(f"有1份{matched_food_name}已到达（订单号：{order['order_id']}），可以前来取餐。")
                    else:
                        response_parts.append(f"有{len(waiting_orders)}份{matched_food_name}已到达，订单号分别是：{', '.join([order['order_id'] for order in waiting_orders])}。")
                
                # 处理正在取餐订单
                if taking_order_info:
                    if len(taking_order_info) == 1:
                        order = taking_order_info[0]
                        response_parts.append(f"有1份{matched_food_name}（订单号：{order['order_id']}）正在取餐中，取餐人是{order['student_name']}，请耐心等待配送完成。")
                    else:
                        response_parts.append(f"有{len(taking_order_info)}份{matched_food_name}正在取餐中。")
                
                # 处理配送订单
                if taken_order_info:
                    if len(taken_order_info) == 1:
                        order = taken_order_info[0]
                        response_parts.append(f"有1份{matched_food_name}（订单号：{order['order_id']}）正在配送中，配送给{order['student_name']}，于{order['taken_timestamp']}开始配送。")
                    else:
                        response_parts.append(f"有{len(taken_order_info)}份{matched_food_name}正在配送中。")
                
                response_text = " ".join(response_parts)
    
    # 第一层和第三层：使用DeepSeek API处理
    # 包括特定问题（模型相关）和其他所有问题
    if not response_text:
        try:
            logging.info(f"正在调用DeepSeek API，请求内容: {user_message[:50]}...")
            start_time = time.time()
            raw_response = ask_deepseek(user_message)
            processing_time = time.time() - start_time
            logging.info(f"DeepSeek API响应成功，响应长度: {len(raw_response)}，处理时间: {processing_time:.2f}秒")
            
            # 检查是否返回了错误消息
            if raw_response.startswith("网络错误:") or raw_response.startswith("解析响应失败:") or raw_response.startswith("处理请求时出错:") or raw_response.startswith("API调用失败"):
                logging.error(f"DeepSeek API返回错误: {raw_response}")
                return jsonify({
                    "status": "error",
                    "message": raw_response
                }), 500
            
            # 清理markdown格式
            response_text = clean_markdown_response(raw_response)
            logging.info(f"清理后响应长度: {len(response_text)}")
            
        except Exception as e:
            logging.error(f"DeepSeek API调用错误: {str(e)}")
            response_text = "抱歉，我遇到了一些问题，无法回答您的问题。请稍后再试。"
    
    # 生成语音回复（所有回答都提供语音版本）
    audio_url = None
    try:
        timestamp = int(time.time())
        audio_file = f'static/audio/response_{timestamp}.mp3'
        output_path = os.path.join(os.path.dirname(__file__), audio_file)
        
        # 确保音频目录存在
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        text_to_speech(response_text, output_file=output_path, voice_type="1")
        audio_url = f'/{audio_file}'
        logging.info(f"语音合成成功: {audio_url}")
    except Exception as e:
        logging.error(f"语音合成错误: {str(e)}")
        audio_url = None
    
    return jsonify({
        "status": "success", 
        "response": response_text,
        "audio_url": audio_url,
        "input_method": "voice" if 'use_voice_input' in data and data['use_voice_input'] else "text",
        "recognized_text": user_message if 'use_voice_input' in data and data['use_voice_input'] else None
    })

@app.route('/api/qrcode', methods=['POST'])
def process_qrcode():
    """处理订单上传（基于订单号）"""
    data = request.json
    if not data or 'code' not in data:
        return jsonify({"status": "error", "message": "缺少必要数据"}), 400
    
    order_id = data['code'].strip()
    
    # 检查订单号是否在预设数据中
    if order_id not in ORDER_DATA:
        return jsonify({"status": "error", "message": "无效的订单号，请输入正确的订单编号"}), 400
    
    order_data = ORDER_DATA[order_id]
    
    # 准备订单数据
    new_order = {
        "order_id": order_id,
        "name": order_data["外卖名称"],
        "location": order_data["送达地点"],
        "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    }
    
    orders = load_orders()
    
    # 检查订单是否已存在
    for order in orders:
        if order['order_id'] == new_order['order_id']:
            return jsonify({"status": "error", "message": "订单已存在"}), 400
    
    orders.append(new_order)
    save_orders(orders)
    
    response_text = f"已上传订单：{new_order['name']}，送达地点：{new_order['location']}，订单编号：{new_order['order_id']}"
    
    return jsonify({
        "status": "success", 
        "message": "订单上传成功",
        "order": new_order,
        "response_text": response_text
    })

@app.route('/api/voice_recognition', methods=['POST'])
def voice_recognition_api():
    """独立的语音识别API"""
    try:
        # 实时录音并识别
        text, confidence = record_and_recognize()
        
        if text:
            return jsonify({
                "status": "success",
                "text": text,
                "confidence": confidence
            })
        else:
            return jsonify({"status": "error", "message": "无法识别语音"}), 400
    
    except Exception as e:
        logging.error(f"语音识别错误: {str(e)}")
        return jsonify({"status": "error", "message": f"语音识别错误: {str(e)}"}), 500

@app.route('/api/text_to_speech', methods=['POST'])
def text_to_speech_api():
    """独立的文字转语音API"""
    try:
        data = request.json
        if not data or 'text' not in data:
            return jsonify({"status": "error", "message": "缺少必要数据"}), 400
        
        text = data['text']
        voice_type = data.get('voice_type', '1')  # 默认使用男声
        
        # 生成唯一的文件名
        timestamp = int(time.time())
        output_file = os.path.join(os.path.dirname(__file__), f'static/audio/tts_{timestamp}.mp3')
        
        # 确保音频目录存在
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        
        # 调用语音合成
        text_to_speech(text, output_file=output_file, voice_type=voice_type)
        
        # 返回音频文件的URL
        audio_url = f'/static/audio/tts_{timestamp}.mp3'
        return jsonify({
            "status": "success",
            "audio_url": audio_url
        })
    
    except Exception as e:
        logging.error(f"语音合成错误: {str(e)}")
        return jsonify({"status": "error", "message": f"语音合成错误: {str(e)}"}), 500

@app.route('/api/ai_assistant', methods=['POST'])
def ai_assistant():
    """统一的AI助手接口 - 前端友好版本"""
    data = request.json
    if not data:
        return jsonify({"status": "error", "message": "缺少必要数据"}), 400
    
    input_type = data.get('input_type', 'text')  # 'text' 或 'voice'
    user_message = data.get('message', '')
    
    if input_type == 'voice':
        # 使用语音输入
        try:
            text, confidence = record_and_recognize()
            if text:
                user_message = text
                logging.info(f"语音识别结果: {text}, 置信度: {confidence}")
            else:
                return jsonify({"status": "error", "message": "语音识别失败，请重试"}), 400
        except Exception as e:
            logging.error(f"语音识别错误: {str(e)}")
            return jsonify({"status": "error", "message": f"语音识别错误: {str(e)}"}), 500
    
    if not user_message.strip():
        return jsonify({"status": "error", "message": "输入内容不能为空"}), 400
    
    # 调用统一的chat处理逻辑
    mock_request_data = {'message': user_message}
    
    # 直接调用chat函数的核心逻辑
    orders = load_orders()
    taking_orders = load_taking_orders()
    taken_orders = load_taken_orders()
    response_text = ""
    
    # 订单查询处理 (支持订单号和餐名查询)
    
    # 首先检测订单号
    order_id_match = re.search(r'订单编号是?(\d+)', user_message) or re.search(r'(\d{6,})', user_message)
    if order_id_match:
        order_id = order_id_match.group(1)
        
        # 检查待取订单
        for order in orders:
            if order['order_id'] == order_id:
                response_text = f"您的订单编号{order_id}（{order['name']}）已到达，可以前来取餐了。该订单上传时间是{order['timestamp']}。"
                break
        
        # 如果待取订单中没找到，检查正在取餐订单
        if not response_text:
            for order in taking_orders:
                if order['order_id'] == order_id:
                    response_text = f"您的订单编号{order_id}（{order['name']}）正在取餐中，取餐人是{order['student_name']}（学号:{order['student_id']}），于{order['taking_start_timestamp']}开始取餐。请耐心等待配送完成。"
                    break
        
        # 如果还没找到，检查配送订单
        if not response_text:
            for order in taken_orders:
                if order['order_id'] == order_id:
                    response_text = f"您的订单编号{order_id}（{order['name']}）正在配送中，配送给{order['student_name']}（学号:{order['student_id']}），于{order['taken_timestamp']}开始配送。"
                    break
        
        # 如果都没找到
        if not response_text:
            response_text = f"抱歉，未找到订单编号{order_id}的相关信息。请确认订单号是否正确。"
    
    # 如果没有匹配到订单号，则检测餐名
    if not response_text:
        # 获取所有可能的餐名（从三个状态的订单中）
        all_food_names = set()
        for order in orders:
            all_food_names.add(order['name'])
        for order in taking_orders:
            all_food_names.add(order['name'])
        for order in taken_orders:
            all_food_names.add(order['name'])
        
        # 检查用户输入是否包含任何餐名
        matched_food_name = None
        for food_name in all_food_names:
            if food_name in user_message:
                matched_food_name = food_name
                break
        
        if matched_food_name:
            # 在三个状态的订单中查找匹配的餐名
            waiting_orders = []
            taking_order_info = []
            taken_order_info = []
            
            for order in orders:
                if order['name'] == matched_food_name:
                    waiting_orders.append(order)
            
            for order in taking_orders:
                if order['name'] == matched_food_name:
                    taking_order_info.append(order)
            
            for order in taken_orders:
                if order['name'] == matched_food_name:
                    taken_order_info.append(order)
            
            # 生成回复
            response_parts = []
            if waiting_orders or taking_order_info or taken_order_info:
                response_parts.append(f"关于{matched_food_name}的订单信息：")
                
                # 处理待取订单
                if waiting_orders:
                    if len(waiting_orders) == 1:
                        order = waiting_orders[0]
                        response_parts.append(f"有1份{matched_food_name}已到达（订单号：{order['order_id']}），可以前来取餐。")
                    else:
                        response_parts.append(f"有{len(waiting_orders)}份{matched_food_name}已到达，订单号分别是：{', '.join([order['order_id'] for order in waiting_orders])}。")
                
                # 处理正在取餐订单
                if taking_order_info:
                    if len(taking_order_info) == 1:
                        order = taking_order_info[0]
                        response_parts.append(f"有1份{matched_food_name}（订单号：{order['order_id']}）正在取餐中，取餐人是{order['student_name']}，请耐心等待配送完成。")
                    else:
                        response_parts.append(f"有{len(taking_order_info)}份{matched_food_name}正在取餐中。")
                
                # 处理配送订单
                if taken_order_info:
                    if len(taken_order_info) == 1:
                        order = taken_order_info[0]
                        response_parts.append(f"有1份{matched_food_name}（订单号：{order['order_id']}）正在配送中，配送给{order['student_name']}，于{order['taken_timestamp']}开始配送。")
                    else:
                        response_parts.append(f"有{len(taken_order_info)}份{matched_food_name}正在配送中。")
                
                response_text = " ".join(response_parts)
    
    # DeepSeek API处理
    if not response_text:
        try:
            logging.info(f"正在调用DeepSeek API，请求内容: {user_message[:50]}...")
            raw_response = ask_deepseek(user_message)
            
            if raw_response.startswith("网络错误:") or raw_response.startswith("解析响应失败:") or raw_response.startswith("处理请求时出错:") or raw_response.startswith("API调用失败"):
                return jsonify({"status": "error", "message": raw_response}), 500
            
            # 清理markdown格式
            response_text = clean_markdown_response(raw_response)
            
        except Exception as e:
            logging.error(f"DeepSeek API调用错误: {str(e)}")
            response_text = "抱歉，我遇到了一些问题，无法回答您的问题。请稍后再试。"
    
    # 生成语音回复
    audio_url = None
    try:
        timestamp = int(time.time())
        audio_file = f'static/audio/response_{timestamp}.mp3'
        output_path = os.path.join(os.path.dirname(__file__), audio_file)
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        text_to_speech(response_text, output_file=output_path, voice_type="1")
        audio_url = f'/{audio_file}'
    except Exception as e:
        logging.error(f"语音合成错误: {str(e)}")
    
    return jsonify({
        "status": "success",
        "response": response_text,
        "audio_url": audio_url,
        "input_type": input_type,
        "recognized_text": user_message if input_type == 'voice' else None
    })

@app.route('/api/scan_and_upload_order', methods=['POST'])
def scan_and_upload_order():
    if 'file' not in request.files:
        return jsonify({"success": False, "message": "未找到上传的文件"}), 400
    
    file = request.files['file']
    if not file or not file.filename:
        return jsonify({"success": False, "message": "未选择文件或文件名为空"}), 400

    filename = secure_filename(file.filename)
    filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    file.save(filepath)

    try:
        # 调用增强版二维码扫描模块
        qr_results = enhanced_scan_qrcode(filepath)
    finally:
        # 确保处理后删除临时图片
        if os.path.exists(filepath):
            os.remove(filepath)

    if not qr_results:
        return jsonify({"success": False, "message": "未识别到二维码"}), 200

    # 加载当前订单数据
    current_orders = load_orders()  # 待取订单
    taking_orders = load_taking_orders()  # 正在取餐订单
    taken_orders = load_taken_orders()  # 已取订单
    
    # 构建所有已存在订单的ID集合（包括三个状态的所有订单）
    waiting_order_ids = {order.get('order_id') for order in current_orders}
    taking_order_ids = {order.get('order_id') for order in taking_orders}
    taken_order_ids = {order.get('order_id') for order in taken_orders}
    all_existing_order_ids = waiting_order_ids | taking_order_ids | taken_order_ids
    
    new_orders_added = []
    confirmed_pickups = []
    ignored_orders = []  # 记录被忽略的订单（已存在但不在正在取餐状态）
    successful_codes = []
    
    for code in qr_results:
        if code in QR_CODE_DATA:
            successful_codes.append(code)
            order_info = QR_CODE_DATA[code]
            order_id = order_info['order_id']
            
            # 第一优先级：检查正在取餐列表，如果有匹配的订单则完成取餐确认
            pickup_confirmed = False
            for i, taking_order in enumerate(taking_orders):
                if taking_order['order_id'] == order_id:
                    # 找到正在取餐的订单，完成取餐确认
                    confirmed_order = taking_orders.pop(i)
                    confirmed_order['taken_timestamp'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    taken_orders.append(confirmed_order)
                    confirmed_pickups.append(confirmed_order)
                    pickup_confirmed = True
                    break
            
            # 第二优先级：如果没有在正在取餐列表中找到，检查是否已存在于任何列表中
            if not pickup_confirmed:
                if order_id in all_existing_order_ids:
                    # 订单已存在于某个列表中（待取或已取），忽略此次扫描
                    ignored_orders.append({
                        "order_id": order_id,
                        "name": order_info['外卖名称'],
                        "reason": "订单已存在（不在正在取餐状态）"
                    })
                else:
                    # 订单在所有列表中都不存在，添加到待取列表
                    new_order = {
                        "order_id": order_id,
                        "name": order_info['外卖名称'],
                        "location": order_info['送达地点'],
                        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    }
                    current_orders.append(new_order)
                    all_existing_order_ids.add(order_id)  # 更新已存在ID集合，防止本次重复添加
                    new_orders_added.append(new_order)

    # 保存所有变更的数据
    if new_orders_added:
        save_orders(current_orders)
    if confirmed_pickups:
        save_taking_orders(taking_orders)
        save_taken_orders(taken_orders)
    
    # 构建返回消息
    messages = []
    if new_orders_added:
        messages.append(f"成功添加 {len(new_orders_added)} 个新订单")
    if confirmed_pickups:
        messages.append(f"成功确认 {len(confirmed_pickups)} 个取餐完成")
    if ignored_orders:
        messages.append(f"忽略 {len(ignored_orders)} 个已存在的订单")
    
    # 如果有任何操作结果，返回详细信息
    if messages or successful_codes:
        return jsonify({
            "success": True, 
            "message": "；".join(messages) if messages else "扫描完成，无需操作",
            "added_orders": new_orders_added,
            "confirmed_pickups": confirmed_pickups,
            "ignored_orders": ignored_orders,
            "scanned_codes": successful_codes
        }), 200
    else:
        return jsonify({"success": False, "message": "扫描到的二维码不在A,B,C之列"}), 200

if __name__ == '__main__':
    # 确保音频目录存在
    os.makedirs(os.path.join(os.path.dirname(__file__), 'static/audio'), exist_ok=True)
    
    # 确保监听所有网络接口，允许局域网内的其他设备访问
    app.run(host='0.0.0.0', port=5000, debug=True) 