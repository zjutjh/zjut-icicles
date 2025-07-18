#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
小车端二维码识别系统 - 单次执行版本
功能：扫描文件夹中的所有图像文件，识别二维码，保存结果，删除文件

配置说明：
直接修改下面的三个参数：
1. SERVER_IP = "192.168.144.121"           # 改为您的Web服务器IP地址
2. IMAGE_FOLDER = "C:/Users/Administrator/Desktop/集成web/primgs"    # 改为您的图像文件夹路径
3. OUTPUT_FOLDER = "C:/Users/Administrator/Desktop/集成web/records"  # 改为您的输出信息文件夹路径

运行方式：
python quick_example.py
"""

import requests
import os
import time

# ========== 配置参数（请修改为实际路径）==========
SERVER_IP = "192.168.144.121"                                        # 服务器IP地址
IMAGE_FOLDER = "C:/Users/Administrator/Desktop/集成web/primgs"        # 图像文件夹路径
OUTPUT_FOLDER = "C:/Users/Administrator/Desktop/集成web/records"      # 输出识别信息文件夹路径

def scan_qr_code(image_path, server_ip=SERVER_IP):
    """
    二维码识别函数（支持新的取餐流程）
    
    Args:
        image_path: 图像文件路径
        server_ip: 服务器IP地址
        
    Returns:
        字典 {
            'scanned_codes': ['A', 'B', 'C'],     # 识别到的二维码
            'added_orders': [...],                # 新添加的订单
            'confirmed_pickups': [...]            # 确认取餐完成的订单
        } 或 None
    """
    try:
        url = f"http://{server_ip}:5000/api/scan_and_upload_order"
        
        with open(image_path, 'rb') as f:
            files = {'file': f}
            response = requests.post(url, files=files, timeout=10)
        
        if response.status_code == 200:
            result = response.json()
            if result.get('success', False):
                return {
                    'scanned_codes': result.get('scanned_codes', []),
                    'added_orders': result.get('added_orders', []),
                    'confirmed_pickups': result.get('confirmed_pickups', []),
                    'ignored_orders': result.get('ignored_orders', []),
                    'message': result.get('message', '')
                }
    
    except Exception as e:
        print(f"识别失败: {e}")
    
    return None

def run_qr_scan():
    """
    执行一次二维码识别任务
    """
    # 确保输出文件夹存在
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    
    print("🤖 小车端二维码识别系统 - 单次执行")
    print(f"📡 服务器地址: {SERVER_IP}")
    print(f"📁 图像文件夹: {IMAGE_FOLDER}")
    print(f"📋 输出文件夹: {OUTPUT_FOLDER}")
    print("=" * 50)
    
    # 获取当前文件夹中的所有图像文件
    image_files = []
    try:
        for filename in os.listdir(IMAGE_FOLDER):
            if filename.lower().endswith(('.jpg', '.jpeg', '.png')):
                image_files.append(filename)
    except FileNotFoundError:
        print(f"❌ 图像文件夹不存在: {IMAGE_FOLDER}")
        return
    
    if not image_files:
        print("📂 文件夹为空，没有图像文件需要处理")
        return
    
    print(f"\n📁 发现 {len(image_files)} 个图像文件，开始处理...")
    
    found_qr = False
    processed_codes = []
    added_orders = []
    confirmed_pickups = []
    ignored_orders = []
    processed_file = None
    server_message = ""
    
    # 逐个处理图像文件，直到识别到二维码
    for filename in image_files:
        image_path = os.path.join(IMAGE_FOLDER, filename)
        
        print(f"🔍 正在处理: {filename}")
        result = scan_qr_code(image_path, SERVER_IP)
        
        if result and result['scanned_codes']:
            codes = result['scanned_codes']
            print(f"✅ 识别成功！发现二维码: {codes}")
            processed_codes.extend(codes)
            processed_file = filename
            server_message = result.get('message', '')
            
            # 处理新添加的订单
            if result['added_orders']:
                added_orders.extend(result['added_orders'])
                print(f"📝 新添加订单数量: {len(result['added_orders'])}")
                for order in result['added_orders']:
                    print(f"   + {order['name']} (订单号: {order['order_id']})")
            
            # 处理确认取餐完成的订单
            if result['confirmed_pickups']:
                confirmed_pickups.extend(result['confirmed_pickups'])
                print(f"✅ 确认取餐完成数量: {len(result['confirmed_pickups'])}")
                for order in result['confirmed_pickups']:
                    print(f"   ✅ {order['name']} (订单号: {order['order_id']}) - 取餐人: {order['student_name']}")
            
            # 处理被忽略的订单
            if result['ignored_orders']:
                ignored_orders.extend(result['ignored_orders'])
                print(f"⚠️ 忽略订单数量: {len(result['ignored_orders'])}")
                for order in result['ignored_orders']:
                    print(f"   ⚠️ {order['name']} (订单号: {order['order_id']}) - {order['reason']}")
            
            # 执行机器人动作（根据新添加的订单或确认取餐的订单）
            for code in codes:
                if code == 'A':
                    if any(o for o in added_orders if o['order_id'] == '15701809'):
                        print("🚀 新订单任务: 收取A楼 - 辣椒炒肉")
                        # your_robot.pickup_order("A楼", "辣椒炒肉")
                    if any(o for o in confirmed_pickups if o['order_id'] == '15701809'):
                        print("🎯 配送完成: A楼 - 辣椒炒肉已送达")
                        # your_robot.delivery_completed("A楼")
                elif code == 'B':
                    if any(o for o in added_orders if o['order_id'] == '50924357'):
                        print("🚀 新订单任务: 收取B楼 - 黄焖鸡米饭")
                        # your_robot.pickup_order("B楼", "黄焖鸡米饭")
                    if any(o for o in confirmed_pickups if o['order_id'] == '50924357'):
                        print("🎯 配送完成: B楼 - 黄焖鸡米饭已送达")
                        # your_robot.delivery_completed("B楼")
                elif code == 'C':
                    if any(o for o in added_orders if o['order_id'] == '11642704'):
                        print("🚀 新订单任务: 收取C楼 - 肯德基")
                        # your_robot.pickup_order("C楼", "肯德基")
                    if any(o for o in confirmed_pickups if o['order_id'] == '11642704'):
                        print("🎯 配送完成: C楼 - 肯德基已送达")
                        # your_robot.delivery_completed("C楼")
            
            found_qr = True
            print("⏹️ 识别到二维码，停止继续扫描")
            break  # 立即停止处理其他文件
        else:
            print(f"❌ 未识别到二维码: {filename}")
    
    # 保存识别结果到输出文件夹
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    result_file = os.path.join(OUTPUT_FOLDER, f"scan_result_{timestamp}.txt")
    
    try:
        with open(result_file, 'w', encoding='utf-8') as f:
            f.write(f"识别时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"处理文件数: {len(image_files)}\n")
            f.write(f"图像文件: {', '.join(image_files)}\n")
            
            if found_qr:
                f.write(f"识别成功: {processed_codes}\n")
                f.write(f"识别文件: {processed_file}\n")
                f.write(f"服务器消息: {server_message}\n")
                
                # 记录新添加的订单
                if added_orders:
                    f.write(f"\n新添加订单 ({len(added_orders)}个):\n")
                    for order in added_orders:
                        f.write(f"  + {order['name']} (订单号: {order['order_id']}, 送达: {order['location']})\n")
                
                # 记录确认取餐完成的订单
                if confirmed_pickups:
                    f.write(f"\n确认取餐完成 ({len(confirmed_pickups)}个):\n")
                    for order in confirmed_pickups:
                        f.write(f"  ✅ {order['name']} (订单号: {order['order_id']}) - 取餐人: {order['student_name']}\n")
                
                # 记录被忽略的订单
                if ignored_orders:
                    f.write(f"\n忽略订单 ({len(ignored_orders)}个):\n")
                    for order in ignored_orders:
                        f.write(f"  ⚠️ {order['name']} (订单号: {order['order_id']}) - {order['reason']}\n")
                
                # 记录执行的机器人任务
                f.write("\n执行的机器人任务:\n")
                for code in processed_codes:
                    if code == 'A':
                        if any(o for o in added_orders if o['order_id'] == '15701809'):
                            f.write("  🚀 新订单任务: 收取A楼 - 辣椒炒肉\n")
                        if any(o for o in confirmed_pickups if o['order_id'] == '15701809'):
                            f.write("  🎯 配送完成: A楼 - 辣椒炒肉已送达\n")
                    elif code == 'B':
                        if any(o for o in added_orders if o['order_id'] == '50924357'):
                            f.write("  🚀 新订单任务: 收取B楼 - 黄焖鸡米饭\n")
                        if any(o for o in confirmed_pickups if o['order_id'] == '50924357'):
                            f.write("  🎯 配送完成: B楼 - 黄焖鸡米饭已送达\n")
                    elif code == 'C':
                        if any(o for o in added_orders if o['order_id'] == '11642704'):
                            f.write("  🚀 新订单任务: 收取C楼 - 肯德基\n")
                        if any(o for o in confirmed_pickups if o['order_id'] == '11642704'):
                            f.write("  🎯 配送完成: C楼 - 肯德基已送达\n")
            else:
                f.write("识别结果: 未发现二维码\n")
            
            f.write(f"\n删除文件: {', '.join(image_files)}\n")
        
        print(f"📋 识别结果已保存到: {result_file}")
    except Exception as e:
        print(f"❌ 保存结果文件失败: {str(e)}")
    
    # 删除文件夹中所有图像文件
    print("\n🗑️ 开始清理所有图像文件...")
    deleted_count = 0
    for filename in image_files:
        image_path = os.path.join(IMAGE_FOLDER, filename)
        try:
            os.remove(image_path)
            deleted_count += 1
            print(f"   ✅ 已删除: {filename}")
        except Exception as e:
            print(f"   ❌ 删除失败: {filename} - {str(e)}")
    
    print(f"🧹 清理完成，共删除 {deleted_count} 个文件")
    
    if found_qr:
        print(f"🎯 识别任务完成！识别到的二维码: {processed_codes}")
        if added_orders:
            print(f"📝 新增订单: {len(added_orders)}个")
        if confirmed_pickups:
            print(f"✅ 完成取餐: {len(confirmed_pickups)}个")
        if ignored_orders:
            print(f"⚠️ 忽略订单: {len(ignored_orders)}个")
    else:
        print("⚠️ 未识别到任何二维码")
    
    print("=" * 50)
    print("🏁 任务执行完成!")

if __name__ == "__main__":
    run_qr_scan() 