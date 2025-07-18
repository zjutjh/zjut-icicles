#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import subprocess
import threading
from datetime import datetime

class ScanFileMonitor:
    def __init__(self, scan_file_path, video_script_path, end_file_path=None):
        """
        初始化文件监听器
        
        Args:
            scan_file_path: scan.txt文件的路径
            video_script_path: video_to_images.py脚本的路径
            end_file_path: end.txt文件的路径（配送状态文件）
        """
        self.scan_file_path = scan_file_path
        self.video_script_path = video_script_path
        self.image_detector_path = os.path.join(os.path.dirname(video_script_path), "image_detector.py")
        self.qr_detector_path = os.path.join(os.path.dirname(video_script_path), "qr_detector.py")
        self.status_uploader_path = os.path.join(os.path.dirname(video_script_path), "status_uploader.py")
        
        # 配送状态监听
        self.end_file_path = end_file_path
        self.last_end_content = ""
        
        self.last_content = ""
        self.is_running = False
        self.is_detecting = False
        self.is_qr_detecting = False
        self.is_uploading = False
        self.running = True
        
        # 读取初始内容
        self.last_content = self.read_scan_file()
        if self.end_file_path:
            self.last_end_content = self.read_end_file()
        
        print(f"初始化监听器")
        print(f"监听文件: {self.scan_file_path}")
        print(f"视频脚本: {self.video_script_path}")
        print(f"图像检测脚本: {self.image_detector_path}")
        print(f"二维码检测脚本: {self.qr_detector_path}")
        print(f"状态上传脚本: {self.status_uploader_path}")
        if self.end_file_path:
            print(f"配送状态文件: {self.end_file_path}")
            print(f"配送状态初始内容: '{self.last_end_content}'")
        print(f"扫描文件初始内容: '{self.last_content}'")
    
    def read_scan_file(self):
        """读取scan.txt文件内容"""
        try:
            if os.path.exists(self.scan_file_path):
                with open(self.scan_file_path, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                return content
            else:
                print(f"警告: 文件不存在 {self.scan_file_path}")
                return ""
        except Exception as e:
            print(f"读取文件时发生错误: {e}")
            return ""
    
    def read_end_file(self):
        """读取end.txt文件内容（配送状态）"""
        try:
            if self.end_file_path and os.path.exists(self.end_file_path):
                with open(self.end_file_path, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                return content
            else:
                if self.end_file_path:
                    print(f"警告: 配送状态文件不存在 {self.end_file_path}")
                return ""
        except Exception as e:
            print(f"读取配送状态文件时发生错误: {e}")
            return ""
    
    def execute_video_script(self, callback=None):
        """执行视频转图像脚本"""
        if self.is_running:
            print("视频脚本正在运行中，跳过本次执行")
            return
            
        print("=" * 50)
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 开始执行视频转图像脚本")
        print("=" * 50)
        
        self.is_running = True
        
        def run_script():
            try:
                # 使用subprocess运行视频脚本
                result = subprocess.run(
                    ['python3', self.video_script_path],
                    capture_output=True,
                    text=True,
                    cwd=os.path.dirname(self.video_script_path)
                )
                
                print("脚本输出:")
                print(result.stdout)
                
                if result.stderr:
                    print("错误信息:")
                    print(result.stderr)
                
                if result.returncode == 0:
                    print("视频转图像脚本执行成功！")
                    # 如果有回调函数，执行回调
                    if callback:
                        print("视频脚本完成，执行后续操作...")
                        callback()
                else:
                    print(f"视频转图像脚本执行失败，返回码: {result.returncode}")
                    
            except Exception as e:
                print(f"执行视频脚本时发生错误: {e}")
            finally:
                self.is_running = False
                print("视频脚本执行完成")
                print("=" * 50)
        
        # 在单独的线程中运行脚本，避免阻塞监听
        script_thread = threading.Thread(target=run_script)
        script_thread.daemon = True
        script_thread.start()
    
    def execute_image_detector(self):
        """执行图像检测脚本（异步）"""
        if self.is_detecting:
            print("图像检测脚本正在运行中，跳过本次执行")
            return
            
        print("=" * 50)
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 开始执行图像检测脚本")
        print("=" * 50)
        
        self.is_detecting = True
        
        def run_detector():
            try:
                # 使用subprocess运行图像检测脚本
                result = subprocess.run(
                    ['python3', self.image_detector_path],
                    capture_output=True,
                    text=True,
                    cwd=os.path.dirname(self.image_detector_path)
                )
                
                print("检测脚本输出:")
                print(result.stdout)
                
                if result.stderr:
                    print("错误信息:")
                    print(result.stderr)
                
                if result.returncode == 0:
                    print("图像检测脚本执行成功！")
                else:
                    print(f"图像检测脚本执行失败，返回码: {result.returncode}")
                    
            except Exception as e:
                print(f"执行图像检测脚本时发生错误: {e}")
            finally:
                self.is_detecting = False
                print("图像检测脚本执行完成")
                print("=" * 50)
        
        # 在单独的线程中运行脚本，避免阻塞监听
        detector_thread = threading.Thread(target=run_detector)
        detector_thread.daemon = True
        detector_thread.start()
    
    def execute_image_detector_sync(self):
        """执行图像检测脚本（同步，用于回调）"""
        if self.is_detecting:
            print("图像检测脚本正在运行中，跳过本次执行")
            return
            
        print("=" * 50)
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 开始执行图像检测脚本（串行）")
        print("=" * 50)
        
        self.is_detecting = True
        
        try:
            # 使用subprocess运行图像检测脚本
            result = subprocess.run(
                ['python3', self.image_detector_path],
                capture_output=True,
                text=True,
                cwd=os.path.dirname(self.image_detector_path)
            )
            
            print("检测脚本输出:")
            print(result.stdout)
            
            if result.stderr:
                print("错误信息:")
                print(result.stderr)
            
            if result.returncode == 0:
                print("图像检测脚本执行成功！")
            else:
                print(f"图像检测脚本执行失败，返回码: {result.returncode}")
                
        except Exception as e:
            print(f"执行图像检测脚本时发生错误: {e}")
        finally:
            self.is_detecting = False
            print("图像检测脚本执行完成")
            print("=" * 50)
    
    def execute_qr_detector_sync(self):
        """执行二维码检测脚本（同步，用于回调）"""
        if self.is_qr_detecting:
            print("二维码检测脚本正在运行中，跳过本次执行")
            return
            
        print("=" * 50)
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 开始执行二维码检测脚本（串行）")
        print("=" * 50)
        
        self.is_qr_detecting = True
        
        try:
            # 使用subprocess运行二维码检测脚本
            result = subprocess.run(
                ['python3', self.qr_detector_path],
                capture_output=True,
                text=True,
                cwd=os.path.dirname(self.qr_detector_path)
            )
            
            print("二维码检测脚本输出:")
            print(result.stdout)
            
            if result.stderr:
                print("错误信息:")
                print(result.stderr)
            
            if result.returncode == 0:
                print("二维码检测脚本执行成功！")
            else:
                print(f"二维码检测脚本执行失败，返回码: {result.returncode}")
                
        except Exception as e:
            print(f"执行二维码检测脚本时发生错误: {e}")
        finally:
            self.is_qr_detecting = False
            print("二维码检测脚本执行完成")
            print("=" * 50)
    
    def execute_status_uploader(self):
        """执行配送状态上传脚本"""
        if self.is_uploading:
            print("状态上传脚本正在运行中，跳过本次执行")
            return
            
        print("=" * 50)
        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 开始执行配送状态上传")
        print("=" * 50)
        
        self.is_uploading = True
        
        def run_uploader():
            try:
                # 使用subprocess运行状态上传脚本
                result = subprocess.run(
                    ['python3', self.status_uploader_path],
                    capture_output=True,
                    text=True,
                    cwd=os.path.dirname(self.status_uploader_path)
                )
                
                print("状态上传脚本输出:")
                print(result.stdout)
                
                if result.stderr:
                    print("错误信息:")
                    print(result.stderr)
                
                if result.returncode == 0:
                    print("配送状态上传成功！")
                else:
                    print(f"配送状态上传失败，返回码: {result.returncode}")
                    
            except Exception as e:
                print(f"执行状态上传脚本时发生错误: {e}")
            finally:
                self.is_uploading = False
                print("配送状态上传完成")
                print("=" * 50)
        
        # 在单独的线程中运行上传脚本，避免阻塞监听
        uploader_thread = threading.Thread(target=run_uploader)
        uploader_thread.daemon = True
        uploader_thread.start()
    
    def check_end_file_change(self):
        """检查配送状态文件是否发生变化"""
        if not self.end_file_path:
            return
            
        current_end_content = self.read_end_file()
        
        if current_end_content != self.last_end_content:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 检测到配送状态变化:")
            print(f"  旧状态: '{self.last_end_content}'")
            print(f"  新状态: '{current_end_content}'")
            
            self.last_end_content = current_end_content
            
            # 状态映射
            status_mapping = {"0": "正在拾取", "1": "已拾取", "2": "已送达"}
            
            # 当配送状态改变时，上传状态到服务器
            if current_end_content in status_mapping:
                status_text = status_mapping[current_end_content]
                print(f"配送状态更新为: {current_end_content} ({status_text})")
                print("触发配送状态上传...")
                self.execute_status_uploader()
            else:
                print(f"未知的配送状态: '{current_end_content}'")

    def check_file_change(self):
        """检查文件是否发生变化"""
        # 1. 检查scan.txt文件变化
        current_content = self.read_scan_file()
        
        if current_content != self.last_content:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 检测到扫描文件变化:")
            print(f"  旧内容: '{self.last_content}'")
            print(f"  新内容: '{current_content}'")
            
            self.last_content = current_content
            
            # 检查内容并执行相应操作（串行执行）
            if current_content == "1":
                print(f"内容为1，触发视频捕获+二维码检测 (内容: '{current_content}')")
                # 先执行视频捕获，完成后自动执行二维码检测
                self.execute_video_script(callback=self.execute_qr_detector_sync)
            elif current_content == "2":
                print(f"内容为2，触发视频捕获+图像检测 (内容: '{current_content}')")
                # 先执行视频捕获，完成后自动执行图像检测
                self.execute_video_script(callback=self.execute_image_detector_sync)
            elif current_content and current_content != "0":
                print(f"内容非0，触发视频捕获 (内容: '{current_content}')")
                # 其他非0值只执行视频捕获
                self.execute_video_script()
            else:
                print(f"内容为0或空，不执行操作 (内容: '{current_content}')")
        
        # 2. 检查end.txt文件变化（配送状态）
        self.check_end_file_change()
    
    def start_monitoring(self, check_interval=1.0):
        """开始监听文件变化"""
        print(f"开始监听文件变化，检查间隔: {check_interval}秒")
        print("按 Ctrl+C 停止监听...")
        
        try:
            while self.running:
                self.check_file_change()
                time.sleep(check_interval)
        except KeyboardInterrupt:
            print("\n用户中断监听")
        except Exception as e:
            print(f"监听过程中发生错误: {e}")
        finally:
            self.running = False
            print("监听结束")

def main():
    # 文件路径配置
    current_dir = os.path.dirname(os.path.abspath(__file__))
    scan_file_path = "/home/pi/src/transbot_ws/src/transbot_composite_app/temp/scan.txt"
    end_file_path = "/home/pi/src/transbot_ws/src/transbot_composite_app/temp/end.txt"
    video_script_path = os.path.join(current_dir, "video_to_images.py")
    image_detector_path = os.path.join(current_dir, "image_detector.py")
    qr_detector_path = os.path.join(current_dir, "qr_detector.py")
    status_uploader_path = os.path.join(current_dir, "status_uploader.py")
    
    print("=== 多文件监听器 (scan.txt + end.txt) ===")
    print(f"工作目录: {current_dir}")
    
    # 检查文件是否存在
    if not os.path.exists(scan_file_path):
        print(f"错误: scan.txt文件不存在: {scan_file_path}")
        return
    
    if not os.path.exists(end_file_path):
        print(f"错误: end.txt文件不存在: {end_file_path}")
        return
    
    if not os.path.exists(video_script_path):
        print(f"错误: video_to_images.py脚本不存在: {video_script_path}")
        return
    
    if not os.path.exists(image_detector_path):
        print(f"错误: image_detector.py脚本不存在: {image_detector_path}")
        return
    
    if not os.path.exists(qr_detector_path):
        print(f"错误: qr_detector.py脚本不存在: {qr_detector_path}")
        return
    
    if not os.path.exists(status_uploader_path):
        print(f"错误: status_uploader.py脚本不存在: {status_uploader_path}")
        return
    
    # 创建监听器并开始监听
    monitor = ScanFileMonitor(scan_file_path, video_script_path, end_file_path)
    monitor.start_monitoring(check_interval=0.5)  # 0.5秒检查一次

if __name__ == '__main__':
    main() 