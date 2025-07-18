#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
系统监控脚本：监控GUI运行状态，帮助诊断崩溃问题
"""

import os
import time
import subprocess
import psutil
import signal
import sys

class SystemMonitor:
    def __init__(self):
        self.running = True
        self.gui_pid = None
        self.linefollow_pid = None
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        print("=== System Monitor Started ===")
        print("Monitoring GUI and line follow processes...")
    
    def signal_handler(self, sig, frame):
        print("\n=== Received shutdown signal ===")
        self.running = False
        sys.exit(0)
    
    def find_processes(self):
        """查找相关进程"""
        gui_processes = []
        linefollow_processes = []
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                
                # 查找GUI进程
                if 'gui_server.py' in cmdline or 'python3' in proc.info['name'] and 'gui' in cmdline:
                    gui_processes.append(proc)
                
                # 查找巡线相关进程  
                if 'follow_line.launch' in cmdline or 'LineDetect' in cmdline:
                    linefollow_processes.append(proc)
                    
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        return gui_processes, linefollow_processes
    
    def check_system_resources(self):
        """检查系统资源"""
        # 内存使用率
        memory = psutil.virtual_memory()
        memory_percent = memory.percent
        memory_available = memory.available / (1024*1024)  # MB
        
        # CPU使用率
        cpu_percent = psutil.cpu_percent(interval=1)
        
        # 磁盘使用率
        disk = psutil.disk_usage('/')
        disk_percent = disk.percent
        
        return {
            'memory_percent': memory_percent,
            'memory_available_mb': memory_available,
            'cpu_percent': cpu_percent,
            'disk_percent': disk_percent
        }
    
    def check_ros_status(self):
        """检查ROS状态"""
        try:
            # 检查roscore是否运行
            result = subprocess.run(['pgrep', '-f', 'roscore'], 
                                 capture_output=True, text=True, timeout=5)
            roscore_running = bool(result.stdout.strip())
            
            # 检查ROS话题
            result = subprocess.run(['rostopic', 'list'], 
                                 capture_output=True, text=True, timeout=5)
            topic_count = len(result.stdout.strip().split('\n')) if result.stdout.strip() else 0
            
            return {
                'roscore_running': roscore_running,
                'topic_count': topic_count
            }
        except Exception as e:
            return {
                'roscore_running': False,
                'topic_count': 0,
                'error': str(e)
            }
    
    def monitor_loop(self):
        """主监控循环"""
        last_gui_count = 0
        last_linefollow_count = 0
        crash_detected = False
        
        while self.running:
            try:
                # 查找进程
                gui_procs, linefollow_procs = self.find_processes()
                
                # 检查系统资源
                resources = self.check_system_resources()
                
                # 检查ROS状态
                ros_status = self.check_ros_status()
                
                # 当前时间
                current_time = time.strftime("%H:%M:%S")
                
                # 检测崩溃
                if len(gui_procs) < last_gui_count and last_gui_count > 0:
                    print("[{}] [CRASH DETECTED] GUI process crashed!".format(current_time))
                    crash_detected = True
                
                if len(linefollow_procs) < last_linefollow_count and last_linefollow_count > 0:
                    print("[{}] [CRASH DETECTED] Line follow process crashed!".format(current_time))
                    crash_detected = True
                
                # 输出状态信息
                status_line = "[{}] GUI:{} LineFollow:{} MEM:{:.1f}% CPU:{:.1f}% ROS:{}".format(
                    current_time,
                    len(gui_procs),
                    len(linefollow_procs), 
                    resources['memory_percent'],
                    resources['cpu_percent'],
                    "OK" if ros_status['roscore_running'] else "ERROR"
                )
                
                print(status_line)
                
                # 资源警告
                if resources['memory_percent'] > 90:
                    print("[{}] [WARNING] High memory usage: {:.1f}%".format(
                        current_time, resources['memory_percent']))
                
                if resources['memory_available_mb'] < 100:
                    print("[{}] [WARNING] Low available memory: {:.1f}MB".format(
                        current_time, resources['memory_available_mb']))
                
                if resources['cpu_percent'] > 90:
                    print("[{}] [WARNING] High CPU usage: {:.1f}%".format(
                        current_time, resources['cpu_percent']))
                
                # 如果检测到崩溃，收集详细信息
                if crash_detected:
                    print("[{}] [INFO] Collecting crash information...".format(current_time))
                    self.collect_crash_info()
                    crash_detected = False
                
                # 更新计数
                last_gui_count = len(gui_procs)
                last_linefollow_count = len(linefollow_procs)
                
                time.sleep(2)  # 每2秒检查一次
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print("[{}] [ERROR] Monitor error: {}".format(
                    time.strftime("%H:%M:%S"), e))
                time.sleep(2)
    
    def collect_crash_info(self):
        """收集崩溃信息"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        try:
            # 检查核心转储
            core_files = []
            for root, dirs, files in os.walk('/home/pi/src/transbot_ws'):
                for file in files:
                    if file.startswith('core'):
                        core_files.append(os.path.join(root, file))
            
            if core_files:
                print("[INFO] Found core files: {}".format(core_files))
            
            # 检查系统日志
            try:
                result = subprocess.run(['dmesg', '|', 'tail', '-20'], 
                                     shell=True, capture_output=True, text=True, timeout=10)
                if result.stdout:
                    print("[INFO] Recent system messages:")
                    print(result.stdout)
            except:
                pass
            
            # 检查ROS日志
            try:
                result = subprocess.run(['rosnode', 'list'], 
                                     capture_output=True, text=True, timeout=5)
                print("[INFO] Active ROS nodes: {}".format(result.stdout.strip()))
            except:
                pass
        
        except Exception as e:
            print("[ERROR] Failed to collect crash info: {}".format(e))

def main():
    try:
        monitor = SystemMonitor()
        monitor.monitor_loop()
        
    except KeyboardInterrupt:
        print("\nMonitor stopped by user")
    except Exception as e:
        print("Monitor crashed: {}".format(e))

if __name__ == '__main__':
    main() 