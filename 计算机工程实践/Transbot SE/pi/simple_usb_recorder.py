#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单USB麦克风录音程序
适用于树莓派arm64系统
设备：UACDemoV1.0 USB Audio (hw:2,0)
使用arecord命令，避免PyAudio的配置问题
"""

import subprocess
import os
import datetime
import time
import signal
import sys

class SimpleUSBRecorder:
    def __init__(self):
        # 音频设备参数
        self.DEVICE = "hw:2,0"  # USB音频设备
        self.RATE = 48000       # 采样率
        self.CHANNELS = 1       # 单声道
        self.FORMAT = "S16_LE"  # 16位PCM格式
        
        # 输出目录
        self.output_dir = "recordings"
        self.create_output_directory()
        
        # 当前录音进程
        self.recording_process = None
        self.is_recording = False
        
    def create_output_directory(self):
        """创建录音文件输出目录"""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"创建录音目录: {self.output_dir}")
    
    def test_device(self):
        """测试音频设备是否可用"""
        print("测试USB麦克风设备...")
        
        # 生成测试文件名
        test_file = os.path.join(self.output_dir, "test_recording.wav")
        
        # 构建arecord命令
        cmd = [
            "arecord",
            "-D", self.DEVICE,
            "-f", self.FORMAT,
            "-c", str(self.CHANNELS),
            "-r", str(self.RATE),
            "-d", "3",  # 录制3秒
            test_file
        ]
        
        try:
            print("正在录制3秒测试音频...")
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                if os.path.exists(test_file):
                    file_size = os.path.getsize(test_file)
                    print(f"✓ 设备测试成功！")
                    print(f"测试文件: {test_file}")
                    print(f"文件大小: {file_size / 1024:.1f} KB")
                    return True
                else:
                    print("✗ 测试失败：未生成录音文件")
                    return False
            else:
                print(f"✗ 录音命令执行失败:")
                print(f"错误信息: {result.stderr}")
                return False
                
        except Exception as e:
            print(f"✗ 设备测试出错: {e}")
            return False
    
    def start_recording(self, duration=None, filename=None):
        """开始录音"""
        if self.is_recording:
            print("正在录音中，请先停止当前录音")
            return False
        
        # 生成文件名
        if filename is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.wav"
        
        if not filename.endswith('.wav'):
            filename += '.wav'
        
        output_file = os.path.join(self.output_dir, filename)
        
        # 构建arecord命令
        cmd = [
            "arecord",
            "-D", self.DEVICE,
            "-f", self.FORMAT,
            "-c", str(self.CHANNELS),
            "-r", str(self.RATE)
        ]
        
        # 如果指定了时长，添加-d参数
        if duration:
            cmd.extend(["-d", str(duration)])
        
        cmd.append(output_file)
        
        try:
            print(f"开始录音...")
            print(f"设备: {self.DEVICE} (UACDemoV1.0)")
            print(f"格式: {self.CHANNELS}通道, {self.RATE}Hz, {self.FORMAT}")
            print(f"输出文件: {output_file}")
            
            if duration:
                print(f"录音时长: {duration}秒")
            else:
                print("手动停止录音: 按 Ctrl+C")
            
            print("-" * 50)
            
            # 启动录音进程
            self.recording_process = subprocess.Popen(cmd)
            self.is_recording = True
            
            if duration:
                # 等待指定时长
                print(f"录音中...")
                for i in range(int(duration)):
                    if not self.is_recording:
                        break
                    remaining = duration - i
                    print(f"\r进度: {i+1}/{int(duration)}秒 (剩余: {remaining-1}秒)", end='', flush=True)
                    time.sleep(1)
                
                # 等待进程结束
                self.recording_process.wait()
                print(f"\n录音完成！")
            else:
                # 手动停止模式
                try:
                    start_time = time.time()
                    while self.is_recording:
                        elapsed = time.time() - start_time
                        print(f"\r录音中... {elapsed:.1f}秒", end='', flush=True)
                        time.sleep(0.1)
                        
                        # 检查进程是否还在运行
                        if self.recording_process.poll() is not None:
                            break
                    
                    print(f"\n录音完成！")
                    
                except KeyboardInterrupt:
                    print(f"\n\n录音被用户中断")
                    self.stop_recording()
            
            self.is_recording = False
            
            # 检查录音文件
            if os.path.exists(output_file):
                file_size = os.path.getsize(output_file)
                print(f"录音已保存: {output_file}")
                print(f"文件大小: {file_size / 1024:.1f} KB")
                return True
            else:
                print("录音文件未生成")
                return False
                
        except Exception as e:
            print(f"录音过程出错: {e}")
            self.stop_recording()
            return False
    
    def stop_recording(self):
        """停止录音"""
        if self.recording_process and self.recording_process.poll() is None:
            try:
                self.recording_process.terminate()
                self.recording_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.recording_process.kill()
                self.recording_process.wait()
            except Exception as e:
                print(f"停止录音时出错: {e}")
        
        self.is_recording = False
        self.recording_process = None
    
    def list_devices(self):
        """列出可用的音频设备"""
        print("=== 可用音频设备 (arecord -l) ===")
        try:
            result = subprocess.run(["arecord", "-l"], capture_output=True, text=True)
            if result.returncode == 0:
                print(result.stdout)
            else:
                print("无法获取设备列表")
        except Exception as e:
            print(f"列出设备时出错: {e}")

def main():
    """主函数"""
    print("=" * 60)
    print("简单USB麦克风录音程序")
    print("适用于树莓派 - UACDemoV1.0 USB Audio")
    print("基于arecord命令，稳定可靠")
    print("=" * 60)
    
    recorder = SimpleUSBRecorder()
    
    # 设置信号处理
    def signal_handler(sig, frame):
        print("\n\n程序被用户中断")
        recorder.stop_recording()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    while True:
        print("\n请选择操作:")
        print("1. 列出音频设备")
        print("2. 测试麦克风")
        print("3. 开始录音 (手动停止)")
        print("4. 录音指定时长")
        print("5. 退出程序")
        
        try:
            choice = input("\n请输入选择 (1-5): ").strip()
            
            if choice == '1':
                recorder.list_devices()
            
            elif choice == '2':
                recorder.test_device()
            
            elif choice == '3':
                print("\n按 Ctrl+C 停止录音")
                recorder.start_recording()
            
            elif choice == '4':
                try:
                    duration = float(input("请输入录音时长(秒): "))
                    if duration > 0:
                        recorder.start_recording(duration=duration)
                    else:
                        print("录音时长必须大于0")
                except ValueError:
                    print("请输入有效的数字")
            
            elif choice == '5':
                print("退出程序")
                break
            
            else:
                print("无效选择，请重新输入")
                
        except KeyboardInterrupt:
            print("\n\n程序被用户中断")
            break
        except Exception as e:
            print(f"发生错误: {e}")

if __name__ == "__main__":
    main() 