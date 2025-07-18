#!/usr/bin/env python3
# encoding: utf-8
import cv2
import os
import time
from datetime import datetime

def capture_frames_to_images(duration=2, output_dir="captured_images", target_fps=10):
    """
    从video2摄像头捕获指定时长的视频，并将每一帧保存为图像文件
    
    Args:
        duration: 录制时长（秒）
        output_dir: 输出图像文件夹
        target_fps: 目标帧率（每秒保存的帧数）
    """
    print(f"开始从 /dev/video2 捕获 {duration} 秒视频...")
    print(f"目标帧率: {target_fps} FPS（每秒保存{target_fps}帧）")
    
    # 计算帧间隔时间（毫秒）
    frame_interval = 1.0 / target_fps
    print(f"帧间隔: {frame_interval:.3f}秒")
    
    # 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"创建输出目录: {output_dir}")
    
    # 打开摄像头 /dev/video2
    print("尝试打开摄像头 /dev/video2...")
    capture = cv2.VideoCapture(2)
    
    # 检查摄像头是否成功打开
    if not capture.isOpened():
        print("错误：无法打开摄像头 /dev/video2")
        print("尝试其他摄像头设备...")
        # 尝试使用其他设备
        for i in [0, 1, 3]:
            print(f"尝试摄像头设备 {i}...")
            capture = cv2.VideoCapture(i)
            if capture.isOpened():
                print(f"成功打开摄像头设备 /dev/video{i}")
                break
        else:
            print("无法找到可用的摄像头设备")
            return False
    else:
        print("成功打开摄像头 /dev/video2")
    
    # 获取摄像头信息
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_camera = capture.get(cv2.CAP_PROP_FPS)
    print(f"摄像头分辨率: {width}x{height}, FPS: {fps_camera}")
    
    # 计算预期的总帧数和精确的帧间隔
    expected_frames = duration * target_fps
    precise_frame_interval = duration / expected_frames  # 精确的帧间隔
    print(f"预期保存帧数: {expected_frames}")
    print(f"精确帧间隔: {precise_frame_interval:.3f}秒")
    
    # 记录开始时间
    start_time = time.time()
    frame_count = 0
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    try:
        for i in range(expected_frames):
            # 计算这一帧应该在什么时候保存
            target_time = start_time + i * precise_frame_interval
            
            # 等待到达目标时间
            while True:
                current_time = time.time()
                if current_time >= target_time:
                    break
                time.sleep(0.005)  # 5ms精度
            
            # 读取视频帧
            ret, frame = capture.read()
            if not ret:
                print("无法读取摄像头数据")
                break
            
            # 生成文件名并保存
            filename = f"frame_{timestamp}_{frame_count:04d}.jpg"
            filepath = os.path.join(output_dir, filename)
            cv2.imwrite(filepath, frame)
            frame_count += 1
            
            # 每隔5帧打印进度
            if frame_count % 5 == 0:
                elapsed_time = current_time - start_time
                print(f"已保存 {frame_count} 帧，耗时 {elapsed_time:.1f}s")
        
        # 确保总时长达到预期
        final_time = time.time()
        total_elapsed = final_time - start_time
        if total_elapsed < duration:
            remaining_time = duration - total_elapsed
            time.sleep(remaining_time)
            
    except Exception as e:
        print(f"捕获过程中发生错误: {e}")
        return False
    finally:
        capture.release()
        
    print(f"视频捕获完成！")
    print(f"总共保存了 {frame_count} 帧图像")
    print(f"图像保存路径: {os.path.abspath(output_dir)}")
    return True

if __name__ == '__main__':
    # 获取当前脚本所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(current_dir, "captured_images")
    
    print("=== 视频转图像脚本 ===")
    print(f"工作目录: {current_dir}")
    
    # 开始捕获
    success = capture_frames_to_images(duration=2, output_dir=output_path, target_fps=10)
    
    if success:
        print("脚本执行成功！")
    else:
        print("脚本执行失败！") 