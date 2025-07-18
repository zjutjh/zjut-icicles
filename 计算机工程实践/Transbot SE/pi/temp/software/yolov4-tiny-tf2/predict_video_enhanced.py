#!/usr/bin/env python3
# encoding: utf-8
import os
import argparse
from utils.yolo import YOLO
from PIL import Image
import numpy as np
import cv2
import time
import tensorflow as tf

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='YOLOv4-tiny实时视频检测')
    parser.add_argument('--source', type=str, default='0',
                        help='视频源：摄像头ID(如0,1)或视频文件路径')
    parser.add_argument('--model', type=str, default='logs/last1.weights.h5',
                        help='模型权重文件路径')
    parser.add_argument('--classes', type=str, default='model_data/CampusBot.txt',
                        help='类别名称文件路径')
    parser.add_argument('--anchors', type=str, default='model_data/CampusBot_anchors.txt',
                        help='锚框文件路径')
    parser.add_argument('--score', type=float, default=0.5,
                        help='置信度阈值')
    parser.add_argument('--iou', type=float, default=0.3,
                        help='IOU阈值')
    parser.add_argument('--save_video', type=str, default='',
                        help='保存检测结果视频的路径')
    parser.add_argument('--show_fps', action='store_true',
                        help='是否显示FPS信息')
    
    args = parser.parse_args()
    
    # 配置GPU
    gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
    for gpu in gpus:
        tf.config.experimental.set_memory_growth(gpu, True)
    
    # 初始化YOLO
    print("正在加载YOLO模型...")
    yolo = YOLO(
        model_path=args.model,
        classes_path=args.classes,
        anchors_path=args.anchors,
        score=args.score,
        iou=args.iou
    )
    
    # 确定视频源
    if args.source.isdigit():
        # 摄像头
        video_source = int(args.source)
        print(f"使用摄像头 {video_source}")
    else:
        # 视频文件
        video_source = args.source
        print(f"使用视频文件: {video_source}")
    
    # 打开视频源
    capture = cv2.VideoCapture(video_source)
    if not capture.isOpened():
        print(f"错误：无法打开视频源 {args.source}")
        return
    
    # 获取视频信息
    fps_input = capture.get(cv2.CAP_PROP_FPS)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"视频信息: {width}x{height}, FPS: {fps_input}")
    
    # 设置视频写入器（如果需要保存）
    video_writer = None
    if args.save_video:
        fourcc = cv2.VideoWriter.fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(args.save_video, fourcc, fps_input, (width, height))
        print(f"将保存检测结果到: {args.save_video}")
    
    fps = 0.0
    print("开始检测，按 'q' 或 ESC 退出...")
    
    try:
        while True:
            t1 = time.time()
            
            # 读取视频帧
            ret, frame = capture.read()
            if not ret:
                print("视频结束或读取失败")
                break
            
            # BGR转RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # 转换为PIL Image
            image = Image.fromarray(np.uint8(frame_rgb))
            
            # YOLO检测
            result_image, out_boxes, out_scores, out_classes = yolo.detect_image(image)
            result_frame = np.array(result_image)
            
            # RGB转BGR用于OpenCV显示
            result_frame = cv2.cvtColor(result_frame, cv2.COLOR_RGB2BGR)
            
            # 计算并显示FPS
            if args.show_fps:
                fps = (fps + abs(1. / (time.time() - t1))) / 2
                result_frame = cv2.putText(result_frame, f"FPS: {fps:.2f}", 
                                         (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                         1, (0, 255, 0), 2)
            
            # 显示检测信息
            if len(out_boxes) > 0:
                print(f"检测到 {len(out_boxes)} 个目标")
            
            # 显示结果
            cv2.imshow("YOLO Detection", result_frame)
            
            # 保存视频帧（如果需要）
            if video_writer:
                video_writer.write(result_frame)
            
            # 检查退出键
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):  # ESC或q键
                break
                
    except KeyboardInterrupt:
        print("用户中断")
    
    finally:
        # 清理资源
        capture.release()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()
        print("检测结束")

if __name__ == "__main__":
    main() 