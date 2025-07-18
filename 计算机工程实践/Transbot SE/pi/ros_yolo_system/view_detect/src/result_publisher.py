#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import json
import time
from view_detect.msg import YoloResult

class ResultPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('result_publisher', anonymous=True)
        
        # 获取共享目录路径
        self.results_dir = rospy.get_param('~results_dir', '/shared/results')
        
        # 确保结果目录存在
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)
        
        # 创建结果发布者
        self.result_pub = rospy.Publisher('yolo_detection', YoloResult, queue_size=10)
        
        # 设置检查结果的频率（默认每秒2次）
        self.check_frequency = rospy.get_param('~check_frequency', 2.0)
        self.check_period = 1.0 / self.check_frequency
        
        # 记录已处理的结果文件
        self.processed_files = set()
        
        rospy.loginfo("Result publisher initialized. Monitoring %s", self.results_dir)
        
        # 启动定时器定期检查结果目录
        rospy.Timer(rospy.Duration(self.check_period), self.check_results)
    
    def check_results(self, event=None):
        # 获取结果目录中的所有JSON文件
        try:
            files = [f for f in os.listdir(self.results_dir) if f.endswith('.json')]
            
            # 按修改时间排序，处理最新的文件
            files.sort(key=lambda x: os.path.getmtime(os.path.join(self.results_dir, x)))
            
            for filename in files:
                # 跳过已处理的文件
                if filename in self.processed_files:
                    continue
                    
                filepath = os.path.join(self.results_dir, filename)
                
                try:
                    # 读取JSON结果文件
                    with open(filepath, 'r') as f:
                        result_data = json.load(f)
                    
                    # 创建并填充YoloResult消息
                    result_msg = YoloResult()
                    result_msg.image_id = result_data.get('image_id', '')
                    result_msg.class_names = result_data.get('class_names', [])
                    result_msg.counts = result_data.get('counts', [])
                    
                    # 发布结果
                    self.result_pub.publish(result_msg)
                    
                    # 添加到已处理集合
                    self.processed_files.add(filename)
                    
                    rospy.loginfo("Published detection result for image: %s", result_msg.image_id)
                    
                    # 限制已处理文件集合的大小
                    if len(self.processed_files) > 1000:
                        self.processed_files = set(list(self.processed_files)[-500:])
                        
                except Exception as e:
                    rospy.logerr("Error processing result file %s: %s", filename, str(e))
                    
        except Exception as e:
            rospy.logerr("Error checking results directory: %s", str(e))

if __name__ == '__main__':
    try:
        result_publisher = ResultPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 