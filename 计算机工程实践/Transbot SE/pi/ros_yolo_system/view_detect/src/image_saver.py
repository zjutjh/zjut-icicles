#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import time

class ImageSaver:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('image_saver', anonymous=True)
        
        # 创建图像转换桥接器
        self.bridge = CvBridge()
        
        # 获取共享目录路径
        self.shared_dir = rospy.get_param('~shared_dir', '/shared/images')
        
        # 确保共享目录存在
        if not os.path.exists(self.shared_dir):
            os.makedirs(self.shared_dir)
            
        # 设置保存图像的频率（默认每秒1张）
        self.save_frequency = rospy.get_param('~save_frequency', 1.0)
        self.last_save_time = time.time()
        
        # 图像计数器
        self.image_counter = 0
        
        # 最大保存图像数量（防止磁盘占满）
        self.max_images = rospy.get_param('~max_images', 100)
        
        # 订阅摄像头图像话题
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        rospy.loginfo("Image saver initialized. Saving images from %s to %s", 
                     self.image_topic, self.shared_dir)
                     
    def image_callback(self, msg):
        # 检查是否需要保存图像（基于频率）
        current_time = time.time()
        if current_time - self.last_save_time < 1.0/self.save_frequency:
            return
            
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 生成图像文件名（使用时间戳和计数器）
            timestamp = int(current_time * 1000)
            filename = "image_{0}_{1}.jpg".format(timestamp, self.image_counter)
            filepath = os.path.join(self.shared_dir, filename)
            
            # 保存图像
            cv2.imwrite(filepath, cv_image)
            
            # 更新计数器和时间
            self.image_counter += 1
            self.last_save_time = current_time
            
            rospy.loginfo("Saved image: %s", filename)
            
            # 清理旧图像（如果超过最大数量）
            self.cleanup_old_images()
            
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))
    
    def cleanup_old_images(self):
        # 获取目录中的所有图像文件
        files = [f for f in os.listdir(self.shared_dir) if f.endswith('.jpg')]
        
        # 如果图像数量超过最大值，删除最旧的图像
        if len(files) > self.max_images:
            # 按修改时间排序
            files.sort(key=lambda x: os.path.getmtime(os.path.join(self.shared_dir, x)))
            
            # 删除最旧的图像
            for i in range(len(files) - self.max_images):
                old_file = os.path.join(self.shared_dir, files[i])
                try:
                    os.remove(old_file)
                    rospy.loginfo("Removed old image: %s", files[i])
                except Exception as e:
                    rospy.logerr("Error removing old image: %s", str(e))

if __name__ == '__main__':
    try:
        image_saver = ImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 