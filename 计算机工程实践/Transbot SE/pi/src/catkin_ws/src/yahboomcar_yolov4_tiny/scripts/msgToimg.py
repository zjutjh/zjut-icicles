#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import base64
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yahboomcar_msgs.msg import Image_Msg

class msgToimg:
    def __init__(self):
        rospy.init_node("msgToimg", anonymous=False)
        rospy.on_shutdown(self.cancel)
        self.bridge = CvBridge()
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像  The original image
        self.img_flip = rospy.get_param("~img_flip", False)
        self.image_sub = rospy.Subscriber("Detect/image_msg", Image_Msg, self.image_sub_callback)
        self.pub_img = rospy.Publisher("yoloDetect/image", Image, queue_size=10)

    def image_sub_callback(self,data):
        if not isinstance(data, Image_Msg): return
        # 将自定义图像消息转化为图像
        # Convert custom image messages to images
        image = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8,
                           buffer=base64.b64decode(data.data))
        self.img[:, :, 0], self.img[:, :, 1], self.img[:, :, 2] = image[:, :, 2], image[:, :, 1], image[:, :, 0]
        self.img = cv.cvtColor(self.img, cv.COLOR_BGR2RGB)
        # 规范输入图像大小
        # Standardize the input image size
        self.img = cv.resize(self.img, (640, 480))
        if self.img_flip == True: self.img = cv.flip(self.img, 1)
        # opencv mat ->  ros msg
        msg = self.bridge.cv2_to_imgmsg(self.img, "bgr8")
        self.pub_img.publish(msg)

    def cancel(self):
        self.image_sub.unregister()
        self.pub_img.unregister()

if __name__ == '__main__':
    msgToimg()
    rospy.spin()
