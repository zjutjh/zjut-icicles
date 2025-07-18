#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


def topic(msg):
    if not isinstance(msg, Image):
        return
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    # 规范输入图像大小
    # Standardize the input image size
    frame = cv.resize(frame, (640, 480))
    frame = cv.flip(frame, 1)
    # opencv mat ->  ros msg
    msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    pub_img.publish(msg)
    # cv.imshow("Image", frame)
    # cv.waitKey(10)


def compressed_topic(msg):
    if not isinstance(msg, CompressedImage):
        return
    bridge = CvBridge()
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    # 规范输入图像大小
    # Standardize the input image size
    frame = cv.resize(frame, (640, 480))
    frame = cv.flip(frame, 1)
    # Create CompressedIamge #
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.data = np.array(cv.imencode('.jpg', frame)[1]).tostring()
    pub_comimg.publish(msg)
    # cv.imshow("CompressedImage", frame)
    # cv.waitKey(10)

if __name__ == '__main__':
    rospy.init_node("usb_cam_flip", anonymous=False)
    sub_img = rospy.Subscriber("/usb_cam/image_raw", Image, topic)
    pub_img = rospy.Publisher("/usb_cam/image_flip", Image, queue_size=10)
    sub_comimg = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, compressed_topic)
    pub_comimg = rospy.Publisher("/usb_cam/image_flip/compressed", CompressedImage, queue_size=10)
    rate = rospy.Rate(2)
    rospy.spin()
