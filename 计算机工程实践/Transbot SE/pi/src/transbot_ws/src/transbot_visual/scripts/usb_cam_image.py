#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def topic(msg):
    if not isinstance(msg, Image):
        return
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    # 规范输入图像大小
    # Standardize the input image size
    frame = cv.resize(frame, (640, 480))
    cv.imshow("color_image", frame)
    cv.waitKey(10)


if __name__ == '__main__':
    rospy.init_node("usb_cam_image_py")
    sub = rospy.Subscriber("/usb_cam/image_raw", Image, topic)
    rate = rospy.Rate(2)
    rospy.spin()
