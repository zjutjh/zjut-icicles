#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlesim.msg import Pose

def poseCallback(msg):
    rospy.loginfo("Turtle pose: x:%0.3f, y:%0.3f", msg.x, msg.y)

def turtle_pose_subscriber():
    
    rospy.init_node('turtle_pose_subscriber', anonymous=True)# ROS节点初始化

    # 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("/turtle1/pose", Pose, poseCallback)

    
    rospy.spin()# 循环等待回调函数

if __name__ == '__main__':
    turtle_pose_subscriber()

