#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import rospy
from turtlesim.srv import Spawn

def turtle_spawn():
    
    rospy.init_node('new_turtle')# ROS节点初始化

    rospy.wait_for_service('/spawn')# 等待/spawn服务

    try:
        new_turtle = rospy.ServiceProxy('/spawn', Spawn)

        
        response = new_turtle(2.0, 2.0, 0.0, "turtle2")# 输入请求数据
        return response.name
    except rospy.ServiceException, e:
        print "failed to call service : %s"%e

if __name__ == "__main__":
    #服务调用并显示调用结果
    print "a new turtle named %s." %(turtle_spawn())
