#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from learning_server.srv import IntPlus, IntPlusResponse

def IntPlusCallback(req):
    
    rospy.loginfo("Ints: a:%d  b:%d", req.a, req.b)# 显示请求数据
    
    return IntPlusResponse(req.a+req.b)# 反馈数据

def IntPlus_server():
    
    rospy.init_node('IntPlus_server')# ROS节点初始化

    # 创建一个server，注册回调函数IntPlusCallback
    s = rospy.Service('/Two_Int_Plus', IntPlus, IntPlusCallback)

    
    print "Ready to caculate two ints."# 循环等待回调函数

    rospy.spin()

if __name__ == "__main__":
    IntPlus_server()

