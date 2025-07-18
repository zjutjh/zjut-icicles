#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from learning_server.srv import IntPlus, IntPlusRequest

def Plus_client():
    # ROS节点初始化
    rospy.init_node('IntPlus_client')

    rospy.wait_for_service('/Two_Int_Plus')
    try:
        Plus_client = rospy.ServiceProxy('/Two_Int_Plus', IntPlus)

        response = Plus_client(22, 20)# 请求服务调用，输入请求数据

        return response.result
    except rospy.ServiceException, e:
        print "failed to call service : %s"%e

if __name__ == "__main__":
    #服务调用并显示调用结果
    print "Show two_int_plus result : %s" %(Plus_client())

