#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from learning_topic.msg import Information #导入自定义的msg

def CompanyInfoCallback(msg):

    rospy.loginfo("company: name:%s  city:%s ", msg.company, msg.city)#打印订阅接收到信息

def Infomation_subscriber():
    
    rospy.init_node('Infomation_subscriber', anonymous=True)# ROS节点初始化

    # 创建一个Subscriber，订阅名为/company_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/company_info", Information, CompanyInfoCallback)

    rospy.spin()# 循环等待回调函数

if __name__ == '__main__':
    Infomation_subscriber()
