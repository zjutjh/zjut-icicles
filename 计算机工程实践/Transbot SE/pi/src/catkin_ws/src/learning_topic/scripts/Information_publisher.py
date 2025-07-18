#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from learning_topic.msg import Information #导入自定义的msg

def information_publisher():
    
    rospy.init_node('information_publisher', anonymous=True)# ROS节点初始化

    # 创建一个Publisher，发布名为/company_info的topic，消息类型为learning_topic::Information，队列长度6
    info_pub = rospy.Publisher('/company_info', Information, queue_size=6)

    rate = rospy.Rate(10) #设置循环的频率

    while not rospy.is_shutdown():

    # 初始化learning_topic::Information 类型的消息
        info_msg = Information()
        info_msg.company = "Yahboom";
        info_msg.city  = "Shenzhen";
    
        info_pub.publish(info_msg)# 发布消息

        rospy.loginfo("This is %s in %s.", info_msg.company, info_msg.city)# 打印发布消息
        
        rate.sleep()# 按照循环频率延时

if __name__ == '__main__':
    try:
        information_publisher()
    except rospy.ROSInterruptException:
        pass
