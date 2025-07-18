#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将发布turtle1/cmd_vel话题，消息类型geometry_msgs::Twist

import rospy
from geometry_msgs.msg import Twist

def turtle_velocity_publisher():
    
    rospy.init_node('turtle_velocity_publisher', anonymous=True) # ROS节点初始化

    # 创建一个小海龟速度发布者，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，8代表消息队列长度
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=8)

    
    rate = rospy.Rate(10) #设置循环的频率

    while not rospy.is_shutdown():
        # 初始化geometry_msgs::Twist类型的消息
        turtle_vel_msg = Twist()
        turtle_vel_msg.linear.x = 0.8
        turtle_vel_msg.angular.z = 0.6

        # 发布消息
        turtle_vel_pub.publish(turtle_vel_msg)
        rospy.loginfo("linear is :%0.2f m/s, angular is :%0.2f rad/s", 
                turtle_vel_msg.linear.x, turtle_vel_msg.angular.z)

        
        rate.sleep()# 按照循环频率延时

if __name__ == '__main__':
    try:
        turtle_velocity_publisher()
    except rospy.ROSInterruptException:
        pass

