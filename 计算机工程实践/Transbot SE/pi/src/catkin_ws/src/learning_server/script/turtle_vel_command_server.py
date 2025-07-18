#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将执行/turtle_command服务，服务数据类型std_srvs/Trigger

import rospy
import thread,time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

pubvel = False;
turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=8)

def pubvel_thread():    
    while True:
        if pubvel:
            vel_msg = Twist()
            vel_msg.linear.x = 0.6
            vel_msg.angular.z = 0.8
            turtle_vel_pub.publish(vel_msg)
            
        time.sleep(0.1)

def pubvelCallback(req):
    global pubvel

    pubvel = bool(1-pubvel)

    rospy.loginfo("Do you want to publish the vel?[%s]", pubvel)# 显示请求数据

    return TriggerResponse(1, "Change  state!")# 反馈数据

def turtle_pubvel_command_server():
    
    rospy.init_node('turtle_vel_command_server')# ROS节点初始化

    # 创建一个名为/turtle_command的server，注册回调函数pubvelCallback
    s = rospy.Service('/turtle_vel_command', Trigger, pubvelCallback)

    # 循环等待回调函数
    print "Ready to receive turtle_pub_vel_command."

    thread.start_new_thread(pubvel_thread, ())
    rospy.spin()

if __name__ == "__main__":
    turtle_pubvel_command_server()
