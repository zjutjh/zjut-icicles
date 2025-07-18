#!/usr/bin/env python
# coding: utf-8
'''
在从机中执行此段代码-->订阅发布话题为"joint_states"的各关节角度,驱动真机移动
Execute this code in the slave device-->Subscribe and publish the angle of each joint whose topic is "joint_states" to drive the real machine to move
'''
import rospy
from math import pi
from Transbot_Lib import Transbot
from sensor_msgs.msg import JointState
import sys
sys.path.append("/root/Transbot/transbot")
from arm_transbot import Transbot_ARM
# 弧度转角度 Radian to angle
RA2DE = 180 / pi

'''
msg.name:  [arm_Joint
  - arm1_Joint
  - arm2_Joint
  - arm2_1_Joint
  - arm3_Joint
  - arm3_1_Joint
  - arm4_Joint
  - arm5_Joint
  - camera_Joint
  - camera1_Joint]
msg.position:  (0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0)
'''


def JointTopic(msg):
    # 如果不是该话题的数据直接返回 If it is not the data of the topic, return directly
    if not isinstance(msg, JointState): return
    if len(msg.position) == 10:
        joints1 = 200-((msg.position[0] * RA2DE)-13)*0.9 #round (200-((msg.position[0] * RA2DE)-13)*0.9)
        #print joints1
        joints2 = 30+((msg.position[1] * RA2DE)+12)*0.9
        #print joints2
        joints3 =  30+((msg.position[2] * RA2DE)-75)*(-1.6) #round (30+((msg.position[2] * RA2DE)-75)*(-1.6))
        #print joints3
        bot.set_uart_servo_angle_array(joints1, joints2, joints3)

if __name__ == '__main__':
    bot_arm = Transbot_ARM()
    bot_arm_offset = bot_arm.get_arm_offset()
    bot = Transbot(arm_offset=bot_arm_offset)
    rospy.init_node("ros_transbot")
    bot.create_receive_threading()
    subscriber = rospy.Subscriber("/joint_states", JointState, JointTopic)
    rospy.spin()