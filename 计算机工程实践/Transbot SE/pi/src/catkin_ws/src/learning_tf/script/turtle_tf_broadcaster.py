#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()#定义一个tf广播
    #广播world与输入命名的turtle之间的tf变换
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
        
    rospy.init_node('turtle1_turtle2_tf_broadcaster')#初始化ros节点
    
    turtlename = rospy.get_param('~turtle') #从参数服务器中获取turtle的名字
    #订阅/pose话题数据，也就是turtle的位姿信息
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
