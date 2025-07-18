#!/usr/bin/env python
# encoding: utf-8
import getpass
import threading
import time

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyTeleop:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.user_name = getpass.getuser()
        self.linear_speed = 0
        self.angular_speed = 0
        self.Joy_state = False
        self.velocity = Twist()
        self.rate = rospy.Rate(20)
        self.Joy_time = time.time()
        self.linear_speed_limit = rospy.get_param('~linear_speed_limit', 2.0)
        self.angular_speed_limit = rospy.get_param('~angular_speed_limit', 2.0)
        self.pub_cmdVel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub_Joy = rospy.Subscriber('/joy', Joy, self.buttonCallback)
        vel_thread = threading.Thread(target=self.pub_vel())
        vel_thread.setDaemon(True)
        vel_thread.start()

    def pub_vel(self):
        while not rospy.is_shutdown():
            now_time = time.time()
            if now_time - self.Joy_time > 1:
                if self.Joy_state == True:
                    self.pub_cmdVel.publish(Twist())
                    self.Joy_state = False
                self.Joy_time = now_time
            if self.linear_speed == self.angular_speed == 0:
                if self.Joy_state == True:
                    self.pub_cmdVel.publish(Twist())
                    self.Joy_state = False
            else:
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed
                self.pub_cmdVel.publish(twist)
                self.Joy_state = True

    def cancel(self):
        self.pub_cmdVel.unregister()
        self.sub_Joy.unregister()

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy): return
        self.Joy_time = time.time()
        # rospy.loginfo(joy_data)
        # rospy.loginfo(joy_data.axes)
        if self.user_name == "jetson":
            self.linear_speed = joy_data.axes[1] * self.linear_speed_limit
            self.angular_speed = joy_data.axes[2] * self.angular_speed_limit
        else:
            self.linear_speed = joy_data.axes[1] * self.linear_speed_limit
            self.angular_speed = joy_data.axes[3] * self.angular_speed_limit


if __name__ == '__main__':
    rospy.init_node('twist_joy')
    joy = JoyTeleop()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')
