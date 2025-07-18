#!/usr/bin/env python
# encoding: utf-8
from Program_Ctrl import *
from Double_PID import *
from geometry_msgs.msg import Twist


class FollowLine:
    def __init__(self,CameraDevice ="Astra"):
        rospy.on_shutdown(self.cancel)
        self.Buzzer_state = False
        self.program_ctrl = ProgramCtrl()
        self.CameraDevice = CameraDevice
        if self.CameraDevice == "Astra":
            (self.follow_line_KP, self.follow_line_KI, self.follow_line_KD) = (0.010, 0.003, 0.010)
            self.linear = 0.2
        else:
            (self.follow_line_KP, self.follow_line_KI, self.follow_line_KD) = (0.015, 0.002, 0.010)
            self.linear = 0.25
        self.PID_init()

    def cancel(self):
        self.Reset()
        self.program_ctrl.pub_cmdVel.publish(Twist())

    def execute(self, point_x, color_radius, warning,img_flip=False):
        self.twist = Twist()
        if color_radius == 0: self.program_ctrl.pub_cmdVel.publish(Twist())
        else:
            self.twist = Twist()
            [z_Pid, _] = self.PID_controller.update([(point_x - 320)/16, 0])
            if img_flip == True: self.twist.angular.z = -z_Pid
            else: self.twist.angular.z = +z_Pid
            self.twist.linear.x = self.linear
            if warning > 10:
                rospy.loginfo("Obstacles ahead !!!")
                self.program_ctrl.pub_cmdVel.publish(Twist())
                self.program_ctrl.Buzzer_srv(1)
                self.Buzzer_state = True
            else:
                if self.Buzzer_state == True:
                    self.program_ctrl.Buzzer_srv(0)
                    self.Buzzer_state = False
                self.program_ctrl.pub_cmdVel.publish(self.twist)
        # rospy.loginfo("point_x: {},linear: {}, z_Pid: {}".format(point_x, self.twist.linear.x, self.twist.angular.z))

    def Reset(self):
        self.program_ctrl.PWMServo_topic(1, 90)
        rospy.sleep(0.1)
        self.program_ctrl.PWMServo_topic(2, 70)  # 115
        rospy.sleep(0.1)
        self.program_ctrl.PWMServo_topic(1, 90)
        self.PID_init()
        rospy.loginfo("follow_line Reset succes!!!")

    def set_PID(self,pid,linear):
        self.follow_line_KP = pid[0]
        self.follow_line_KI = pid[1]
        self.follow_line_KD = pid[2]
        self.linear = linear

    def PID_init(self):
        # print ("follow_line_pid:", self.follow_line_KP, self.follow_line_KI, self.follow_line_KD)
        self.PID_controller = DoublePID(
            [0, 0],
            [self.follow_line_KP, 0],
            [self.follow_line_KI, 0],
            [self.follow_line_KD, 0])
