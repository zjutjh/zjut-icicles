#!/usr/bin/env python
# encoding: utf-8
from Double_PID import *
from Program_Ctrl import *
from transbot_msgs.msg import PWMServo


class MonoTracker:
    def __init__(self,):
        rospy.on_shutdown(self.Reset)
        self.target_servox = 90
        self.target_servoy = 90
        self.PWMServo = PWMServo()
        self.program_ctrl = ProgramCtrl()
        self.lin_pid = [0.025, 0.000, 0.002]
        self.ang_pid = [0.025, 0.000, 0.002]
        self.PID_init()

    def Reset(self):
        self.program_ctrl.PWMServo_topic(1, 90)
        rospy.sleep(0.1)
        self.program_ctrl.PWMServo_topic(2, 90)
        rospy.sleep(0.1)
        self.program_ctrl.PWMServo_topic(1, 90)
        self.target_servox = 90
        self.target_servoy = 90
        self.PID_init()
        rospy.loginfo("PWM init succes!!!")

    def execute(self, point_x, point_y, img_flip):
        # rospy.loginfo("point_x: {}, point_y: {}".format(point_x, point_y))
        [x_Pid, y_Pid] = self.Mono_Tracker_PID.update([point_x - 320, point_y - 240])
        if img_flip == True:
            self.target_servox -= x_Pid
            self.target_servoy -= y_Pid
        else:
            self.target_servox += x_Pid
            self.target_servoy += y_Pid
        if self.target_servox >= 180: self.target_servox = 180
        elif self.target_servox <= 0: self.target_servox = 0
        if self.target_servoy >= 180: self.target_servoy = 180
        elif self.target_servoy <= 0: self.target_servoy = 0
        # rospy.loginfo("target_servox: {}, target_servoy: {}".format(self.target_servox, self.target_servoy))
        self.program_ctrl.PWMServo_topic(1, self.target_servox)
        self.program_ctrl.PWMServo_topic(2, self.target_servoy)

    def Set_PID(self, lin_pid, ang_pid):
        self.lin_pid = lin_pid
        self.ang_pid = ang_pid
        self.PID_init()

    def PID_init(self,):
        # print ("Mono_Tracker_pid:", self.mono_PID)
        self.Mono_Tracker_PID = DoublePID(
            [0, 0],
            [self.lin_pid[0], self.ang_pid[0]],
            [self.lin_pid[1], self.ang_pid[1]],
            [self.lin_pid[2], self.ang_pid[2]])
