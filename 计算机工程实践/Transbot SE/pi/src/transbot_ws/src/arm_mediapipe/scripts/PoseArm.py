#!/usr/bin/env python3.8
# encoding: utf-8
import threading
import cv2 as cv
import numpy as np
from media_library import *
from time import sleep, time

class PoseCtrlArm:
    def __init__(self):
        self.car_status = True
        self.stop_status = 0
        self.locking = False
        self.pose_detector = Holistic()
        self.hand_detector = HandDetector()
        self.pTime = self.index = 0
        self.media_ros = Media_ROS()
        self.event = threading.Event()
        self.event.set()

    def process(self, frame):
        frame = cv.flip(frame, 1)
        if self.media_ros.Joy_active:
            frame, pointArray, lhandptArray, rhandptArray = self.pose_detector.findHolistic(frame)
            threading.Thread(target=self.arm_ctrl_threading, args=(pointArray, lhandptArray, rhandptArray)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.media_ros.pub_imgMsg(frame)
        return frame


    def arm_ctrl_threading(self, pointArray, lhandptArray, rhandptArray):
        if self.event.is_set():
            self.event.clear()
            joints = [150, 80, 100]
            if self.stop_status <= 30:
                self.media_ros.pub_vel(0, 0)
                self.stop_status += 1
            if len(pointArray) != 0:
                point11 = pointArray[11][1:3]
                point12 = pointArray[12][1:3]
                point13 = pointArray[13][1:3]
                point15 = pointArray[15][1:3]
                point21 = pointArray[21][1:3]
                angle11 = round(calc_angle(point11, point12, point13), 2)
                angle13 = round(180 - calc_angle(point11, point13, point15), 2)
                if len(lhandptArray) != 0:
                    point0 = lhandptArray[0][1:3]
                    point4 = lhandptArray[4][1:3]
                    point8 = lhandptArray[8][1:3]
                    angle15 = round(180 - calc_angle(point13, point0, point8), 2)
                    if point8[1] > point0[1]: angle15 = -angle15
                    grip_angle = round(180 - calc_angle(point4, point0, point8), 2)
                    grip_joint = np.interp(grip_angle, [90, 180], [30, 190])
                else:
                    angle15 = round(180 - calc_angle(point13, point15, point21), 2)
                    if point21[1] > point15[1]: angle15 = -angle15
                    grip_joint = 30
                if point13[1] > point11[1]: angle11 = -angle11
                if point15[1] > point13[1]: angle13 = -angle13
                angle13 += 90
                angle15 += 90
                angle13 =abs(angle13)
                angle15 =abs(angle15)
                # print("angle11: {},angle13: {},angle15: {}".format(angle11, angle13, angle15))
                #print(angle11)
                if abs(angle11) > 10:
                    #if self.index >= 30:
                        #self.media_ros.RobotBuzzer()
                        #self.media_ros.pub_arm(joints, runtime=5000)
                        #sself.car_status = True
                    #else:self.index += 1
                    #if angle11 < 0: angle11 = 0  # [0, 30]
                    #angle11 = np.interp(angle11, [0, 30], [0, 90])
                    #self.media_ros.pub_arm([0, angle11, angle13, angle15, 90, grip_joint])
                    #self.index = 0
                    print(abs(angle15))
                    self.media_ros.RobotBuzzer()
                else:
                    if angle11 < 0: angle11 = 0  # [0, 30]
                    angle11 = np.interp(angle11, [0, 30], [0, 90])
                    if grip_joint <180 :
                    	if angle13 <180 and angle15 <180:
                    		self.media_ros.pub_arm((angle13), (angle15), grip_joint)
                    self.index = 0
                    #self.media_ros.RobotBuzzer()
                self.event.set()
            else:
                #self.media_ros.RobotBuzzer()
                #self.media_ros.pub_arm(joints, runtime=5000)
                self.event.set()
                #self.car_status = True
        #self.event.set()

'''
Robot控制模式：
以小车视角
    手势拳头：前进
    手势1：左移
    手势2：右移
    手势3：左旋
    手势4：右旋
    手势5：后退
    手势LOCK：模式切换arm模式。
arm控制模式： 模拟右手臂运动
    右手臂竖直略偏左放下连续识别30次,切换Robot模式。
'''

if __name__ == '__main__':
    rospy.init_node('PoseCtrlArm_node', anonymous=True)
    pose_ctrl_arm = PoseCtrlArm()
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    while capture.isOpened():
        ret, frame = capture.read()
        frame = pose_ctrl_arm.process(frame)
        if cv.waitKey(1) & 0xFF == ord('q'): break
        cv.imshow('frame', frame)
    capture.release()
    cv.destroyAllWindows()

