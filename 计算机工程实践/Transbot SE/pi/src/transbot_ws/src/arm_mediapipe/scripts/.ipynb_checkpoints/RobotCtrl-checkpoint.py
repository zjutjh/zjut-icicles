#!/usr/bin/env python3.8
# encoding: utf-8
import threading
import numpy as np
from media_library import *
from time import sleep, time

class HandCtrlArm:
    def __init__(self):
        self.media_ros = Media_ROS()
        self.hand_detector = HandDetector()
        self.arm_status = True
        self.locking = True
        self.init = True
        self.pTime = 0
        self.add_lock = self.remove_lock = 0
        self.media_ros.pub_arm(192, 100, 60)
        self.event = threading.Event()
        self.event.set()

    def dance(self):
        time_sleep = 0.5
        self.media_ros.pub_vel(0, 0)
        sleep(time_sleep)
        self.media_ros.pub_vel(0, 0)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 60, 120)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 60, 135)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 60.0, 120)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 100, 80)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 120, 60)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 135, 45)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(50, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(50, 90, 180)
        sleep(time_sleep)
        #self.media_ros.pub_arm([], 6, 180)
        #sleep(time_sleep)
        #self.media_ros.pub_arm(0.0, 0.0, 0.0)
        #sleep(time_sleep)
        self.media_ros.pub_arm(90, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(90, 70, 0)
        sleep(time_sleep)

    def process(self, frame):
        frame = cv.flip(frame, 1)
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0 and self.media_ros.Joy_active:
            threading.Thread(target=self.arm_ctrl_threading, args=(lmList,bbox)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.media_ros.pub_imgMsg(frame)
        return frame

    def arm_ctrl_threading(self, lmList,bbox):
        if self.event.is_set():
            self.event.clear()
            fingers = self.hand_detector.fingersUp(lmList)
            self.hand_detector.draw = False
            gesture = self.hand_detector.get_gesture(lmList)
            self.arm_status = False
            point_x = lmList[9][1]
            point_y = lmList[9][2]
            if point_y >= 260: x = -0.2
            elif point_y <= 220: x = 0.2
            else: x =0 
            if point_x >= 340: z = -0.2
            elif point_x <= 300: z = 0.2
            else: z = 0
            self.media_ros.pub_vel(x, z)
            print("angle: {},value: {}".format(x, z))
            self.arm_status = True
            self.event.set()


if __name__ == '__main__':
    rospy.init_node('HandCtrlArm_node', anonymous=True)
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    ctrl_arm = HandCtrlArm()
    while capture.isOpened():
        ret, frame = capture.read()
        action = cv.waitKey(1) & 0xFF
        frame = ctrl_arm.process(frame)
        if action == ord('q'):
            ctrl_arm.media_ros.cancel()
            break
        cv.imshow('frame', frame)
    capture.release()
    cv.destroyAllWindows()
