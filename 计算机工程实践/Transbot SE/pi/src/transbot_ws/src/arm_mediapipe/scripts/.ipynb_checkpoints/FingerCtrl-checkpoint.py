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
        self.media_ros.pub_arm(219, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 60, 120)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 60, 135)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 60, 120)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 100, 80)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 120, 60)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 135, 45)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(219, 90, 180)
        sleep(time_sleep)
        #self.media_ros.pub_arm([], 6, 180)
        #sleep(time_sleep)
        #self.media_ros.pub_arm(0.0, 0.0, 0.0)
        #sleep(time_sleep)
        self.media_ros.pub_arm(190, 90, 90)
        sleep(time_sleep)
        self.media_ros.pub_arm(190, 70, 30)
        sleep(time_sleep)

    def init_pose(self):
        self.media_ros.pub_arm(180, 80, 100)
        sleep(0.5)
        self.media_ros.RobotBuzzer()
        

    def arm_applaud(self):
        for i in range(3):
            self.media_ros.pub_arm(208, 100, 50)    
            sleep(0.5)
            self.media_ros.pub_arm(208, 100, 94)    
            sleep(0.5)
        self.media_ros.pub_arm(208, 100, 50)
    def shake(self):
        for i in range(3):
            self.media_ros.pub_arm(138, 179, 62)    
            sleep(0.5)
            self.media_ros.pub_arm(138, 179, 92)    
            sleep(0.5)
        self.media_ros.pub_arm(138, 179, 62)

    def arm_nod(self):
        for i in range(3):
            self.media_ros.pub_arm(208.0, 80, 92)    
            sleep(0.5)
            self.media_ros.pub_arm(208, 110, 92)    
            sleep(0.5)
        self.media_ros.pub_arm(208, 80, 92)

        
			
			
			

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
            if gesture == "Yes":	
                self.arm_status = False
                self.media_ros.pub_vel(0, 0)
                self.dance()
                sleep(0.5)
                self.init_pose()
                sleep(1.0)
                self.arm_status = True
			
            elif gesture == "OK":
                self.arm_status = False
                for i in range(3):
                    self.media_ros.pub_vel(0, 0)
                    sleep(0.1)
                    self.media_ros.pub_arm(186, 186, 52)
                    sleep(0.5)
                    self.media_ros.pub_arm(186, 186, 112)
                    sleep(0.5)
                    self.media_ros.pub_arm(186, 186, 52)
                self.init_pose()
                sleep(1.0)
                self.arm_status = True
            elif gesture == "Thumb_down":
                self.arm_status = False
                self.media_ros.pub_vel(0, 0)
                self.media_ros.pub_arm(120, 227, 68)
                sleep(0.5)
                self.media_ros.pub_arm(177, 227, 68)
                sleep(0.5)
                self.init_pose()
                sleep(1.0)
                self.arm_status = True


            elif sum(fingers) == 1: 
                self.arm_status = False
                self.media_ros.pub_vel(0, 0)
                self.arm_nod()
                self.init_pose()
                sleep(1.0)
                self.arm_status = True


            elif fingers[1] == fingers[4] == 1 and sum(fingers) == 2: 
                self.arm_status = False
                self.media_ros.pub_vel(0, 0)
                self.shake()
                self.init_pose()
                sleep(1.0)
                self.arm_status = True

            elif sum(fingers) == 5: 
                self.arm_status = False
                self.media_ros.pub_vel(0, 0)
                self.arm_applaud()
                self.init_pose()
                sleep(1.0)
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
