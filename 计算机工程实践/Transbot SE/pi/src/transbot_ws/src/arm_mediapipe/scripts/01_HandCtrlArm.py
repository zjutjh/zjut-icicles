#!/usr/bin/env python3
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
        self.media_ros.pub_arm([90, 135, 0, 45, 90, 90])
        self.event = threading.Event()
        self.event.set()

    def dance(self):
        time_sleep = 0.5
        self.media_ros.pub_vel(0, 0)
        sleep(time_sleep)
        self.media_ros.pub_vel(0, 0)
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 60, 120, 60, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 45, 135, 45, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 60, 120, 60, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 100, 80, 80, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 120, 60, 60, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 135, 45, 45, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 20, 90, 150])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 20, 90, 150])
        sleep(time_sleep)
        self.media_ros.pub_arm([0, 90, 90, 90, 0, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([0, 90, 180, 0, 0, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([], 6, 180)
        sleep(time_sleep)
        self.media_ros.pub_arm([], 6, 30)
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 90, 90, 90, 90, 90])
        sleep(time_sleep)
        self.media_ros.pub_arm([90, 135, 0, 45, 90, 90])
        sleep(time_sleep)

    def process(self, frame):
        frame = cv.flip(frame, 1)
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0 and self.media_ros.Joy_active:
            threading.Thread(target=self.arm_ctrl_threading, args=(lmList, bbox)).start()
        self.cTime = time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.media_ros.pub_imgMsg(frame)
        return frame

    def arm_ctrl_threading(self, lmList, bbox):
        if self.event.is_set():
            self.event.clear()
            fingers = self.hand_detector.fingersUp(lmList)
            # 手势lock切换robot控制模式
            if fingers[1] == fingers[4] == 1 and sum(fingers) == 2 and self.locking:
                self.remove_lock += 1
                if self.remove_lock > 10:
                    self.locking = False
                    self.hand_detector.draw = False
                    self.media_ros.RobotBuzzer()
                    self.add_lock = 0
            # 手势4切换arm控制模式
            if fingers[0] == 0 and sum(fingers) == 4 and not self.locking:
                self.add_lock += 1
                if self.add_lock > 10:
                    self.locking = True
                    self.hand_detector.draw = True
                    self.media_ros.RobotBuzzer()
                    self.remove_lock = 0
            if self.locking:
                if self.init:
                    self.media_ros.pub_arm([90, 90, 35, 60, 90, 30])
                    sleep(0.5)
                    self.init = False
                else:
                    self.arm_status = False
                    angle = self.hand_detector.ThumbTOforefinger(lmList)
                    value = np.interp(angle, [0, 70], [185, 20])
                    indexX = (bbox[0] + bbox[2]) / 2
                    indexY = (bbox[1] + bbox[3]) / 2
                    if indexY > 400: indexY = 400
                    elif indexY < 200: indexY = 200
                    joint2 = -0.4 * indexY + 170
                    joint3 = 0.05 * indexY + 25
                    joint4 = -0.125 * indexY + 85
                    if 300 < indexX < 340: joint1 = 90
                    else: joint1 = -0.3 * indexX + 186
                    self.media_ros.pub_arm([joint1, joint2, joint3, joint4, 90, value])
                    sleep(0.01)
                    self.arm_status = True
            else:
                self.init = True
                gesture = self.hand_detector.get_gesture(lmList)
                if gesture == "Yes":
                    self.arm_status = False
                    self.media_ros.pub_vel(0, 0)
                    self.dance()
                    self.arm_status = True
                elif gesture == "OK":
                    self.arm_status = False
                    for i in range(3):
                        self.media_ros.pub_vel(0, 0)
                        sleep(0.1)
                        self.media_ros.pub_arm([80, 135, 0, 15, 90, 180])
                        sleep(0.5)
                        self.media_ros.pub_arm([110, 135, 0, 15, 90, 90])
                        sleep(0.5)
                    self.arm_status = True
                elif gesture == "Thumb_down":
                    self.arm_status = False
                    self.media_ros.pub_vel(0, 0)
                    self.media_ros.pub_arm([90, 0, 180, 0, 90, 180])
                    sleep(0.5)
                    self.media_ros.pub_arm([90, 0, 180, 0, 90, 90])
                    sleep(0.5)
                    self.arm_status = True
                else:
                    self.arm_status = False
                    point_x = lmList[9][1]
                    point_y = lmList[9][2]
                    if point_y >= 260: x = -0.2
                    elif point_y <= 220: x = 0.2
                    else: x = 0
                    if point_x >= 340: y = 0.2
                    elif point_x <= 300: y = -0.2
                    else: y = 0
                    self.media_ros.pub_vel(x, y)
                    # print("angle: {},value: {}".format(x, y))
                    self.arm_status = True
            self.event.set()

'''
手势控制X3plus
默认arm控制模式
    手势lock：robot控制模式
    arm控制模式控制机械臂抓取物体
robot控制模式下：
    手势4  ：arm控制模式
    手势Yes：机械臂跳舞
    手势OK ：点头
    手势Thumb_down：磕头
    移动手部，控制小车移动
'''

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
