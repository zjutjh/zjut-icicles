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
        self.media_ros.pub_arm([90, 135, 0, 45, 90, 90])
        self.event = threading.Event()
        self.event.set()


    def process(self, frame):
        frame = cv.flip(frame, 1)
        if self.media_ros.Joy_active:
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
        #fingers = self.hand_detector.fingersUp(lmList)
        self.hand_detector.draw = True
        
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
