#!/usr/bin/env python
# coding: utf-8
import sys
import time
import rospy
import cv2
import numpy as np
import random
from transbot_msgs.msg import *
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
#from Transbot_Lib import Transbot
import threading
from transbot_facetracker.cfg import FaceTrackerPIDConfig
from mono_common import *



class faceTracker:
    def __init__(self):
        rospy.init_node("faceTracker", anonymous=False)
        rospy.on_shutdown(self.cancel)
        self.target_servox = 90
        self.target_servoy = 30
        #rospy.on_shutdown(self.cancel)
        self.PWM_init = False
        self.PWMServo = PWMServo()
        self.pub_PWMServo = rospy.Publisher("/PWMServo", PWMServo, queue_size=50)
        Server(FaceTrackerPIDConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client("/faceTracker", timeout=60)
        self.PWM_Reset()
        self.mono_PID = (20, 0, 2)
        self.scale = 1000
        self.img_flip = False
        self.PID_init()

        
        
    def dynamic_reconfigure_callback(self, config, level):
        self.scale = config['scale']
        self.mono_PID = (config['Kp'], config['Ki'], config['Kd'])
        #self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          #(config['Hmax'], config['Smax'], config['Vmax']))
        self.PID_init()
        return config
        
                
    def PWM_Reset(self):
        self.PWM_init = True
        self.PWMServo_topic(1, 89)
        rospy.sleep(0.1)
        self.PWMServo_topic(2, 30)
        rospy.sleep(0.1)
        self.PWMServo_topic(1, 89)
        
    def PWM_init(self):
        self.PWM_init = True
        self.PWMServo_topic(1, 89)
        rospy.sleep(0.1)
        self.PWMServo_topic(2, 90)
        rospy.sleep(0.1)
        self.PWMServo_topic(1, 89)        

    def PWMServo_topic(self, id, angle):
        '''
        PWMservo云台舵机接口
        :param id: [1 :servo left/right 云台左右, 2 :servo up/down 云台上下]
        :param angle: [0, 180]
        '''
        self.PWMServo.id = id
        self.PWMServo.angle = int(angle)
        self.pub_PWMServo.publish(self.PWMServo)
        # rospy.loginfo("pub PWMServo succes!!!")

    def cancel(self):
        self.pub_PWMServo.unregister()



    def PID_init(self):
        #self.mono_PID = (20, 0, 2)
        #self.scale = 1000
        self.PID_controller = simplePID(
            [0, 0],
            [self.mono_PID[0] / float(self.scale), self.mono_PID[0] / float(self.scale)],
            [self.mono_PID[1] / float(self.scale), self.mono_PID[1] / float(self.scale)],
            [self.mono_PID[2] / float(self.scale), self.mono_PID[2] / float(self.scale)])




    def execute(self, point_x, point_y):
        # rospy.loginfo("point_x: {}, point_y: {}".format(point_x, point_y))
        [x_Pid, y_Pid] = self.PID_controller.update([point_x - 320, point_y - 240])
        if self.img_flip == True:
            self.target_servox += x_Pid
            self.target_servoy += y_Pid
        else:
            self.target_servox -= x_Pid
            self.target_servoy += y_Pid
        if self.target_servox >= 180:
            self.target_servox = 180
        elif self.target_servox <= 0:
            self.target_servox = 0
        if self.target_servoy >= 180:
            self.target_servoy = 180
        elif self.target_servoy <= 0:
            self.target_servoy = 0
        # rospy.loginfo("target_servox: {}, target_servoy: {}".format(self.target_servox, self.target_servoy))
        self.PWMServo_topic(1, self.target_servox)
        self.PWMServo_topic(2, self.target_servoy)


class simplePID:
    '''very simple discrete PID controller'''

    def __init__(self, target, P, I, D):
        '''Create a discrete PID controller
        each of the parameters may be a vector if they have the same length
        Args:
        target (double) -- the target value(s)
        P, I, D (double)-- the PID parameter
        '''
        # check if parameter shapes are compatabile.
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        rospy.loginfo('P:{}, I:{}, D:{}'.format(P, I, D))
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
        self.setPoint = np.array(target)
        self.integrator_max = float('inf')

    def update(self, current_value):
        '''Updates the PID controller.
        Args:
            current_value (double): vector/number of same legth as the target given in the constructor
        Returns:
            controll signal (double): vector of same length as the target
        '''
        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            # the PID was called for the first time. we don't know the deltaT yet
            # no controll signal is applied
            self.timeOfLastCall = time.clock()
            return np.zeros(np.size(current_value))
        error = self.setPoint - current_value
        P = error
        currentTime = time.clock()
        deltaT = (currentTime - self.timeOfLastCall)
        # integral of the error is current error * time since last update
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        # derivative is difference in error / time since last update
        D = (error - self.last_error) / deltaT
        self.last_error = error
        self.timeOfLastCall = currentTime
        # return controll signal
        return self.Kp * P + self.Ki * I + self.Kd * D



    #def 




if __name__ == '__main__':
    global m
    m=0
    global n
    n=0
    mono_Tracker = faceTracker()
    #time.sleep(1)
    mono_Tracker.PWM_Reset()
    #mono_Tracker.execute(640,480)
    capture = cv2.VideoCapture(0)
    print("capture get FPS : ", capture.get(cv2.CAP_PROP_FPS))
    while capture.isOpened():
            
            start = time.time()
            ret, frame = capture.read()
            action = cv2.waitKey(10) & 0xFF
            end = time.time()
            fps = 1 / (end - start)
            
            ret, frame = capture.read()
            frame = cv2.flip(frame, 1)
        	#显示每一帧
            face_patterns = cv2.CascadeClassifier('/root/transbot_ws/src/transbot_facetracker/haarcascade_frontalface_default.xml')
            faces = face_patterns.detectMultiScale(frame , scaleFactor=1.1, minNeighbors=5, minSize=(100, 100))

            for (x, y, w, h) in faces:
            	m= x 
            	n= y 
            	#print("x",m)
            	#print("y",n)
            	cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.imshow("img", frame)
            c = cv2.waitKey(1)
            #if cv2.imshow("img", frame) :
            mono_Tracker.execute(m,n)            
            #cv2.imshow('frame', frame)
            if action == ord('q') or action == 113: break
    
    capture.release()
    cv2.destroyAllWindows()





