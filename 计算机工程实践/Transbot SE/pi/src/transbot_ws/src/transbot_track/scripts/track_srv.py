#!/usr/bin/env python
# coding: utf-8
import time
import rospy
import cv2 as cv
from track_common import simplePID
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from transbot_msgs.msg import * #JoyState, Position,Arm
from transbot_msgs.srv import *
from dynamic_reconfigure.server import Server
from transbot_track.cfg import TrackFollowPIDConfig


class track_src:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.circle = (0, 0, 0)
        self.minDist = 60
        
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_time = 0
        self.prev_dist = 0
        self.prev_angular = 0
        self.prev_joint = 0
        self.Joy_active = False
        self.Robot_Run = False
        self.dist = []
        self.sub_JoyState = rospy.Subscriber('/JoyState', JoyState, self.JoyStateCallback)
        self.sub_position = rospy.Subscriber("/Current_point", Position, self.positionCallback)
        self.pub_Arm = rospy.Publisher("/TargetAngle", Arm, queue_size=10)
        self.PWMServo = PWMServo()
        self.pub_PWMServo = rospy.Publisher("/PWMServo", PWMServo, queue_size=50)
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        Server(TrackFollowPIDConfig, self.TrackFollowPID_callback)
        self.linear_PID = (3.0, 0.0, 1.0)
        self.angular_PID = (0.3, 0.0, 1.0)
        self.PWM_Reset()
        self.scale = 1000
        self.PID_init()
        t=0.01
        self.arm_ctrl(t * 1000, 225, 65,30)
        
       
        
        
    

    def TrackFollowPID_callback(self, config, level):
        self.linear_PID = (config['linear_Kp'], config['linear_Ki'], config['linear_Kd'])
        self.angular_PID = (config['angular_Kp'], config['angular_Ki'], config['angular_Kd'])
        
        self.minDist = config['minDist'] * 10
        print ("linear_PID: ", self.linear_PID)
        print ("angular_PID: ", self.angular_PID)
        print self.minDist
        self.PID_init()
        return config

    def PID_init(self):
        self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)
        
    def arm_ctrl(self, run_time,j1, j2, j3):
        '''
        发布机械臂控制角度
        :param run_time: 运行时间
        :param j1: 关节1
        :param j2: 关节2
        :param j3: 关节3
        Release manipulator control Angle
        :param run_time: indicates the running time
        : Param J1: joint 1
        : Param J2: Joint 2
        : Param J3: Joint 3
        '''
        robot_arm = Arm()
        joint1 = Joint()
        joint2 = Joint()
        joint3 = Joint()
        joint1.id = 7
        joint2.id = 8
        joint3.id = 9
        joint1.angle = j1
        joint2.angle = j2
        joint3.angle = j3
        joint1.run_time = int(run_time)
        joint2.run_time = int(run_time)
        joint3.run_time = int(run_time)
        robot_arm.joint.append(joint1)
        robot_arm.joint.append(joint2)
        robot_arm.joint.append(joint3)
        # rospy.loginfo("robot_arm: ", robot_arm)
        self.pub_Arm.publish(robot_arm)        
        
    
    def PWM_Reset(self):
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
        
        
    def execute(self, point_x, point_y,dist):
        t=0.01
        print(dist)#
        #print(point_y)
        
        
        if (dist-self.minDist) >= 0:
            self.Robot_Run =False
            linear_x = 0
            
        if abs(self.prev_angular - point_x) > 300:
            self.prev_angular = point_x
            return
        if self.Joy_active == True: return
    
        if (dist-self.minDist) <= 0:
            linear_x = -(self.linear_pid.compute(dist, 1000))
        
        angular_z = self.angular_pid.compute(320, point_x)
        if point_y>330 and point_y<440:
            self.arm_ctrl(t * 1000, 225,40 , 30)
        if point_y>130 and point_y < 330:
            self.arm_ctrl(t * 1000, 225,65 , 30)  
        if point_y>60 and point_y<130:
            self.arm_ctrl(t * 1000, 225,110 , 30)    
        
        
        if abs(point_x - 320.0) < 30: angular_z = 0
        twist = Twist()
        twist.angular.z = angular_z
        twist.linear.x = linear_x
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True
        
        
        
        # rospy.loginfo(
        #     "point_x: {},dist: {},linear_x: {}, angular_z: {}".format(
        #         point_x, dist, twist.linear.x, twist.angular.z))

    def JoyStateCallback(self, msg):
        if not isinstance(msg, JoyState): return
        self.Joy_active = msg.state
        self.pub_cmdVel.publish(Twist())

    def positionCallback(self, msg):
        if not isinstance(msg, Position): return
        self.Center_x = msg.angleX
        self.Center_y = msg.angleY
        self.Center_r = msg.distance
        if self.Center_r != 0:
            self.execute(self.Center_x,self.Center_y, self.Center_r)
        else:
            if self.Robot_Run ==True:
                self.pub_cmdVel.publish(Twist())
                
                self.Robot_Run = False
        

    def cleanup(self):
        self.pub_cmdVel.publish(Twist())
        self.pub_Arm.unregister()
        self.sub_JoyState.unregister()
        self.sub_position.unregister()
        self.pub_cmdVel.unregister()
        self.pub_PWMServo.unregister()
        print ("Shutting down this node.")
        cv.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("track_src", anonymous=False)
    track_src()    
    rospy.spin()
   
