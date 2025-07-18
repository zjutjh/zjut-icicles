#!/usr/bin/env python2
# coding:utf-8
import tf
import numpy as np
#from laser_Avoidance import *
#from laser_Warning import *
#from laser_Tracker import *
from Program_Ctrl import *
from transbot_msgs.msg import *
from transbot_msgs.srv import *
#from sensor_msgs.msg import LaserScan
from math import radians, copysign, sqrt, pow
from geometry_msgs.msg import Twist, Point, Quaternion
from transform_utils import quat_to_angle, normalize_angle
from transbot_bringup.cfg import PatrolParamConfig
from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client

class TransbotPatrol():
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(20)
        self.moving = True
        self.SetLoop = False
        self.Switch = False
        self.command_src = "finish"
        self.Command = "finish"
        self.Function = ""
        self.reverse = 1
        self.warning = 1
        self.Length = 1.0
        self.Angle = 360
        self.ResponseDist = 0.7
        self.LaserAngle = 20
        self.Linear = 0.2
        self.Angular = 1.0
        self.RotationScaling = 1.0
        self.LineScaling = 1.0
        self.LineTolerance = 0.1
        self.RotationTolerance = 0.3
        #self.laser_avoid = laserAvoid()
        #self.laser_warning = laserWarning()
        #self.laser_tracker = laserTracker()
        self.program_ctrl = ProgramCtrl()
        self.tf_listener = tf.TransformListener()
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.srv_patrol = rospy.Service('/Patrol', Patrol, self.PatrolCallback)
        self.sub_patrolwarning = rospy.Subscriber('/PatrolWarning', PatrolWarning, self.PatrolWarningCallback)
        Server(PatrolParamConfig, self.dynamic_reconfigure_callback)
       # self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.registerScan)
        self.dyn_client = dynamic_reconfigure.client.Client("TransbotPatrol", timeout=60)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
        rospy.loginfo("Bring up rqt_reconfigure to control the Transbot.")

    def dynamic_reconfigure_callback(self, config, level):
        '''
        巡逻玩法设置
        :param Commond: 巡逻指令 [LengthTest, AngleTest, Triangle, Square, Parallelogram, Circle]
        :param Length: 运行长度（米）[0.3,3]
        :param Linear: 运行速度（米/秒）[0.2,1.2]
        :param SetLoop: 是否循环巡逻 [True or False]
        :param ResponseDist: 激光雷达避障距离[0.4,8]
        :param LaserAngle: 激光雷达避障角度[10,180]
        :param LineScaling: 直线距离缩放比例
        :param RotationScaling: 旋转角度缩放比例
        :param LineTolerance: 允许的直线距离误差
        :param RotationTolerance: 允许的旋转角度误差
        :param Switch: 巡逻功能【开始/暂停】
		
         Patrol gameplay settings
         :param Commond: Patrol command [LengthTest, AngleTest, Triangle, Square, Parallelogram, Circle]
         :param Length: Running length (m) [0.3,3]
         :param Linear: Running speed (m/s) [0.2,1.2]
         :param SetLoop: Whether to patrol in a loop [True or False]
         :param ResponseDist: Lidar avoid distance [0.4,8]
         :param LaserAngle: Lidar ravoidance angle [10,180]
         :param LineScaling: Linear distance scaling
         :param RotationScaling: Rotation angle scaling
         :param LineTolerance: Allowable linear distance error
         :param RotationTolerance: Allowable rotation angle error
         :param Switch: Patrol function [Start/Pause]
        '''
        self.Linear = config['Linear']
        self.Angular = config['Angular']
        self.Length = config['Length']
        self.Angle = config['Angle']
        #self.ResponseDist = config['ResponseDist']
        #self.LaserAngle = config['LaserAngle']
        self.LineScaling = config['LineScaling']
        self.RotationScaling = config['RotationScaling']
        self.LineTolerance = config['LineTolerance']
        self.RotationTolerance = config['RotationTolerance']
        self.Command = config['Command']
        self.command_src = config['Command']
        self.SetLoop = config['SetLoop']
        self.Switch = config['Switch']
        if self.Switch == True: print ("Command: ", self.Command)
        return config

    def PatrolWarningCallback(self, msg):
        if not isinstance(msg, PatrolWarning): return
        self.Function = msg.Function
        self.LaserAngle = msg.LaserAngle
        self.ResponseDist = msg.ResponseDist
        self.Linear = msg.speed
        if self.Function == "stop" : self.Command = "finish"
        print ("Function: ",self.Function)

    def registerScan(self, scan_data):
        if self.program_ctrl.Joy_active: return
        ranges = np.array(scan_data.ranges)
        sortedIndices = np.argsort(ranges)
        self.warning = 1
        back_state = True
        Right_warning = 0
        Left_warning = 0
        front_warning = 0
        frontMinDistList = []
        frontIDList = []
        minDistList = []
        minDistIDList = []
      
    def Triangle(self, index, angle):
        index += 1
        advancing = False
        sleep(0.5)
        if index == 1 or index == 3: advancing = self.advancing(self.Length)
        elif index == 2:
            side = sqrt(pow((self.Length), 2) * 2)
            advancing = self.advancing(side)
        sleep(0.5)
        if advancing == True:
            spin = self.Spin(angle)
            if spin == True:
                if index == 1:
                    self.Triangle(index, 135)
                elif index == 2:
                    self.Triangle(index, 90)
                else:
                    self.Command = "finish"
                    return

    def Square(self, index, angle):
        index += 1
        if index == 5:
            self.Command = "finish"
            return
        sleep(0.5)
        advancing = self.advancing(self.Length)
        sleep(0.5)
        if advancing == True:
            spin = self.Spin(angle)
            if spin == True:
                if index == 2:
                    self.Square(index, 90)
                else:
                    self.Square(index, 90)

    def Parallelogram(self, index, angle):
        index += 1
        if index == 5:
            self.Command = "finish"
            return
        sleep(0.5)
        advancing = self.advancing(self.Length)
        sleep(0.5)
        if advancing == True:
            spin = self.Spin(angle)
            if spin == True:
                if index == 2:
                    self.Parallelogram(index, 120)
                else:
                    self.Parallelogram(index, 60)

    def PatrolCallback(self, request):
        if not isinstance(request, PatrolRequest): return
        print("request.Commond",request.Commond)
        if request.Commond != "":
            self.Function = patrol
            self.Command = request.Commond
        if request.LineScaling != 0: self.LineScaling = request.LineScaling
        if request.RotationScaling != 0: self.RotationScaling = request.RotationScaling
        response = PatrolResponse()
        index = 0
        self.Switch = True
        print ("Commond: ", request.Commond)
        if self.Command == "LengthTest": self.advancing(self.Length)
        elif self.Command == "AngleTest": self.Spin(self.Angle)
        elif self.Command == "Triangle": self.Triangle(index, 135)
        elif self.Command == "Square": self.Square(index, 90)
        elif self.Command == "Parallelogram": self.Parallelogram(index, 120)
        elif self.Command == "Circle": self.Spin(360)
        self.r.sleep()
        self.program_ctrl.pub_cmdVel.publish(Twist())
        self.Switch = False
        response.result = True
        return response

    def process(self):
        index = 0
        while not rospy.is_shutdown():
            if self.Switch==True:
                if self.Command == "LengthTest":
                    advancing = self.advancing(self.Length)
                    if advancing == True: self.Command = "finish"
                elif self.Command == "AngleTest":
                    spin = self.Spin(self.Angle)
                    if spin == True: self.Command = "finish"
                elif self.Command == "Triangle": self.Triangle(index, 135)
                elif self.Command == "Square": self.Square(index, 90)
                elif self.Command == "Parallelogram": self.Parallelogram(index, 120)
                elif self.Command == "Circle":
                    self.RotationScaling = 0.9
                    spin = self.Spin(360)
                    if spin == True: self.Command = "finish"
                if self.Command == "finish":
                    self.program_ctrl.pub_cmdVel.publish(Twist())
                    if self.SetLoop == False:
                        params = {'Switch': False}
                        self.dyn_client.update_configuration(params)
                    else: self.Command = self.command_src
            self.r.sleep()
        self.program_ctrl.pub_cmdVel.publish(Twist())

    def Spin(self, angle):
        target_angle = radians(angle)
        odom_angle = self.get_odom_angle()
        last_angle = odom_angle
        turn_angle = 0
        error = target_angle - turn_angle
        # Alternate directions between tests
        while abs(error) > self.RotationTolerance:
            # Get the current rotation angle from tf
            odom_angle = self.get_odom_angle()
            # Compute how far we have gone since the last measurement
            delta_angle = self.RotationScaling * normalize_angle(odom_angle - last_angle)
            # Add to our total angle so far
            turn_angle += delta_angle
            # Compute the new error
            error = target_angle - turn_angle
            # Store the current angle for the next comparison
            last_angle = odom_angle
            # print("self.RotationScaling: {},target_angle: {},turn_angle: {}".format(
            #     self.RotationScaling, target_angle, turn_angle))
            move_cmd = Twist()
            if self.program_ctrl.Joy_active or self.Switch == False or self.warning > 10:
                if self.moving == True:
                    self.program_ctrl.pub_cmdVel.publish(Twist())
                    self.moving = False
                continue
            else:
                if self.Command == "Circle":
                    adjust = 6.0 / 5.0
                    length = self.Linear * adjust / self.Length
                    move_cmd.linear.x = self.Linear
                    move_cmd.angular.z = copysign(length, error)
                else: move_cmd.angular.z = copysign(self.Angular, error)
                self.program_ctrl.pub_cmdVel.publish(move_cmd)
            self.moving = True
            if self.Function == "stop" : break
            self.r.sleep()
        # Stop the robot
        self.program_ctrl.pub_cmdVel.publish(Twist())
        return True

    def advancing(self, target_distance):
        position = self.get_position()
        x_start, y_start = position.x, position.y
        while not rospy.is_shutdown():
            # Get the current position from the tf transform between the odom and base frames
            self.position = self.get_position()
            # Compute the Euclidean distance from the target point
            distance = sqrt(pow((self.position.x - x_start), 2) +
                            pow((self.position.y - y_start), 2))
            distance *= self.LineScaling
            # How close are we?
            error = distance - target_distance
            # print ("target_distance: {},distance: {}".format(target_distance, distance))
            # Are we close enough?
            move_cmd = Twist()
            move_cmd.linear.x = self.Linear
            if abs(error) <= self.LineTolerance:
                self.program_ctrl.pub_cmdVel.publish(Twist())
                return True
            if self.program_ctrl.Joy_active or self.Switch == False or self.warning > 10:
                if self.moving == True:
                    self.program_ctrl.pub_cmdVel.publish(Twist())
                    self.moving = False
                continue
            else: self.program_ctrl.pub_cmdVel.publish(move_cmd)
            self.moving = True
            if self.Function == "stop" : break
        return False

    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))

    def get_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans)

    def cancel(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.program_ctrl.pub_cmdVel.publish(Twist())
        self.tf_listener.clear()
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('TransbotPatrol', anonymous=False)
    patrol = TransbotPatrol()
    patrol.process()
    patrol.cancel()
    rospy.spin()

