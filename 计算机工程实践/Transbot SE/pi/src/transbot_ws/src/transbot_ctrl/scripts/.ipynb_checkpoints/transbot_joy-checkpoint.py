#!/usr/bin/env python
# encoding: utf-8
import os
import time
import rospy
import getpass
import threading
from time import sleep
from sensor_msgs.msg import Joy
from transbot_msgs.srv import *
from transbot_msgs.msg import *
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class JoyTeleop:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.active = 0
        self.current_joint = False
        self.Buzzer_value = False
        self.Headlight_value = False
        self.CameraDevice = "Astra"
        self.user_name = getpass.getuser()
        self.Joy_active = False
        self.cancel_time = time.time()
        self.arm_time = time.time()
        self.RGBLight_value = 0
        self.PWMServo_X = 90
        self.PWMServo_Y = 90
        self.joint1 = 225
        self.joint2 = 45
        self.joint3 = 90
        self.joint_key1 = 0
        self.joint_key2 = 0
        self.joint_key3 = 0
        self.joint_key4 = 0
        self.Servo_leftX = 0
        self.Servo_rightB = 0
        self.Servo_downA = 0
        self.Servo_upY = 0
        self.linear_Gear = 1
        self.angular_Gear = 1
        self.velocity = Twist()
        self.PWMServo = PWMServo()
        self.rate = rospy.Rate(20)
        self.linear_speed_limit = rospy.get_param('~linear_speed_limit', 0.45)
        self.angular_speed_limit = rospy.get_param('~angular_speed_limit', 2.0)
        self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_PWMServo = rospy.Publisher("/PWMServo", PWMServo, queue_size=10)
        self.pub_Arm = rospy.Publisher("/TargetAngle", Arm, queue_size=10)
        self.pub_goal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        self.pub_JoyState = rospy.Publisher("/JoyState", JoyState, queue_size=10)
        self.pub_Adjust = rospy.Publisher("/Adjust", Adjust, queue_size=10)
        self.RobotArm_client = rospy.ServiceProxy("/CurrentAngle", RobotArm)
        self.RGBLight_client = rospy.ServiceProxy("/RGBLight", RGBLight)
        self.Buzzer_client = rospy.ServiceProxy("/Buzzer", Buzzer)
        self.CamDevice_client = rospy.ServiceProxy("/CamDevice", CamDevice)
        self.Headlight_client = rospy.ServiceProxy("/Headlight", Headlight)
        self.sub_Joy = rospy.Subscriber('/joy', Joy, self.buttonCallback)
        arm_thread = threading.Thread(target=self.analyse_PWM())
        arm_thread.setDaemon(True)
        arm_thread.start()

    def cancel(self):
        self.cmdVelPublisher.unregister()
        self.pub_PWMServo.unregister()
        self.pub_Arm.unregister()
        self.pub_goal.unregister()
        self.pub_JoyState.unregister()
        self.RobotArm_client.close()
        self.RGBLight_client.close()
        self.Buzzer_client.close()
        self.CamDevice_client.close()
        self.Headlight_client.close()
        self.sub_Joy.unregister()

    def buttonCallback(self, joy_data):
        '''
        获取无线手柄接收信号
        [1：夹紧，2：松开，左上：joint2+，左下：joint2-，左左：joint1上，左右joint1：下]
        Get wireless handset reception signal
        [1: clamp, 2: loosen, upper left: JoinT2 +, lower left: JoinT2 -, left: left: JoinT1, left and right: JoinT1]
        '''
        if not isinstance(joy_data, Joy): return
        # rospy.loginfo("joy_data.buttons:", joy_data.buttons)
        # rospy.loginfo("joy_data.axes:", joy_data.axes)
        if self.user_name == "pi": self.user_pi(joy_data)
        else: self.user_jetson(joy_data)
        # rospy.loginfo("linear_Gear: {},angular_Gear: {}".format(self.linear_Gear,self.angular_Gear))
        # rospy.loginfo("linear_speed: {},angular_speed: {}".format(linear_speed,angular_speed))

    def user_pi(self,joy_data):
        '''
        joy_data.buttons: (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)  11
        joy_data.axes: (0.0, -0.0, 1.0, -0.0, 0.0, 1.0, -0.0, -0.0) 8
        '''
        #jet [linear_speed:axes[1], angular_speed:axes[2]]
        #pi  [linear_speed:axes[1], angular_speed:axes[3]]
        linear_speed = joy_data.axes[1] * self.linear_speed_limit * self.linear_Gear
        angular_speed = joy_data.axes[3] * self.angular_speed_limit * self.angular_Gear
        if linear_speed==0:self.pub_Adjust.publish(False)
        else:self.pub_Adjust.publish(True)
        if linear_speed > self.linear_speed_limit: linear_speed = self.linear_speed_limit
        elif linear_speed < -self.linear_speed_limit: linear_speed = -self.linear_speed_limit
        if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmdVelPublisher.publish(twist)
        # [jet:buttons[9],pi:axes[5]]
        if joy_data.axes[5]==1: self.cancel_nav()
        # 档位控制
        # Gear control
        #jet [linear_Gear:13, angular_Gear:14]
        #pi  [linear_Gear:9 , angular_Gear:10]
        if joy_data.buttons[9] == 1:
            if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
        if joy_data.buttons[10] == 1:
            if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
        # 机械臂控制 joint1   joint2          joint3
        # Manipulator control joint1   joint2          joint3
        # jet [key1:axes[6], key2:axes[7], key3:buttons[6], key4:buttons[8]]
        #pi  [key1:axes[6], key2:axes[7], key3:buttons[4], key4:axes[2]]
        self.joint_key1 = joy_data.axes[6]
        self.joint_key2 = joy_data.axes[7]
        self.joint_key3 = joy_data.buttons[4]
        if joy_data.axes[2]==1.0: self.joint_key4 = 0
        else: self.joint_key4 = -joy_data.axes[2]
        # 云台舵机
        # Head steering gear
        #jet [left:3, right:1, down:0, up:4]
        #pi  [left:2, right:1, down:3, up:0]
        self.Servo_leftX = joy_data.buttons[2]
        self.Servo_rightB = joy_data.buttons[1]
        self.Servo_downA = joy_data.buttons[3]
        self.Servo_upY = joy_data.buttons[0]
        # 蜂鸣器 [jet:11,pi:7]
        # Buzzer [Jet :11, PI :7]
        if joy_data.buttons[7] != 0:
            self.Buzzer_value = not self.Buzzer_value
            self.Buzzer_srv(self.Buzzer_value)
        # 探照灯 [jet:10,pi:6]
        # Searchlight [JET :10, PI :6]
        if joy_data.buttons[6] != 0:
            self.Headlight_value = bool(1 - self.Headlight_value)
            if self.Headlight_value: self.Headlight_srv(100)
            else: self.Headlight_srv(0)
        # 流水灯 [jet:7,pi:5]
        # Flow light [JET :7, PI :5]
        if joy_data.buttons[5] != 0:
            self.RGBLight_value += 1
            if self.RGBLight_value <= 6: self.RGBLight_srv(self.RGBLight_value, 5)
            else:
                self.RGBLight_value = 0
                self.RGBLight_srv(0, 5)
        #rospy.loginfo("linear_Gear: {},angular_Gear: {}".format(self.linear_Gear,self.angular_Gear))
        #rospy.loginfo("linear_speed: {},angular_speed: {}".format(linear_speed,angular_speed))

    def user_jetson(self,joy_data):
        '''
        axes: [-0.0, -0.0, 0.0, -0.0, 0.0, 0.0]  6
        axes(8):[0:左正右负(左遥感);1:上正下负(左遥感)<->前进后退; 2:左正右负(右遥感) <->左右移动;
        3:上正下负(右遥感);4:; 5:; 6:左正右负(左排按键)<->Arm-1号; 7:上正下负(左排按键)<->Arm-2号]
        buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  12
        buttons(15):[0:A<->云台下移; 1:B<->云台右移; 2:; 3:X<->云台左移; 4:Y<->云台上移; 5:;
        6:L1<->Arm-夹爪夹紧; 7:R1<->流水灯; 8:L2<->Arm-夹爪松开;9: R2<->开关控制; 10:SELECT<->探照灯;
        11:START<->蜂鸣器; 12: ; 13:按左遥感<->线速度档位; 14:按右遥感<->角速度档位]
        axes: [-0.0, -0.0, 0.0, -0.0, 0.0, 0.0] 6
         Axes(8): (0: left positive and right negative (left remote sensing); 1: up positive and down negative (left remote sensing) <-> forward and backward; 2: left positive and right negative (right remote sensing) <-> move left and right;
         3: Up positive and down negative (right remote sensing); 4:; 5:; 6: Left positive and right negative (left row button) <->Arm-1 number; 7: Up positive and bottom negative (left row button)<-> Arm-2]
         buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 12
         buttons(15):[0:A<->PTZ move down; 1:B<->PTZ move right; 2:; 3:X<->PTZ move left; 4:Y<->PTZ Move up; 5:;
         6:L1<->Arm-Clamping jaws; 7:R1<->Flowing water lamp; 8:L2<->Arm-Clamping jaws released; 9: R2<->switch control; 10:SELECT<-> searchlight;
         11: START<-> buzzer; 12:; 13: press left remote sensing <-> linear speed gear; 14: press right remote sensing <-> angular speed gear]
        '''
        #jet [linear_speed:axes[1], angular_speed:axes[2]]
        #pi  [linear_speed:axes[1], angular_speed:axes[3]]
        linear_speed = joy_data.axes[1] * self.linear_speed_limit * self.linear_Gear
        angular_speed = joy_data.axes[2] * self.angular_speed_limit * self.angular_Gear
        if linear_speed==0:self.pub_Adjust.publish(False)
        else:self.pub_Adjust.publish(True)
        if linear_speed > self.linear_speed_limit: linear_speed = self.linear_speed_limit
        elif linear_speed < -self.linear_speed_limit: linear_speed = -self.linear_speed_limit
        if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmdVelPublisher.publish(twist)
        # 取消
        # Cancel
        if joy_data.buttons[9]==1: self.cancel_nav()
        # 档位控制
        # Gear control
        if joy_data.buttons[13] == 1:
            if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
        if joy_data.buttons[14] == 1:
            if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
        # 机械臂控制
        # Robotic arm control
        #jet [key1:axes[6], key2:axes[7], key3:buttons[6], key4:buttons[8]]
        # pi [key1:2, key2:1, key3:3, key4:0]
        self.joint_key1 = joy_data.axes[6]
        self.joint_key2 = joy_data.axes[7]
        self.joint_key3 = joy_data.buttons[6]
        self.joint_key4 = joy_data.buttons[8]
        # 云台舵机
        # PTZ servo
        #jet [left:3, right:1, down:0, up:4,]
        # pi [left:2, right:1, down:3, up:0]
        self.Servo_leftX = joy_data.buttons[3]
        self.Servo_rightB = joy_data.buttons[1]
        self.Servo_downA = joy_data.buttons[0]
        self.Servo_upY = joy_data.buttons[4]
        # 蜂鸣器 [jet:11,pi:7]
        # Buzzer [jet:11,pi:7]
        if joy_data.buttons[11] != 0:
            self.Buzzer_value = not self.Buzzer_value
            self.Buzzer_srv(self.Buzzer_value)
        # 探照灯 [jet:10,pi:6]
        # Searchlight [jet:10,pi:6]
        if joy_data.buttons[10] != 0:
            self.Headlight_value = bool(1 - self.Headlight_value)
            if self.Headlight_value: self.Headlight_srv(100)
            else: self.Headlight_srv(0)
        # 流水灯 [jet:7,pi:5]
        # Flow light [JET :7, PI :5]
        if joy_data.buttons[7] != 0:
            self.RGBLight_value += 1
            if self.RGBLight_value <= 6: self.RGBLight_srv(self.RGBLight_value, 5)
            else:
                self.RGBLight_value = 0
                self.RGBLight_srv(0, 5)

    def cancel_nav(self):
        # 发布move_base取消命令
        # Issue the move_base cancel command
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            self.Joy_active = not self.Joy_active
            self.pub_JoyState.publish(JoyState(self.Joy_active))
            self.pub_goal.publish(GoalID())
            self.cancel_time=now_time

    def PWMServo_topic(self, id, angle):
        '''
        云台舵机接口
        :param id: [1 : 云台左右, 2 : 云台上下]
        :param angle: [0, 180]
        Head steering gear interface
        :param ID: [1: left/right, 2: up/down]
        : param Angle: [0, 180]
        '''
        self.PWMServo.id = id
        self.PWMServo.angle = int(angle)
        # rospy.loginfo(self.PWMServo)
        self.pub_PWMServo.publish(self.PWMServo)

    def Buzzer_srv(self, value):
        '''
        蜂鸣器控制
        :param value:
         [  0：   关闭,
            1：   一直响,
            >=10：响xx毫秒后自动关闭（value是10的倍数) ]
        Buzzer control
        : param value:
        [0: close,
        1: Keep ringing,
        & gt;=10: automatically closes after xx milliseconds (value is a multiple of 10)]
        '''
        self.Buzzer_client.wait_for_service()
        request = BuzzerRequest()
        request.buzzer = value
        try:
            response = self.Buzzer_client.call(request)
            if isinstance(response, BuzzerResponse): return response.result
        except Exception:
            rospy.loginfo("Buzzer error")
        return False

    def Headlight_srv(self, value):
        '''
        高帧率相机前灯控制
        High frame rate camera headlight control
        :param value: [0, 100]
        '''
        self.Headlight_client.wait_for_service()
        request = HeadlightRequest()
        request.Headlight = value
        try:
            response = self.Headlight_client.call(request)
            if isinstance(response, HeadlightResponse): return response.result
        except Exception:
            rospy.loginfo("Headlight error")
        return False

    def RGBLight_srv(self, effect, speed):
        '''
        RGB可编程灯带特效展示。
        :param effect: [0：停止灯效，1：流水灯，2：跑马灯，3：呼吸灯，4：渐变灯，5：星光点点，6：电量显示]
        :param speed: [1, 10]，数值越小速度变化越快。
        RGB programmable light band special effects display.
        : Param effect: [0: stop light effect, 1: water light, 2: running horse light, 3: breathing light, 4: gradient light,       5: starlight, 6: power display]
        :param speed: [1, 10], the smaller the value, the faster the speed changes.
        '''
        self.RGBLight_client.wait_for_service()
        request = RGBLightRequest()
        request.effect = effect
        request.speed = speed
        try:
            response = self.RGBLight_client.call(request)
            if isinstance(response, RGBLightResponse): return response.result
        except Exception:
            rospy.loginfo("RGBLight error")
        return False

    def RobotArmSrv(self, value):
        '''
        获取机械臂当前角度
        Obtain the current Angle of the manipulator
        '''
        self.RobotArm_client.wait_for_service()
        request = RobotArmRequest()
        request.apply = value
        joints = []
        try:
            response = self.RobotArm_client.call(request)
            if isinstance(response, RobotArmResponse):
                for joint in response.RobotArm.joint: joints.append(joint.angle)
                self.joint1 = joints[0]
                self.joint2 = joints[1]
                self.joint3 = joints[2]
        except Exception: rospy.loginfo("robotArm error")
        print("Arm_joints: ", joints)
        return False

    def CamDevice_srv(self, value):
        '''
        获取当前接入的相机设备
        Gets the currently connected camera device
        '''
        self.CamDevice_client.wait_for_service()
        request = CamDeviceRequest()
        request.GetGev = value
        try:
            response = self.CamDevice_client.call(request)
            if isinstance(response, CamDeviceResponse):
                self.CameraDevice = response.camDevice
                rospy.loginfo(response)
        except Exception:
            rospy.loginfo("CamDevice error")
        return True

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

    def analyse_PWM(self):
        # 云台控制程序
        t = 0.01
        rospy.sleep(1)
       
        
        while not rospy.is_shutdown():
            
            if not self.joint_key1 == self.joint_key2 == self.joint_key3 == self.joint_key4 == 0:
                
                self.joint1 += self.joint_key1
                self.joint2 += self.joint_key2
                self.joint3 += self.joint_key3
                self.joint3 -= self.joint_key4
                if self.joint1 <= 0: self.joint1 = 0
                elif self.joint1 >= 225: self.joint1 = 225
                if self.joint2 <= 30: self.joint2 = 30
                elif self.joint2 >= 270: self.joint2 = 270
                if self.joint3 <= 60: self.joint3 = 60
                elif self.joint3 >= 180: self.joint3 = 180
                # rospy.loginfo("joint: ",self.joint1, self.joint2, self.joint3)
                self.arm_ctrl(t * 1000, self.joint1, self.joint2, self.joint3)
            if not self.Servo_leftX == self.Servo_rightB == self.Servo_downA == self.Servo_upY == 0:
                self.PWMServo_X += self.Servo_leftX
                self.PWMServo_X -= self.Servo_rightB
                self.PWMServo_Y -= self.Servo_downA
                self.PWMServo_Y += self.Servo_upY
                if self.PWMServo_X <= 0: self.PWMServo_X = 0
                elif self.PWMServo_X >= 180: self.PWMServo_X = 180
                if self.PWMServo_Y <= 55: self.PWMServo_Y = 55
                elif self.PWMServo_Y >= 150: self.PWMServo_Y = 150
                self.PWMServo_topic(1, self.PWMServo_X)
                self.PWMServo_topic(2, self.PWMServo_Y)
            sleep(t)


if __name__ == '__main__':
    rospy.init_node('transbot_joy')
    joy = JoyTeleop()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')
