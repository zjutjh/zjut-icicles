#!/usr/bin/env python
# encoding: utf-8
import sys
sys.path.append("/root/Transbot/transbot")
import rospy
import random
import threading
from math import pi
from time import sleep
from transbot_msgs.msg import *
from transbot_msgs.srv import *
from sensor_msgs.msg import Imu
from Transbot_Lib import Transbot
from geometry_msgs.msg import Twist
from arm_transbot import Transbot_ARM
from dynamic_reconfigure.server import Server
from transbot_bringup.cfg import PIDparamConfig
from std_msgs.msg import Bool

class transbot_driver:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.bot_arm = Transbot_ARM()
        bot_arm_offset = self.bot_arm.get_arm_offset()
        self.bot = Transbot(arm_offset=bot_arm_offset)
        # 弧度转角度
	    # Radians turn angle
        self.RA2DE = 180 / pi
        imu = rospy.get_param("imu", "/transbot/imu")
        vel = rospy.get_param("vel", "/transbot/get_vel")
        self.CameraDevice = rospy.get_param("CameraDevice", "astra")
        self.linear_max = rospy.get_param('~linear_speed_limit', 0.4)
        self.linear_min = rospy.get_param('~linear_speed_limit', 0.0)
        self.angular_max = rospy.get_param('~angular_speed_limit', 2.0)
        self.angular_min = rospy.get_param('~angular_speed_limit', 0.0)
        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=10)
        self.sub_TargetAngle = rospy.Subscriber("/TargetAngle", Arm, self.sub_armcallback, queue_size=10)
        self.sub_PWMServo = rospy.Subscriber("/PWMServo", PWMServo, self.sub_PWMServocallback, queue_size=10)
        self.sub_Adjust = rospy.Subscriber("/Adjust", Adjust, self.sub_Adjustcallback, queue_size=10)
        self.srv_CurrentAngle = rospy.Service("/CurrentAngle", RobotArm, self.RobotArmcallback)
        self.srv_RGBLight = rospy.Service("/RGBLight", RGBLight, self.RGBLightcallback)
        self.srv_Buzzer = rospy.Subscriber("Buzzer", Bool, self.Buzzercallback, queue_size=100)
        self.drv_Buzzer = rospy.Service("/Buzzer", Buzzer, self.DrvBuzzercallback)
        self.srv_Headlight = rospy.Service("/Headlight", Headlight, self.Headlightcallback)
        self.ediPublisher = rospy.Publisher('/edition', Edition, queue_size=10)
        self.volPublisher = rospy.Publisher("/voltage", Battery, queue_size=10)
        self.velPublisher = rospy.Publisher(vel, Twist, queue_size=10)
        self.imuPublisher = rospy.Publisher(imu, Imu, queue_size=10)
        self.dyn_server = Server(PIDparamConfig, self.dynamic_reconfigure_callback)
        self.bot.create_receive_threading()
        self.bot.set_uart_servo_angle(9, 90)

    def cancel(self):
        self.srv_CurrentAngle.shutdown()
        self.srv_RGBLight.shutdown()
        self.srv_Buzzer.unregister()
        self.drv_Buzzer.shutdown()
        self.srv_Headlight.shutdown()
        self.velPublisher.unregister()
        self.imuPublisher.unregister()
        self.volPublisher.unregister()
        self.sub_cmd_vel.unregister()
        self.sub_TargetAngle.unregister()
        self.sub_PWMServo.unregister()
        # Always stop the robot when shutting down the node
        rospy.loginfo("Close the robot...")
        rospy.sleep(1)

    def RobotArmcallback(self, request):
        # 服务端，机械臂当前关节角度
        # Server, the current joint angle of the robotic arm   
        if not isinstance(request, RobotArmRequest): return
        # print ("request: ",request)
        response = RobotArmResponse()
        joints = self.bot.get_uart_servo_angle_array()
        for i in range(3):
            joint = Joint()
            joint.id = i + 7
            joint.angle = joints[i]
            joint.run_time = 500
            if joints[i] < 0: continue
            else: response.RobotArm.joint.append(joint)
            # print ("id: {},angle: {}".format(joint.id, joint.angle))
        return response

    def pub_data(self):
        # 发布小车运动速度、陀螺仪数据、电池电压
	## Publish the speed of the car, gyroscope data, and battery voltage
        while not rospy.is_shutdown():
            sleep(0.05)
            ax, ay, az = self.bot.get_accelerometer_data()
            gx, gy, gz = self.bot.get_gyroscope_data()
            velocity, angular = self.bot.get_motion_data()
            voltage = self.bot.get_battery_voltage()
            battery = Battery()
            battery.Voltage = voltage
            self.volPublisher.publish(battery)
            robot_edition = self.bot.get_version()
            edition = Edition()
            edition.edition = robot_edition
            self.ediPublisher.publish(edition)
            # rospy.loginfo("battery: {}".format(battery))
            # 发布陀螺仪的数据
	        # Publish gyroscope data
            imu = Imu()
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            self.imuPublisher.publish(imu)
            # 将小车当前的线速度和角速度发布出去
	        # Publish the current linear velocity and angular velocity of the car
            twist = Twist()
            twist.linear.x = velocity
            twist.angular.z = angular
            # print(ax, ay, az, gx, gy, gz)
            # rospy.loginfo("velocity: {}, angular: {}".format(twist.linear.x, twist.angular.z))
            self.velPublisher.publish(twist)

    def sub_Adjustcallback(self, msg):
        # 机械臂控制，订阅者回调函数
	    # Robotic arm control, subscriber callback function
        if not isinstance(msg, Adjust): return
        self.bot.set_imu_adjust(msg.adjust)

    def sub_armcallback(self, msg):
        # 机械臂控制，订阅者回调函数
	    # Robotic arm control, subscriber callback function
        if not isinstance(msg, Arm): return
        for joint in msg.joint:
            if not isinstance(joint, Joint): continue
            # print "joint: ", joint
            if joint.run_time != 0: self.bot.set_uart_servo_angle(joint.id, joint.angle, joint.run_time)

    def sub_PWMServocallback(self, msg):
        # 云台控制，订阅者回调函数
	    # PWMServo control, subscriber callback function
        PWMServo_angle = int(msg.angle)
        if msg.id == 2:
            if PWMServo_angle < 10: PWMServo_angle = 10
            if PWMServo_angle > 140: PWMServo_angle = 140
        self.bot.set_pwm_servo(msg.id, PWMServo_angle)

    def cmd_vel_callback(self, msg):
        # 小车运动控制，订阅者回调函数
	    # Car motion control, subscriber callback function
        if not isinstance(msg, Twist): return
        # 下发线速度和角速度
	    # Issue linear velocity and angular velocity
        velocity = msg.linear.x
        angular = msg.angular.z
        # 小车运动控制,velocity=[-0.45, 0.45], angular=[2, 2]
	    # Trolley motion control,velocity=[-0.45, 0.45], angular=[2, 2]
        if velocity > self.linear_max:
            velocity = self.linear_max
        elif velocity < -self.linear_max:
            velocity = -self.linear_max
        elif -self.linear_min < velocity < 0:
            velocity = -self.linear_min
        elif 0 < velocity < self.linear_min:
            velocity = self.linear_min
        if angular > self.angular_max:
            angular = self.angular_max
        elif angular < -self.angular_max:
            angular = -self.angular_max
        elif -self.angular_min < angular < 0:
            angular = -self.angular_min
        elif 0 < angular < self.angular_min:
            angular = self.angular_min
        # rospy.loginfo("cmd_vel: {}, cmd_ang: {}".format(velocity, angular))
        self.bot.set_car_motion(velocity, angular)

    def RGBLightcallback(self, request):
        # 流水灯控制，服务端回调函数
	    # RGBLight control, server callback function
        if not isinstance(request, RGBLightRequest): return
        # print (request.effect, request.speed)
        self.bot.set_colorful_effect(request.effect, request.speed, parm=1)
        response = RGBLightResponse()
        response.result = True
        return response

    def Buzzercallback(self, msg):
        # 蜂鸣器控制回调函数
	    # Buzzer control, callback function
        if not isinstance(msg, Bool): return
        print ("Buzzer: ", msg.data)
        if msg.data:
            for i in range(3):
                self.bot.set_beep(1)
                sleep(0.01)
        else:
            for i in range(3):
                self.bot.set_beep(0)
                sleep(0.01)
    def DrvBuzzercallback(self, request):
        # 蜂鸣器控制回调函数
	    # Buzzer control, callback function
        if not isinstance(request, BuzzerRequest): return
        self.bot.set_beep(request.buzzer)
        response = BuzzerResponse()
        response.result = True
        return response               

    def Headlightcallback(self, request):
        # 探照灯控制，服务端回调函数
	    # Searchlight control, server callback function
        if not isinstance(request, HeadlightRequest): return
        self.bot.set_floodlight(request.Headlight)
        response = HeadlightResponse()
        response.result = True
        return response

    def dynamic_reconfigure_callback(self, config, level):
        # self.bot.set_pid_param(config['Kp'], config['Ki'], config['Kd'])
        print(config['Kp'], config['Ki'], config['Kd'])
        self.linear_max = config['linear_max']
        self.linear_min = config['linear_min']
        self.angular_max = config['angular_max']
        self.angular_min = config['angular_min']
        return config


if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        driver = transbot_driver()
        driver.pub_data()
        rospy.spin()
    except:
        rospy.loginfo("Final!!!")
