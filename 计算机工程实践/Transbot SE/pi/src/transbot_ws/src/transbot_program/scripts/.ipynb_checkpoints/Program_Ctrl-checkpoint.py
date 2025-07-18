#!/usr/bin/env python
# encoding: utf-8
import base64
import rospy
from time import sleep
from transbot_msgs.srv import *
from transbot_msgs.msg import *
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class ProgramCtrl:
    def __init__(self):
        self.Arm = Arm()
        self.joint = Joint()
        self.patrol = Patrol()
        self.PWM_init = True # 舵机初始化  Servo initialization
        self.Joy_active = False
        self.general = General()
        self.position = Position()
        self.PWMServo = PWMServo()
        self.PatrolWarning = PatrolWarning()
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_Arm = rospy.Publisher("/TargetAngle", Arm, queue_size=50)
        self.pub_PWMServo = rospy.Publisher("/PWMServo", PWMServo, queue_size=50)
        self.pub_PatrolWarning = rospy.Publisher("/PatrolWarning", PatrolWarning, queue_size=50)
        self.pub_Graphics = rospy.Publisher("/Graphics_topic", General, queue_size=50)
        self.pub_autopilot = rospy.Publisher("/Autopilot", General, queue_size=50)
        self.pub_colorTracker = rospy.Publisher("/colorTracker", General, queue_size=50)
        self.pub_goal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        self.pub_image = rospy.Publisher('/image_msg', Image_Msg, queue_size=10)
        self.pub_position = rospy.Publisher("/Current_point", Position, queue_size=10)
        self.sub_JoyState = rospy.Subscriber('/JoyState', JoyState, self.JoyStateCallback)
        self.CamDevice_client = rospy.ServiceProxy("/CamDevice", CamDevice)
        self.Patrol_client = rospy.ServiceProxy("/Patrol", Patrol)
        self.RGBLight_client = rospy.ServiceProxy("/RGBLight", RGBLight)
        self.Buzzer_client = rospy.ServiceProxy("/Buzzer", Buzzer)
        self.Headlight_client = rospy.ServiceProxy("/Headlight", Headlight)

    def cancel(self):
        self.pub_Arm.unregister()
        self.pub_PWMServo.unregister()
        self.pub_PatrolWarning.unregister()
        self.pub_Graphics.unregister()
        self.pub_autopilot.unregister()
        self.pub_colorTracker.unregister()
        self.pub_goal.unregister()
        self.pub_image.unregister()
        self.RGBLight_client.close()
        self.Buzzer_client.close()
        self.Headlight_client.close()
        self.Patrol_client.close()

    def pub_imgMsg(self, frame):
        pic_base64 = base64.b64encode(frame)
        image = Image_Msg()
        size = frame.shape
        image.height = size[0]
        image.width = size[1]
        image.channels = size[2]
        image.data = pic_base64
        self.pub_image.publish(image)

    # 取消路径规划  Cancel path planning
    def cancel_nav(self):
        for i in range(2):
            self.pub_goal.publish(GoalID())
            sleep(0.1)
        return True

    def JoyStateCallback(self, msg):
        if not isinstance(msg, JoyState): return
        self.Joy_active = msg.state

    def Position_topic(self, center_x, center_y, center_r):
        self.position.angleX = center_x
        self.position.angleY = center_y
        self.position.distance = center_r
        self.pub_position.publish(self.position)

    def PatrolWarning_topic(self, Function, speed, LaserAngle, ResponseDist):
        print ("Function: ",Function)
        self.PatrolWarning.speed = speed
        self.PatrolWarning.Function = Function
        self.PatrolWarning.LaserAngle = LaserAngle
        self.PatrolWarning.ResponseDist = ResponseDist
        for i in range(3): self.pub_PatrolWarning.publish(self.PatrolWarning)

    def ColorTrackert_topic(self, value):
        '''
        物体追踪 Object tracking
        :param value: [getHSV,HSVOK,execute]
        :return:
        '''
        self.general.TrackState = value
        self.pub_colorTracker.publish(self.general)

    def Autopilo_topict(self, value):
        '''
        自动驾驶 Autopilot
        :param value: [getHSV,HSVOK,execute]
        :return:
        '''
        self.general.TrackState = value
        self.pub_autopilot.publish(self.general)

    def Graphics_topic(self, value):
        '''
        AR图像选择 AR image selection
        :param value:
        [ParallelBars, Stickman, Bench,
            Desk, Knife, Arrow,
            Ball, TableTennisTable, WindMill,
            Parallelogram, Rectangle, Triangle ]
        '''
        self.general.Graphics = value
        self.pub_Graphics.publish(self.general)

    def PWMServo_topic(self, id, angle):
        '''
        云台舵机接口 Gimbal Servo Interface
        :param id: [1, 6]
        :param angle: [0, 180]
        '''
        self.PWMServo.id = id
        self.PWMServo.angle = int(angle)
        self.pub_PWMServo.publish(self.PWMServo)

    def Arm_topic(self, joint):
        '''
        机械臂控制[[id,角度,运行时间],[],[]] Robotic arm control [[id, angle, running time],[],[]]
        :param joint: [[7,90,500],[8,90,500],[9,90,500]]
        '''
        for i in joint:
            self.joint.id = i[0]
            self.joint.angle = i[1]
            self.joint.run_time = i[2]
        self.Arm.joint.append(self.joint)
        self.pub_Arm.publish(self.Arm)

    def Buzzer_srv(self, value):
        '''
        蜂鸣器控制Buzzer control
        :param value:
         [  0：   关闭,Close
            1：   一直响, Keep on
            >=10：响xx毫秒后自动关闭（value是10的倍数)Turn off automatically after xx milliseconds (value is a multiple of 10) ]
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
        高帧率相机前灯控制 High frame rate camera headlight control
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

    def Patrol_srv(self, Commond, LineScaling, RotationScaling):
        '''
        巡逻 Patrol
        :param Commond: Triangle,Square,Parallelogram,Circle,finish
        :param LineScaling: [0.0,2.0]
        :param RotationScaling: [0.0,2.0]
        '''
        self.Patrol_client.wait_for_service()
        request = PatrolRequest()
        request.Commond = Commond
        request.LineScaling = LineScaling
        request.RotationScaling = RotationScaling
        try:
            print("request   !!!")
            response = self.Patrol_client.call(request)
            if isinstance(response, PatrolResponse): return response.result
        except Exception:
            rospy.loginfo("Patrol error")
        return False

    def RGBLight_srv(self, effect, speed):
        '''
        RGB可编程灯带特效展示。 RGB programmable lights special effects.
        :param effect: [0：停止灯效，1：流水灯，2：跑马灯，3：呼吸灯，4：渐变灯，5：星光点点，6：电量显示]
        :param speed: [1, 10]，数值越小速度变化越快。
        :param effect: [0: stop light effect, 1: running water light, 2: marquee light, 3: breathing light, 4: gradient light, 5: starlight, 6: battery display]
        :param speed: [1, 10], the smaller the value, the faster the speed change.
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

    def CamDevice_srv(self, value):
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

    def PWM_Reset(self):
        self.PWMServo_topic(1, 90)
        rospy.sleep(0.1)
        self.PWMServo_topic(2, 70)  # 115
        rospy.sleep(0.1)
        self.PWMServo_topic(1, 90)

if __name__ == '__main__':
    rospy.init_node("ProgramCtrl",anonymous=False)
    program_ctrl = ProgramCtrl()
    program_ctrl.CamDevice_srv("getCamDevice")
