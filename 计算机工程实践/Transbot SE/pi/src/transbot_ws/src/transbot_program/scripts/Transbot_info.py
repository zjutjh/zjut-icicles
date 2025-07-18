#!/usr/bin/env python2
# coding:utf-8
import time
from Program_Ctrl import *
from transbot_msgs.srv import *
from transbot_msgs.msg import *
import rospy


class TransbotInfo():
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.voltage = 0.0
        self.edition = 0.0
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.sub_edition = rospy.Subscriber('/edition', Edition, self.editionCallback)
        self.sub_voltage = rospy.Subscriber("/voltage", Battery, self.voltageCallback)
        self.RobotArm_client = rospy.ServiceProxy("/CurrentAngle", RobotArm)
        

    def editionCallback(self, msg):
        if not isinstance(msg, Edition): return
        # print("msg:", msg)
        self.edition = msg.edition

    def voltageCallback(self, msg):
        if not isinstance(msg, Battery): return
        # print("msg:", msg)
        self.voltage = msg.Voltage

    def RobotArmSrv(self, value):
        '''
        获取机械臂当前角度  Get the current angle of the robotic arm
        '''
        self.RobotArm_client.wait_for_service()
        request = RobotArmRequest()
        request.apply = value
        joints = []
        try:
            response = self.RobotArm_client.call(request)
            # print("---response--:", response)
            if isinstance(response, RobotArmResponse):
                # rospy.loginfo("arm_joint: ", response.RobotArm.joint)
                # print("response.RobotArm:", response.RobotArm)
                # print("response.RobotArm.joint:", response.RobotArm.joint)
                for joint in response.RobotArm.joint: 
                    # print("joint---:", joint)
                    joints.append(int(joint.angle))
        except Exception:
            rospy.loginfo("robotArm error")
        return joints


    def cancel(self):
        rospy.loginfo("Stopping the robot...")
        self.sub_edition.unregister()
        self.sub_voltage.unregister()
        self.RobotArm_client.close()
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('TransbotInfo', anonymous=False)
    transbot_info = TransbotInfo()
    time.sleep(.1)

    print("voltage: ", transbot_info.voltage)
    print("edition: ", transbot_info.edition)
    print("joints: ", transbot_info.RobotArmSrv(""))
    rospy.spin()

