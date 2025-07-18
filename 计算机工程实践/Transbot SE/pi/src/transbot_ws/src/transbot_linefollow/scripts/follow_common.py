#!/usr/bin/env python
# encoding: utf-8
import time
import rospy
import cv2 as cv
import numpy as np
from transbot_msgs.srv import *
from transbot_msgs.msg import *
from geometry_msgs.msg import Twist

def write_HSV(wf_path, value):
    with open(wf_path, "w") as wf:
        wf_str = str(value[0][0]) + ', ' + str(
            value[0][1]) + ', ' + str(value[0][2]) + ', ' + str(
            value[1][0]) + ', ' + str(value[1][1]) + ', ' + str(
            value[1][2])
        wf.write(wf_str)
        wf.flush()

def read_HSV(rf_path):
    rf = open(rf_path, "r+")
    line = rf.readline()
    if len(line) == 0: return ()
    list = line.split(',')
    if len(list) != 6: return ()
    hsv = ((int(list[0]), int(list[1]), int(list[2])),
           (int(list[3]), int(list[4]), int(list[5])))
    rf.flush()
    return hsv

# 定义函数，第一个参数是缩放比例，第二个参数是需要显示的图片组成的元组或者列表
# Define the function, the first parameter is the zoom ratio, and the second parameter is a tuple or list of pictures to be displayed
def ManyImgs(scale, imgarray):
    rows = len(imgarray)         # 元组或者列表的长度
				 # Length of tuple or list
    cols = len(imgarray[0])      # 如果imgarray是列表，返回列表里第一幅图像的通道数，如果是元组，返回元组里包含的第一个列表的长度
				# If imgarray is a list, return the number of channels of the first image in the list, if it is a tuple, return the length of the first list contained in the tuple

    # print("rows=", rows, "cols=", cols)
    # 判断imgarray[0]的类型是否是list
    # 是list，表明imgarray是一个元组，需要垂直显示
    # Determine whether the type of imgarray[0] is list
    # It is a list, indicating that imgarray is a tuple and needs to be displayed verticallyIt is a list, indicating that imgarray is a tuple and needs to be displayed vertically
    rowsAvailable = isinstance(imgarray[0], list)
    # 第一张图片的宽高
    # The width and height of the first picture
    width = imgarray[0][0].shape[1]
    height = imgarray[0][0].shape[0]
    # print("width=", width, "height=", height)
    # 如果传入的是一个元组
    # If the incoming is a tuple
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                # 遍历元组，如果是第一幅图像，不做变换
		# Traverse the tuple, if it is the first image, do not transform
                if imgarray[x][y].shape[:2] == imgarray[0][0].shape[:2]:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (0, 0), None, scale, scale)
                # 将其他矩阵变换为与第一幅图像相同大小，缩放比例为scale
		# Transform other matrices to the same size as the first image, and the zoom ratio is scale
                else:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (imgarray[0][0].shape[1], imgarray[0][0].shape[0]), None, scale, scale)
                # 如果图像是灰度图，将其转换成彩色显示
		# If the image is grayscale, convert it to color display
                if  len(imgarray[x][y].shape) == 2:
                    imgarray[x][y] = cv.cvtColor(imgarray[x][y], cv.COLOR_GRAY2BGR)
        # 创建一个空白画布，与第一张图片大小相同
	# Create a blank canvas, the same size as the first picture
        imgBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imgBlank] * rows   # 与第一张图片大小相同，与元组包含列表数相同的水平空白图像
				  # The same size as the first picture, and the same number of horizontal blank images as the tuple contains the list
        for x in range(0, rows):
            # 将元组里第x个列表水平排列
	    # Arrange the x-th list in the tuple horizontally
            hor[x] = np.hstack(imgarray[x])
        ver = np.vstack(hor)   # 将不同列表垂直拼接
			       # Concatenate different lists vertically
    # 如果传入的是一个列表
    # If the incoming is a list
    else:
        # 变换操作，与前面相同
	# Transformation operation, same as before
        for x in range(0, rows):
            if imgarray[x].shape[:2] == imgarray[0].shape[:2]:
                imgarray[x] = cv.resize(imgarray[x], (0, 0), None, scale, scale)
            else:
                imgarray[x] = cv.resize(imgarray[x], (imgarray[0].shape[1], imgarray[0].shape[0]), None, scale, scale)
            if len(imgarray[x].shape) == 2:
                imgarray[x] = cv.cvtColor(imgarray[x], cv.COLOR_GRAY2BGR)
        # 将列表水平排列
	# Arrange the list horizontally
        hor = np.hstack(imgarray)
        ver = hor
    return ver

class ROSCtrl:
    def __init__(self):
        self.Joy_active = False
        self.PWMServo = PWMServo()
        self.CamDevice_client = rospy.ServiceProxy("/CamDevice", CamDevice)
        self.pub_PWMServo = rospy.Publisher("/PWMServo", PWMServo, queue_size=50)
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_JoyState = rospy.Subscriber('/JoyState', JoyState, self.JoyStateCallback)
        self.Buzzer_client = rospy.ServiceProxy("/Buzzer", Buzzer)
        self.pub_Arm = rospy.Publisher("/TargetAngle", Arm, queue_size=10)

    def JoyStateCallback(self, msg):
        if not isinstance(msg, JoyState): return
        self.Joy_active = msg.state
        self.pub_cmdVel.publish(Twist())

    def CamDevice_srv(self, value):
        self.CamDevice_client.wait_for_service()
        request = CamDeviceRequest()
        request.GetGev = str(value)
        try:
            response = self.CamDevice_client.call(request)
            if isinstance(response, CamDeviceResponse):
                self.CameraDevice = response.camDevice
                rospy.loginfo(response)
        except Exception:
            rospy.loginfo("CamDevice error")
        return self.CameraDevice

    def Buzzer_srv(self, value):
        '''
        Buzzer control 蜂鸣器控制
        :param value:
         [  0：   close 关闭,
            1：   Keep ringing一直响,
            >=10：Turn off automatically after xx milliseconds (value is a multiple of 10) 响xx毫秒后自动关闭（value是10的倍数) ]
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

    def Arm_Reset(self):
        t=0.01
        self.arm_ctrl(t * 1000,225,30,30)
        
    
    def PWM_Reset(self):
        self.PWMServo_topic(1, 89)
        rospy.sleep(0.1)
        self.PWMServo_topic(2, 70)  # 90
        rospy.sleep(0.1)
        self.PWMServo_topic(1,89)
 
    def PWMServo_topic(self, id, angle):
        '''
        PWMServo云台舵机接口
        :param id: [2] : servo up/down云台上下 , [1] :servo left/right云台左右]
        :param angle: [0, 180]
        '''
        self.PWMServo.id = id
        self.PWMServo.angle = angle
        self.pub_PWMServo.publish(self.PWMServo)
        # rospy.loginfo("pub PWMServo succes!!!")

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
        
    
    def cancel(self):
        self.sub_JoyState.unregister()
        self.pub_cmdVel.unregister()
        self.pub_PWMServo.unregister()
        self.Buzzer_client.close()
        self.pub_Arm.unregister()
        
class color_follow:
    def __init__(self):
        '''
        初始化一些参数
	Initialize some parameters
        '''
        self.binary = None
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0

    def line_follow(self, rgb_img, hsv_msg):
        height, width = rgb_img.shape[:2]
        img = rgb_img.copy()
        #img[0:int(height/2), 0:width] = 0
        # 将图像转换为HSV。
	# Convert the image to HSV.
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        # 根据特定颜色范围创建mask
	# Create a mask based on a specific color range
        mask = cv.inRange(hsv_img, lower, upper)
        color_mask = cv.bitwise_and(hsv_img, hsv_img, mask=mask)
        # 将图像转为灰度图
	# Convert the image to grayscale
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素
	# Get structure elements of different shapes
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
	# Morphological closed operation
        gray_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 图像二值化操作
	# Image binarization operation
        ret, binary = cv.threshold(gray_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标)
	# Get the set of contour points (coordinates)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3: contours = find_contours[1]
        else: contours = find_contours[0]
        if len(contours) != 0:
            areas = []
            for c in range(len(contours)): areas.append(cv.contourArea(contours[c]))
            max_id = areas.index(max(areas))
            max_rect = cv.minAreaRect(contours[max_id])
            max_box = cv.boxPoints(max_rect)
            max_box = np.int0(max_box)
            # 获取盒⼦顶点
	    # Get box vertices
            box = cv.boxPoints(max_rect)
            # 转成long类型
	    # Convert to long type
            box = np.int0(box)
            # 绘制最小矩形
	    # Draw the smallest rectangle
            cv.drawContours(rgb_img, [box], 0, (255, 0, 0), 2)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(max_box)
            # 将检测到的颜色用原形线圈标记出来
	    # Mark the detected color with the original shape coil
            self.Center_x = int(color_x)
            self.Center_y = int(color_y)
            self.Center_r = int(color_radius)
            cv.circle(rgb_img, (self.Center_x, self.Center_y), 5, (255, 0, 255), -1)
        else:
            self.Center_x = 0
            self.Center_y = 0
            self.Center_r = 0
        return rgb_img, binary, (self.Center_x, self.Center_y,self.Center_r)

    def Roi_hsv(self, img, Roi):
        '''
        Get the range of HSV in a certain area获取某一区域的HSV的范围
        :param img: 彩色图
        :param Roi:  (x_min, y_min, x_max, y_max)
        Roi=(290,280,350,340)
        :return: The range of images and HSV such as:(0,0,90)(177,40,150) 图像和HSV的范围 例如：(0,0,90)(177,40,150)
        '''
        H = [];S = [];V = []
        # img = cv.resize(img, (640, 480), )
        # 将彩色图转成HSV
	# Convert color image to HSV
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # 画矩形框
	# Draw a rectangular frame
        # cv.rectangle(img, (Roi[0], Roi[1]), (Roi[2], Roi[3]), (0, 255, 0), 2)
        # 依次取出每行每列的H,S,V值放入容器中
	# Take out the H, S, V values of each row and each column in turn and put them into the container
        for i in range(Roi[0], Roi[2]):
            for j in range(Roi[1], Roi[3]):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        # 分别计算出H,S,V的最大最小
	# Calculate the maximum and minimum of H, S, and V respectively
        H_min = min(H);H_max = max(H)
        S_min = min(S);S_max = max(S)
        V_min = min(V);V_max = max(V)
        # HSV范围调整
	# HSV range adjustment
        if H_max + 5 > 255:H_max = 255
        else:H_max += 5
        if H_min - 5 < 0:H_min = 0
        else:H_min -= 5
        if S_min - 20 < 0:S_min = 0
        else:S_min -= 20
        if V_min - 20 < 0:V_min = 0
        else:V_min -= 20
        S_max = 253;V_max = 255
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        if S_min < 5 or V_min < 5:
            cv.putText(img, txt1, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            cv.putText(img, txt2, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv.putText(img, lowerb, (150, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv.putText(img, upperb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range

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

