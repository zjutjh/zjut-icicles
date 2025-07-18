#!/usr/bin/env python
# encoding: utf-8
import time

import cv2 as cv
import numpy as np
import rospy


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
    # 判断imgarray[0]的类型是否是list,
    # 是list，表明imgarray是一个元组，需要垂直显示
    # Determine whether the type of imgarray[0] is list,
    # It is a list, indicating that imgarray is a tuple and needs to be displayed vertically
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

class color_follow:
    def __init__(self):
        '''
        初始化一些参数
	Initialize some parameters
        '''
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0

    def object_follow(self, img,  hsv_msg):
        src = img.copy()
        # 由颜色范围创建NumPy数组
	# Create NumPy array from color range
        src = cv.cvtColor(src, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        # 根据特定颜色范围创建mask
	# Create a mask based on a specific color range
        mask = cv.inRange(src, lower, upper)
        color_mask = cv.bitwise_and(src, src, mask=mask)
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
            (color_x, color_y), color_radius = cv.minEnclosingCircle(max_box)
            # 将检测到的颜色用原形线圈标记出来
	    # Mark the detected color with the original shape coil
            self.Center_x = int(color_x)
            self.Center_y = int(color_y)
            self.Center_r = int(color_radius)
            cv.circle(img, (self.Center_x, self.Center_y), self.Center_r, (255, 0, 255), 2)
            cv.circle(img, (self.Center_x, self.Center_y), 2, (0, 0, 255), -1)
        else:
            self.Center_x = 0
            self.Center_y = 0
            self.Center_r = 0
        return img,binary, (self.Center_x, self.Center_y, self.Center_r)

    def Roi_hsv(self, img, Roi):
        '''
        获取某一区域的HSV的范围
        Get the range of HSV in a certain area
        :param img: Color map 彩色图 
        :param Roi:  (x_min, y_min, x_max, y_max)
        Roi=(290,280,350,340)
        :return: 图像和HSV的范围 例如：(0,0,90)(177,40,150)
	         Image and HSV range E.g：(0,0,90)(177,40,150) 
        '''
        H = [];S = [];V = []
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
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.targetpoint = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0

    def compute(self, target, current):
        error = target - current
        self.intergral += error
        self.derivative = error - self.prevError
        self.targetpoint = self.kp * error + self.ki * self.intergral + self.kd * self.derivative
        self.prevError = error
        return self.targetpoint

    def reset(self):
        self.targetpoint = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0


class MessageItem(object):
    # 用于封装信息的类,包含图片和其他信息
    # Class used to encapsulate information, including pictures and other information
    def __init__(self, frame, message):
        self._frame = frame
        self._message = message

    def getFrame(self):
        # 图片信息
	# image information
        return self._frame

    def getMessage(self):
        # 文字信息,json格式
	# Text information, json format
        return self._message


class Tracker(object):
    '''
    追踪者模块,用于追踪指定目标
    Tracker module, used to track the specified target
    '''
    def __init__(self, tracker_type="BOOSTING"):
        '''
        初始化追踪器种类
	Initialize the type of tracker
        '''
        # 获得opencv版本
	# Get the opencv version
        (major_ver, minor_ver, subminor_ver) = (cv.__version__).split('.')
        self.tracker_type = tracker_type
        self.isWorking = False
        # 构造追踪器
	# Construct tracker
        if int(major_ver) < 3: self.tracker = cv.Tracker_create(tracker_type)
        else:
            if tracker_type == 'BOOSTING': self.tracker = cv.TrackerBoosting_create()
            if tracker_type == 'MIL': self.tracker = cv.TrackerMIL_create()
            if tracker_type == 'KCF': self.tracker = cv.TrackerKCF_create()
            if tracker_type == 'TLD': self.tracker = cv.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW': self.tracker = cv.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN': self.tracker = cv.TrackerGOTURN_create()
            if tracker_type == 'MOSSE': self.tracker = cv.TrackerMOSSE_create()
            if tracker_type == "CSRT": self.tracker = cv.TrackerCSRT_create()

    def initWorking(self, frame, box):
        '''
        Tracker work initialization 追踪器工作初始化
        frame:初始化追踪画面
        box:追踪的区域
	Tracker work initialization
	frame:Initialize the tracking screen
	box:Tracked area
        '''
        if not self.tracker: raise Exception("追踪器未初始化Tracker is not initialized")
        status = self.tracker.init(frame, box)
        if not status: raise Exception("追踪器工作初始化失败Failed to initialize tracker job")
        self.coord = box
        self.isWorking = True

    def track(self, frame):
        if self.isWorking:
            status, self.coord = self.tracker.update(frame)
            if status:
                p1 = (int(self.coord[0]), int(self.coord[1]))
                p2 = (int(self.coord[0] + self.coord[2]), int(self.coord[1] + self.coord[3]))
                cv.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
                return frame, p1, p2
            else:
                # 跟踪失败
		# Tracking failed
                cv.putText(frame, "Tracking failure detected", (100, 80), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                return frame, (0, 0), (0, 0)
        else: return frame, (0, 0), (0, 0)

