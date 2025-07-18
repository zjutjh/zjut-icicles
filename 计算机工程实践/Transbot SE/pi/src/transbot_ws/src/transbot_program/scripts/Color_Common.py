#!/usr/bin/env python
# encoding: utf-8
import cv2 as cv
import numpy as np

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
    rows = len(imgarray)  # 元组或者列表的长度(Length of tuple or list)
    cols = len(imgarray[0])  # 如果imgarray是列表，返回列表里第一幅图像的通道数，如果是元组，返回元组里包含的第一个列表的长度
    #If imgarray is a list, return the number of channels of the first image in the list, if it is a tuple, return the length of the first list contained in the tuple
    # print("rows=", rows, "cols=", cols)
    # 判断imgarray[0]的类型是否是list (Determine whether the type of imgarray[0] is list)
    # 是list，表明imgarray是一个元组，需要垂直显示 (It is a list, indicating that imgarray is a tuple and needs to be displayed vertically)
    rowsAvailable = isinstance(imgarray[0], list)
    # 第一张图片的宽高(The width and height of the first picture)
    width = imgarray[0][0].shape[1]
    height = imgarray[0][0].shape[0]
    # print("width=", width, "height=", height)
    # 如果传入的是一个元组(If the incoming is a tuple)
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                # 遍历元组，如果是第一幅图像，不做变换(Traverse the tuple, if it is the first image, do not transform)
                if imgarray[x][y].shape[:2] == imgarray[0][0].shape[:2]:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (0, 0), None, scale, scale)
                # 将其他矩阵变换为与第一幅图像相同大小，缩放比例为scale (Transform other matrices to the same size as the first image, and the zoom ratio is scale)
                else:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (imgarray[0][0].shape[1], imgarray[0][0].shape[0]), None,
                                               scale, scale)
                # 如果图像是灰度图，将其转换成彩色显示 (If the image is grayscale, convert it to color display)
                if len(imgarray[x][y].shape) == 2:
                    imgarray[x][y] = cv.cvtColor(imgarray[x][y], cv.COLOR_GRAY2BGR)
        # 创建一个空白画布，与第一张图片大小相同 (Create a blank canvas, the same size as the first picture)
        imgBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imgBlank] * rows  # 与第一张图片大小相同，与元组包含列表数相同的水平空白图像 (The same size as the first image, and the same number of horizontal blank images as the tuple contains the list)
        for x in range(0, rows):
            # 将元组里第x个列表水平排列 (Arrange the x-th list in the tuple horizontally)
            hor[x] = np.hstack(imgarray[x])
        ver = np.vstack(hor)  # 将不同列表垂直拼接 (Concatenate different lists vertically)
    # 如果传入的是一个列表 (If the incoming is a list)
    else:
        # 变换操作，与前面相同 (Transformation operation, same as before)
        for x in range(0, rows):
            if imgarray[x].shape[:2] == imgarray[0].shape[:2]:
                imgarray[x] = cv.resize(imgarray[x], (0, 0), None, scale, scale)
            else:
                imgarray[x] = cv.resize(imgarray[x], (imgarray[0].shape[1], imgarray[0].shape[0]), None, scale, scale)
            if len(imgarray[x].shape) == 2:
                imgarray[x] = cv.cvtColor(imgarray[x], cv.COLOR_GRAY2BGR)
        # 将列表水平排列 (Arrange the list horizontally)
        hor = np.hstack(imgarray)
        ver = hor
    return ver

class ColorFollow:
    def __init__(self):
        '''
        初始化一些参数 (Initialize some parameters)
        '''
        self.binary = None
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.hsv_follow = "TransbotFollowHSV.text"
        self.hsv_tracker = "TransbotTrackerHSV.text"

    def get_position(self, rgb_img, hsv_msg, Function):
        height, width = rgb_img.shape[:2]
        img = rgb_img.copy()
        if Function=="Follow_Line": img[0:int(height / 2), 0:width] = 0
        # 检测轮廓点集 Detect contour point set
        (lowerb, upperb) = hsv_msg
        # 复制原始图像,避免处理过程中干扰 Copy the original image to avoid interference during processing
        color_mask = img.copy()
        # 将图像转换为HSV Convert the image to HSV
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素 Filter out the elements located between two arrays
        color = cv.inRange(hsv_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色 Set the non-mask detection part to all black
        color_mask[color == 0] = [0, 0, 0]
        # 将图像转为灰度图 Convert image to grayscale
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素 Get different shapes of structural elements
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作 Morphological closing operation
        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 图像二值化操作 Image binarization operation
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) Get the set of contour points (coordinates)
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
            # 获取盒子顶点 Get box vertices
            box = cv.boxPoints(max_rect)
            # 转成long类型 Convert to long type
            box = np.int0(box)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(max_box)
            # 将检测到的颜色用原形线圈标记出来 Mark the detected color with the original shape coil
            self.Center_x = int(color_x)
            self.Center_y = int(color_y)
            self.Center_r = int(color_radius)
            # Draw the smallest rectangle
            if Function == "Follow_Line": cv.drawContours(rgb_img, [box], 0, (255, 0, 0), 2)
            else: cv.circle(rgb_img, (self.Center_x, self.Center_y), self.Center_r, (255, 0, 255), 2)
            cv.circle(rgb_img, (self.Center_x, self.Center_y), 5, (255, 0, 255), -1)
        else:
            self.Center_x = 0
            self.Center_y = 0
            self.Center_r = 0
        return rgb_img, binary, (self.Center_x, self.Center_y,self.Center_r)

    def Roi_hsv(self, img, Roi):
        '''
        获取某一区域的HSV的范围 Get the range of HSV in a certain area
        :param img: 彩色图 Color map
        :param Roi:  (x_min, y_min, x_max, y_max)
        Roi=(290,280,350,340)
        :return: 图像和HSV的范围 例如：(0,0,90)(177,40,150) The range of the image and HSV For example: (0,0,90)(177,40,150)
        '''
        H = [];S = [];V = []
        # img = cv.resize(img, (640, 480), )
        # 将彩色图转成HSV  Convert color image to HSV
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # 画矩形框 Draw rectangle
        # cv.rectangle(img, (Roi[0], Roi[1]), (Roi[2], Roi[3]), (0, 255, 0), 2)
        # 依次取出每行每列的H,S,V值放入容器中 Take out the H, S, V values of each row and each column in turn and put them into the container
        for i in range(Roi[0], Roi[2]):
            for j in range(Roi[1], Roi[3]):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        # 分别计算出H,S,V的最大最小 Calculate the maximum and minimum of H, S, and V respectively
        H_min = min(H);H_max = max(H)
        S_min = min(S);S_max = max(S)
        V_min = min(V);V_max = max(V)
        # HSV范围调整 HSV range adjustment
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

