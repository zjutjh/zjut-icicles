#!/usr/bin/env python3
# coding: utf-8
import cv2
import numpy as np
import random
import time
from Transbot_Lib import Transbot
import threading
import pyzbar.pyzbar as pyzbar

class Arm_grab:
    def __init__(self):
        self.bot = Transbot()
        self.destination_name = None
        self.image = None
        self.p_hold = [185,94]
        self.g_state_arm = 0
        self.started = 0

        

    # Define the function of moving the manipulator, and control the motion of No. 7-8 servos at the same time, p=[S 1, S 2]
    def arm_move(self, p, s_time = 500):
        for i in range(2):
            id = i + 7
            if id == 8:
                time.sleep(.1)
                self.bot.set_uart_servo_angle(8, 94)
            #elif id == 1 :
                #self.bot.set_uart_servo_angle_array(id, p[i], int(3*s_time/4))
            else:
                self.bot.set_uart_servo_angle(7, 215)
            time.sleep(.01)
        time.sleep(s_time/1000)
    
    # Define the function of clamping blocks, enable=1: clamp, =0: release
    def arm_clamp_block(self, enable):
        if enable == 0:
            self.bot.set_uart_servo_angle(9, 105)
        else:
            self.bot.set_uart_servo_angle(9, 155)
        time.sleep(.5)

    # Control the movement of the robotic arm
    def ctrl_arm_move(self, index):
        self.arm_clamp_block(0)
        if index == 1:
            print("front-left")
            self.bot.set_beep(1000)
            time.sleep(1)
            self.bot.set_beep(0)
            self.number_action(index)
            self.bot.set_car_motion(0,0.2)
            time.sleep(3)
            self.bot.set_car_motion(0,0)
            self.put_down_block()
            self.bot.set_car_motion(0,-0.2)
            time.sleep(3)
            self.bot.set_car_motion(0,0)
        elif index == 2:
            print("middle")
            self.bot.set_beep(1000)
            time.sleep(1)
            self.bot.set_beep(0)
            self.number_action(index)
            #self.bot.set_car_motion(0.1,0.1)
            time.sleep(3)
            #self.bot.set_car_motion(0,0)
            self.put_down_block()
        elif index == 3:
            print("front-right")
            self.bot.set_beep(1000)
            time.sleep(1)
            self.bot.set_beep(0)
            self.number_action(index)
            self.bot.set_car_motion(0,-0.2)
            time.sleep(3)
            self.bot.set_car_motion(0,0)
            self.put_down_block()
            self.bot.set_car_motion(0,0.2)
            time.sleep(3)
            self.bot.set_car_motion(0,0)
        self.g_state_arm = 0

    #Digital function definition
    def number_action(self, index):
        if index == 1:
            # Grab the block of position on front-left.
            self.arm_move(self.p_hold, 1000)
            time.sleep(3)
            self.arm_clamp_block(1)
            # time.sleep(.5)
        elif index == 2:
            # Grab the block of position on middle.
            self.arm_move(self.p_hold, 1000)
            self.arm_clamp_block(1)
        elif index == 3:
            # Grab the block of position on front-right.
            self.arm_move(self.p_hold, 1000)
            self.arm_clamp_block(1)


    # Grab the blocks
    def put_down_block(self):
        self.bot.set_uart_servo_angle_array(210, 30, 155)
        time.sleep(3)
        self.arm_clamp_block(0) 
        self.bot.set_uart_servo_angle_array(180, 80, 120)
        time.sleep(1)

    def start_move_arm(self, index):
        # Open the robot arm control thread
        if self.g_state_arm == 0:
            closeTid = threading.Thread(target = self.ctrl_arm_move, args = [index])
            closeTid.setDaemon(True)
            closeTid.start()
            
            self.g_state_arm = 1
 


    def decodeDisplay(self,image):
        destination_name={}
        barcodes = pyzbar.decode(image)
        for barcode in barcodes:
        # 提取二维码的边界框的位置
        # 画出图像中条形码的边界框
          (x, y, w, h) = barcode.rect
          cv2.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 2)
        # 提取二维码数据为字节对象，所以如果我们想在输出图像上画出来，就需要先將它转换成字符串
          barcodeData = barcode.data.decode("utf-8")
          barcodeType = barcode.type
        # 绘出图像上条形码的数据和条形码类型
          text = "{} ({})".format(barcodeData, barcodeType)
          cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 0, 0), 2)
        # 向终端打印条形码数据和条形码类型
          print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
          destination_name['name'] = barcodeData
        #detect_control(barcodeData)
        return image,destination_name


    def reset_state(self):
        self.started = 0


    def Position_Recongnize(self, frame):
        if self.started == 0:
            #self.bot.set_uart_servo_angle_array(180, 80, 130, 1000)
            time.sleep(1)
   
        
        frame, destination_name = self.decodeDisplay(frame)
        if len(destination_name)==1:
            # print ("name :", destination_name['name'])
            if destination_name['name'] == 'front-left':
                self.Arm.set_beep(1)
                #self.start_move_arm(1)
            elif destination_name['name'] == 'middle':
#                 self.Arm.set_beep(1)
                self.start_move_arm(2)
            elif  destination_name['name'] == 'front-right':
#                 self.Arm.set_beep(1)
                self.start_move_arm(3)
            self.started = 1
            #self.bot.set_beep(1)  
  
        return frame
            

    def start_grab(self, img):
        '''
        Color follow control function

        :param img:       input image
        :return: img      output processed image
        '''
        self.image = self.Position_Recongnize(img)

        return self.image

if __name__ == '__main__':
    grab = Arm_grab()
    capture = cv2.VideoCapture(0)
    while capture.isOpened():
        _, img = capture.read()
        img = grab.start_grab(img)
        cv2.imshow("img", img)
        action = cv2.waitKey(10) & 0xff
        if action == ord('q') or action == 27:
            print("over")
            cv2.destroyAllWindows()
            capture.release()
            break
    cv2.destroyAllWindows()
    capture.release()

