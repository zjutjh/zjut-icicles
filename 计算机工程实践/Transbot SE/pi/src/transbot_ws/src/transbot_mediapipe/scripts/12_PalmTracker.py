#!/usr/bin/env python3.7
# coding: utf-8
import cv2
from cvzone.HandTrackingModule import HandDetector
import math
import numpy as np
import cvzone
import rospy
from Transbot_Lib import Transbot
from arm_transbot import Transbot_ARM
import struct
import time
import serial
import threading
import math
from pygame.math import Vector2

class M_Tracker:
	def __init__(self):
		self.bot = Transbot(debug=False)
		self.bot.create_receive_threading()
		self.bot.set_beep(50)
		time.sleep(.1)
		self.version = self.bot.get_version()
		print("version=", self.version)
		#self.bot.set_uart_servo_angle_array(220, 90, 180)
		self.traceFlag = 0
		self.link1 = 59
		self.link2 = 149
		self.theta1 = math.radians(90)
		self.theta2 = math.radians(180)
		self.effector = Vector2( self.link1 +  self.link2, 0)
		self.angles = Vector2(0, 0)
		self.limit = 80
		self.x = [300, 245, 200, 170, 145, 130, 112, 103, 93, 87, 80, 75, 70, 67, 62, 59, 57]
		self.y = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]
		self.coff = np.polyfit(self.x, self.y, 2)
		self.size = 80
		self.velocity = 0
		self.angular = 1.5
		self.distance = 100
		self.key_map = {114: lambda: M_Tracker.bot.set_uart_servo_angle_array(220, 90, 180),113: lambda: M_Tracker.bot.set_car_motion(0, 0)}


	def invKin(self,e):
		r_2 = e.length_squared()
		l_sq = self.link1 ** 2 + self.link2 ** 2
		term2 = (r_2 - l_sq) / (2 * self.link1 * self.link2)
		if term2 > 1:
			term2 = 1
		elif term2 < -1:
			term2 = -1
		term1 = -1 * math.sqrt(1 - (term2 ** 2))
		th2 = math.atan2(term1, term2)
		k1 = self.link1 + self.link2 * math.cos(th2)
		k2 = self.link2 * math.sin(th2)
		r = math.sqrt(k1 ** 2 + k2 ** 2)
		gamma = math.atan2(k2, k1)
		th1 = math.atan2(e.y, e.x) - gamma
		th1 = abs(math.degrees(th1))
		th2 = -1 * math.degrees(th2)
		if th1> self.limit:
			th1=self.limit
		if th2 > self.limit:
			th2=self.limit
		th = Vector2(120+th1, self.limit*2-th2)
		return th


	def move(self,x,center_x ,distanceCM):
		if 640 > x > center_x + self.size:
			for i in range(10):
				self.bot.set_car_motion(self.velocity,-self.angular)
				time.sleep(0.01)
				self.bot.set_car_motion(self.velocity,0)
		elif 0 < x < center_x - self.size:
			for i in range(10):
				self.bot.set_car_motion(self.velocity,self.angular)
				time.sleep(0.01)
				self.bot.set_car_motion(self.velocity,0)
		elif center_x-self.size<x<center_x+self.size:
			self.bot.set_car_motion(self.velocity,0)
		if self.distance < distanceCM:
			for i in range(3):
				self.bot.set_car_motion(0.45,0)
				time.sleep(0.1)
				self.bot.set_car_motion(self.velocity,0)
		elif distanceCM + 20 < self.distance:
			for i in range(3):
				self.bot.set_car_motion(-0.45,0)
				time.sleep(0.1)
				self.bot.set_car_motion(self.velocity,0)
		elif distanceCM < self.distance < distanceCM + 20:
			self.bot.set_car_motion(0,0)


if __name__ == '__main__':
	M_Tracker=M_Tracker()
	cap = cv2.VideoCapture(0)
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
	M_Tracker.bot.set_uart_servo_angle_array(220,90,180)
	detector = HandDetector(detectionCon=0.8, maxHands=1)
	while True:
		ret, frame = cap.read()
		timer = cv2.getTickCount()
		hands = detector.findHands(frame, draw=True)
		distanceCM = 0
		center_x = 0
		center_y = 0
		if len(hands[0]) == 1:
			lmList = hands[0][0]['lmList']
			x, y, w, h=hands[0][0]['bbox']
			x1, y1, z1 = lmList[5]
			x2, y2, z2 = lmList[17]
			distance = int(math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2))
			A, B, C = M_Tracker.coff
			distanceCM = A * distance ** 2 + B * distance + C
			center_x,center_y = hands[0][0]['center']
		if center_x == 0.0:
			M_Tracker.bot.set_car_motion(0,0)
		else:
			threading.Thread(target=M_Tracker.move,args=(center_x,320,distanceCM)).start()
			if distanceCM <= 40:
				distanceCM = 40
			elif distanceCM >= 90:
				distanceCM = 90
			distanceCM = 234/50 *(distanceCM-40)
			center_y = 234 - 234/480 *(center_y)
			print("center_x:",center_x,"center_y:",center_y,"distanceCM:",distanceCM)
			angle = M_Tracker.invKin(Vector2(distanceCM,center_y))
			M_Tracker.bot.set_uart_servo_angle_array(angle[0],angle[1],angle[1])
		fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
		cv2.putText(frame, "FPS : " + str(int(fps)), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
		cv2.putText(frame, "distance : " + str(int(distanceCM)) + "cm", (50,70), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
		cv2.putText(frame, "center_x : " + str(int(center_x)) + "cm", (50,90), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
		cv2.putText(frame, "center_y : " + str(int(center_y)) + "cm", (50,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
		if ret:
			cv2.imshow('Camera', frame)
		key = cv2.waitKey(10)
		if key in M_Tracker.key_map:
			M_Tracker.key_map[key]()
			if key == 113:
				break
cap.release()
cv2.destroyAllWindows()
