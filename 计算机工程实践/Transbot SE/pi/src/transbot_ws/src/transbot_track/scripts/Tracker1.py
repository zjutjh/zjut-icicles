#!/usr/bin/env python3
# coding: utf-8
import os
import time
import rospy
import getpass
import threading
from track_common import *
from geometry_msgs.msg import Twist
from transbot_msgs.msg import Position
from sensor_msgs.msg import CompressedImage
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from transbot_track.cfg import TrackColorHSVConfig
from pygame.math import Vector2
from Transbot_Lib import Transbot
from arm_transbot import Transbot_ARM
import math

class M_Tracker:
    def __init__(self):
        rospy.init_node("m_Tracker", anonymous=False)
        rospy.on_shutdown(self.cancel)
        self.index = 2
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT', 'color']
        self.tracker_type = rospy.get_param("~tracker_type", 'KCF')
        self.VideoSwitch = rospy.get_param("~VideoSwitch", True)
        self.user_name = getpass.getuser()
        text_path = '/home/' + self.user_name + '/transbot_ws/src/transbot_track/scripts'
        self.hsv_text = text_path + "/TrackerHSV.text"
        self.pub_position = rospy.Publisher("/Current_point", Position, queue_size=10)
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        Server(TrackColorHSVConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client("/m_Tracker", timeout=60)
        print("OpenCV Version: ",cv.__version__)
        if self.tracker_type == "color":
            self.Track_state = 'identify'
        else:
            self.gTracker = Tracker(tracker_type=self.tracker_type)
            self.tracker_type = self.tracker_types[self.index]
            self.Track_state = 'init'
        if self.VideoSwitch == False:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            img_topic = rospy.get_param("~camera", "/usb_cam/image_raw/compressed")
            self.sub_img = rospy.Subscriber(img_topic, CompressedImage, self.compressed_topic, queue_size=10)
        self.bot = Transbot(debug=False)
        self.bot.create_receive_threading()
        self.bot.set_beep(50)
        time.sleep(.1)
        self.version = self.bot.get_version()
        print("version=", self.version)
        self.traceFlag = 0
        self.link1 = 59
        self.link2 = 149
        self.theta1 = math.radians(90)
        self.theta2 = math.radians(180)
        self.effector = Vector2( self.link1 +  self.link2, 0)
        self.angles = Vector2(0, 0)

    def dynamic_reconfigure_callback(self, config, level):
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        return config

    def compressed_topic(self, msg):
        if not isinstance(msg, CompressedImage): return
        start = time.time()
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        action = cv.waitKey(10) & 0xFF
        rgb_img, binary = self.process(frame, action)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(rgb_img, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        thread_text = "thread : " + str(len(threading.enumerate()))
        cv.putText(rgb_img, thread_text, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))
        else: cv.imshow(self.windows_name, rgb_img)

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def process(self, rgb_img, action):
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == 32: self.Track_state = 'tracking'
        elif action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()
        elif action == ord('q') or action == ord('Q'): self.cancel()
        elif action == ord('f') or action == ord('F'):
            self.index += 1
            if self.index >= len(self.tracker_types): self.index = 0
            self.tracker_type = self.tracker_types[self.index]
            self.gTracker = Tracker(tracker_type=self.tracker_type)
            print("tracker_type: ", self.tracker_type)
            self.Reset()
            self.Track_state = 'init'
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    if self.tracker_type == "color": rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text) and self.tracker_type == "color":
                self.hsv_range = read_HSV(self.hsv_text)
            else: self.Track_state = 'init'
        if self.Track_state != 'init':
            if self.tracker_type == "color" and len(self.hsv_range) != 0:
                rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_client.update_configuration(params)
                    self.dyn_update = False
            if self.tracker_type != "color":
                if self.gTracker_state == True:
                    Roi = (self.Roi_init[0], self.Roi_init[1], self.Roi_init[2] - self.Roi_init[0], self.Roi_init[3] - self.Roi_init[1])
                    self.gTracker = Tracker(tracker_type=self.tracker_type)
                    self.gTracker.initWorking(rgb_img, Roi)
                    self.gTracker_state = False
                rgb_img, (targBegin_x, targBegin_y), (targEnd_x, targEnd_y) = self.gTracker.track(rgb_img)
                center_x = targEnd_x / 2 + targBegin_x / 2
                center_y = targEnd_y / 2 + targBegin_y / 2
                width = targEnd_x - targBegin_x
                high = targEnd_y - targBegin_y
                self.point_pose = (center_x, center_y, min(width, high))
        if self.Track_state == 'tracking':
            self.Start_state = True
            if self.circle[2] != 0: 
                if self.circle[2] <= 20:
                    circle_2 = 20
                elif self.circle[2] >= 60:
                    circle_2 = 60
                else:
                    circle_2 = self.circle[2]
                circle_2 = 234/40 *(circle_2-20)
                circle_1 = 234 - 234/480 *(self.circle[1])
                print("circle================",self.circle[0],circle_1,circle_2)
                angle = self.invKin(Vector2(234-circle_2,circle_1))
                self.bot.set_uart_servo_angle_array(angle[0],angle[1],120)
                #threading.Thread(target=self.execute,args=(self.circle[0],self.circle[1],self.circle[2])).start()
                threading.Thread(target=self.move,args=(self.circle[0],234-circle_2)).start()
            if self.point_pose[0] != 0 and self.point_pose[1] != 0:
                angle1 = self.invKin(Vector2(600,480-self.point_pose[1]))[0]
                self.bot.set_uart_servo_angle_array(angle1+30,135,120)
                threading.Thread(target=self.move_,args=(self.point_pose[0],320)).start()
                #threading.Thread(target=self.execute,args=(self.point_pose[0],self.point_pose[1],self.point_pose[2])).start()
        else:
            if self.Start_state == True:
                self.pub_cmdVel.publish(Twist())
                self.Start_state = False
        if self.tracker_type != "color": cv.putText(rgb_img, self.tracker_type + " Tracker", (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        return rgb_img, binary

    def execute(self, x, y, z):
        position = Position()
        position.angleX = x
        position.angleY = y
        position.distance = z
        self.pub_position.publish(position)

    def cancel(self):
        self.Reset()
        self.dyn_client.close()
        self.pub_position.unregister()
        if self.VideoSwitch == False: self.sub_img.unregister()
        print("Shutting down this node.")
        cv.destroyAllWindows()

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        for i in range(3): self.pub_position.publish(Position())
        rospy.loginfo("init succes!!!")


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
        if th1> 90:
            th1=90
        if th2 > 90:
            th2=90
        th = Vector2(120+th1, 180-th2)
        return th

    def move_(self, x,center_x):
        size = 50
        velocity = 0
        angular = 1
        if 640 > x > center_x + size:
            for i in range(10):
                self.bot.set_car_motion(velocity,-angular)
                time.sleep(0.01)
                self.bot.set_car_motion(velocity,0)
        elif 0 < x < center_x - size:
            for i in range(10):
                self.bot.set_car_motion(velocity,angular)
                time.sleep(0.01)
                self.bot.set_car_motion(velocity,0)
        elif center_x-size<x<center_x+size:
            self.bot.set_car_motion(velocity,0)

    def move(self,x,distanceCM):
        size = 80
        velocity = 0
        angular = 1.5
        distance = 150
        center_x = 320
        if 640 > x > center_x + size:
            for i in range(10):
                self.bot.set_car_motion(velocity,-angular)
                time.sleep(0.01)
                self.bot.set_car_motion(velocity,0)
        elif 0 < x < center_x - size:
            for i in range(10):
                self.bot.set_car_motion(velocity,angular)
                time.sleep(0.01)
                self.bot.set_car_motion(velocity,0)
        elif center_x-size<x<center_x+size:
            self.bot.set_car_motion(velocity,0)
        if distance < distanceCM:
            for i in range(3):
                self.bot.set_car_motion(0.45,0)
                time.sleep(0.1)
                self.bot.set_car_motion(velocity,0)
        elif distanceCM + 40 < distance:
                self.bot.set_car_motion(-0.45,0)
                time.sleep(0.1)
                self.bot.set_car_motion(velocity,0)
        elif distanceCM < distance < distanceCM + 40:
                self.bot.set_car_motion(0,0)

if __name__ == '__main__':

    m_tracker = M_Tracker()
    if m_tracker.VideoSwitch == False: rospy.spin()
    else:
        capture = cv.VideoCapture(0)
        cv_edition = cv.__version__
        if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        print("capture: ",capture.isOpened())
        for i in range(3):
            m_tracker.bot.set_pwm_servo(1,90)
            m_tracker.bot.set_pwm_servo(2,115)
        while capture.isOpened():
            start = time.time()
            ret, frame = capture.read()
            action = cv.waitKey(10) & 0xFF
            frame, binary = m_tracker.process(frame, action)
            end = time.time()
            fps = 1 / (end - start)
            text = "FPS : " + str(int(fps))
            cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
            thread_text = "thread : " + str(len(threading.enumerate()))
            cv.putText(frame, thread_text, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
            if len(binary) != 0:
                cv.imshow('frame', ManyImgs(1, ([frame, binary])))
            else:
                cv.imshow('frame', frame)
            if action == ord('q') or action == ord('Q'):
                m_tracker.cancel()
                break
        capture.release()
        cv.destroyAllWindows()
