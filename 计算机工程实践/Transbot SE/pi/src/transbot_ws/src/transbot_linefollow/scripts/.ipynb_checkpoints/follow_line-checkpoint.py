#!/usr/bin/env python
# encoding: utf-8
import os
import threading
import platform
import getpass
from follow_common import *
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from sensor_msgs.msg import CompressedImage
from transbot_linefollow.cfg import LineDetectPIDConfig
# RED: 0, 85, 126, 9, 253, 255

class LineDetect:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("LineDetect", anonymous=False)
        self.img = None
        self.circle = ()
        self.hsv_range = ()
        self.Roi_init = ()
        self.now_time = time.time()
        self.prev_time = 0
        self.warning = 1
        self.index = 1
        self.scale = 1000
        self.PWM_init = False
        self.Start_state = True
        self.ResponseDist = 0.55
        self.dyn_update = True
        self.Buzzer_state = False
        self.select_flags = False
        self.Track_state = 'identify'
        self.windows_name = 'frame'
        self.ros_ctrl = ROSCtrl()
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.user_name = getpass.getuser()
        text_path = '/root/transbot_ws/src/transbot_linefollow/scripts'
        self.hsv_text = text_path+"/LineFollowHSV.text"
        self.python_version = int(platform.python_version()[0])
        self.VideoSwitch = rospy.get_param("~VideoSwitch", False)
        self.img_flip = rospy.get_param("~img_flip", False)
        self.CameraDevice = self.ros_ctrl.CamDevice_srv("GetDevice!!!") 
        Server(LineDetectPIDConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client("LineDetect", timeout=60)
        self.FollowLinePID = (40, 0, 12)
        self.linear = 0.3
        self.PID_init()
        img_topic = "/usb_cam/image_raw/compressed"
        if self.VideoSwitch == False:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.sub_img = rospy.Subscriber(img_topic, CompressedImage, self.compressed_callback)

    def cancel(self):
        self.Reset()
        self.ros_ctrl.cancel()
        print ("Shutting down this node.")
        if self.VideoSwitch==False:
            self.sub_img.unregister()
            cv.destroyAllWindows()

    def compressed_callback(self, msg):
        if self.VideoSwitch == True: return
        if not isinstance(msg, CompressedImage): return
        start = time.time()
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        action = cv.waitKey(10) & 0xFF
        rgb_img, binary = self.process(frame,action)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(rgb_img, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))
        else:cv.imshow(self.windows_name, rgb_img)

    def process(self, rgb_img, action):
        binary = []
        
        rgb_img = cv.resize(rgb_img, (640, 480))
        if self.PWM_init == False: 
            self.ros_ctrl.PWM_Reset()
            self.ros_ctrl.Arm_Reset()
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
        if action == 32: self.Track_state = 'tracking'
        elif action == ord('i') or action == 105: self.Track_state = "identify"
        elif action == ord('r') or action == 114: self.Reset()
        elif action == ord('q') or action == 113: self.cancel()
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0]!=self.Roi_init[2] and self.Roi_init[1]!=self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.dyn_update = True
                else: self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
            else: self.Track_state = 'init'
        if self.Track_state != 'init' and len(self.hsv_range) != 0:
            rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
            if self.dyn_update == True :
                write_HSV(self.hsv_text, self.hsv_range)
                if self.VideoSwitch == False:
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_client.update_configuration(params)
                self.dyn_update = False
        if self.Track_state == 'tracking':
            if len(self.circle) != 0:
                threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
        else:
            if self.Start_state == True:
                self.ros_ctrl.pub_cmdVel.publish(Twist())
                self.Start_state = False
        self.PWM_init = True
        return rgb_img, binary

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x,y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def execute(self, point_x, color_radius):
        self.now_time = time.time()
        if self.ros_ctrl.Joy_active == True:
            if self.Start_state == True:
                self.PID_init()
                self.Start_state = False
            return
        self.Start_state = True
        self.twist = Twist()
        if color_radius == 0: self.ros_ctrl.pub_cmdVel.publish(Twist())
        else:
            self.twist = Twist()
            [z_Pid, _] = self.PID_controller.update([(point_x - 320)/16, 0])
            if self.img_flip == True: self.twist.angular.z = -z_Pid
            else: self.twist.angular.z = +z_Pid
            self.twist.linear.x = self.linear
            if self.warning > 10:
                rospy.loginfo("Obstacles ahead !!!")
                self.ros_ctrl.pub_cmdVel.publish(Twist())
                self.ros_ctrl.Buzzer_srv(1)
                self.Buzzer_state = True
            else:
                if self.Buzzer_state == True:
                    for i in range(3): self.ros_ctrl.Buzzer_srv(0)
                    self.Buzzer_state = False
                self.ros_ctrl.pub_cmdVel.publish(self.twist)
     

    def Reset(self):
        self.PID_init()
        self.PWM_init = False
        self.Track_state = 'init'
        self.hsv_range = ()
        self.ros_ctrl.Joy_active =False
        self.Mouse_XY = (0, 0)
        self.ros_ctrl.Buzzer_srv(0)
        self.ros_ctrl.pub_cmdVel.publish(Twist())
        rospy.loginfo("Reset succes!!!")

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])

    def dynamic_reconfigure_callback(self, config, level):
        print ("dynamic_reconfigure_callback!!!")
        self.scale = config['scale']
        self.linear = config['linear']        
        self.FollowLinePID = (config['Kp'], config['Ki'], config['Kd'])
        self.hsv_range=((config['Hmin'],config['Smin'],config['Vmin']),
                        (config['Hmax'],config['Smax'],config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        print ("HSV: ",self.hsv_range)
        self.PID_init()
        return config

if __name__ == '__main__':
    line_detect = LineDetect()
    if line_detect.VideoSwitch==False:rospy.spin()
    else:
        capture = cv.VideoCapture(0)
        cv_edition = cv.__version__
        if cv_edition[0]=='3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        while capture.isOpened():
            start = time.time()
            ret, frame = capture.read()
            action = cv.waitKey(10) & 0xFF
            frame, binary = line_detect.process(frame, action)
            end = time.time()
            fps = 1 / (end - start)
            text = "FPS : " + str(int(fps))
            cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
            if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
            else:cv.imshow('frame', frame)
            if action == ord('q') or action == 113:
                line_detect.cancel()
                break
        capture.release()
        cv.destroyAllWindows()
