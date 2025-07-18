#!/usr/bin/env python3
# encoding: utf-8
import getpass
import threading
from mono_common import *
from sensor_msgs.msg import CompressedImage
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from transbot_mono.cfg import monoTrackerPIDConfig
from geometry_msgs.msg import Twist
from transbot_msgs.msg import Position

class mono_Tracker:
    def __init__(self):
        rospy.init_node("mono_Tracker", anonymous=False)
        rospy.on_shutdown(self.cancel)
        self.target_servox = 89
        self.target_servoy = 90
        self.point_pose = (0, 0, 0)
        self.joint1 = 145
        self.joint2 = 155
        self.joint3 = 30
        self.circle = (0, 0, 0)
        self.hsv_range = ()
        self.dyn_update = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.index = 2
        self.color = color_follow()
        self.prev_dist = 0
        self.prev_angular = 0
        self.ros_ctrl = ROSCtrl()
        self.tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT' ,"color"]
        self.img_flip = rospy.get_param("~img_flip", False)
        self.tracker_type = rospy.get_param("~tracker_type", 'KCF')
        self.VideoSwitch = rospy.get_param("~VideoSwitch", True)
        self.user_name = getpass.getuser()
        text_path = '/home/'+self.user_name+'/transbot_ws/src/transbot_mono/scripts'
        self.hsv_text = text_path+"/MonoTrackerHSV.text"
        Server(monoTrackerPIDConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client("/mono_Tracker", timeout=60)
        #self.mono_PID = (20, 0, 2)
        #self.scale = 1000
        #self.PID_init()
        self.linear_PID = (3.0, 0.0, 1.0)
        self.angular_PID = (0.3, 0.0, 1.0)
        self.scale = 1000
        self.PID_init()
        t=0.01
        self.ros_ctrl.arm_ctrl(t * 1000, self.joint1, self.joint2, self.joint3)
        print("OpenCV Version: ",cv.__version__)
        if self.tracker_type == "color": self.Track_state = 'identify'
        else:
            self.gTracker = Tracker(tracker_type=self.tracker_type)
            self.tracker_type = self.tracker_types[self.index]
            self.Track_state = 'init'
        if self.VideoSwitch == False:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.img_topic = rospy.get_param("~camera", "/usb_cam/image_raw/compressed")
            self.__sub_img = rospy.Subscriber(self.img_topic, CompressedImage, self.compressed_callback)

    def cancel(self):
        self.Reset()
        self.ros_ctrl.cancel()
        if self.VideoSwitch==False: self.__sub_img.unregister()
        cv.destroyAllWindows()

    def compressed_callback(self, msg):
        if self.VideoSwitch == True: return
        if not isinstance(msg, CompressedImage): return
        start = time.time()
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        action = cv.waitKey(10) & 0xFF
        rgb_img, binary = self.process(frame, action)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(rgb_img, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        thread_text = "thread : " + str(len(threading.enumerate()))
        cv.putText(rgb_img, thread_text, (20, 60), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))
        else:cv.imshow(self.windows_name, rgb_img)

    def Reset(self):
        self.ros_ctrl.PWM_Reset()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.target_servox = 89
        self.target_servoy = 90
        rospy.loginfo("PWM init succes!!!")

    def execute(self, point_x,point_y,dist):
        t=0.01
        if abs(self.prev_dist - dist) > 5:
            self.prev_dist = dist
            return
        if abs(self.prev_angular - point_x) > 5:
            self.prev_angular = point_x
            return
        #if self.Joy_active == True: return
        linear_x = self.linear_pid.compute(dist, self.minDist)
        angular_z = self.angular_pid.compute(320, point_x)
        if abs(dist - self.minDist) < 30: linear_x = 0
        if abs(point_x - 320.0) < 30: angular_z = 0
        twist = Twist()
        twist.angular.z = angular_z
        twist.linear.x = linear_x
        self.pub_cmdVel.publish(twist)
        
        
        #t = 0.01
        #print (point_x)
        #rospy.loginfo("point_x: {}, point_y: {}".format(point_x, point_y))
        #[x_Pid, y_Pid] = self.PID_controller.update([point_x - 320, point_y - 240])
        #print(y_Pid)
        #print(self.circle)
        
        '''self.ros_ctrl.arm_ctrl(t * 1000, self.joint1, self.joint2, self.joint3)
        if self.img_flip == True:
            self.target_servox -= x_Pid
            
            self.target_servoy += y_Pid
            self.joint2 =y_Pid+2 
            

        else:
            self.target_servox += x_Pid
            self.target_servoy += y_Pid
        if self.target_servox >= 180:
            self.target_servox = 180
        elif self.target_servox <= 0:
            self.target_servox = 0
        if self.target_servoy >= 180:
            self.target_servoy = 180
        elif self.target_servoy <= 0:
            self.target_servoy = 0
        # rospy.loginfo("target_servox: {}, target_servoy: {}".format(self.target_servox, self.target_servoy))
        self.ros_ctrl.PWMServo_topic(1, self.target_servox)
        self.ros_ctrl.PWMServo_topic(2, self.target_servoy)
        self.ros_ctrl.arm_ctrl(t * 100, self.joint1, self.joint2, self.joint3)
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0
        self.ros_ctrl.cmdVelPublisher.publish(twist)
        print(self.joint2)'''
        
        
        

    '''def dynamic_reconfigure_callback(self, config, level):
        self.scale = config['scale']
        self.mono_PID = (config['Kp'], config['Ki'], config['Kd'])
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        self.PID_init()
        return config'''

    def PID_init(self):
        self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)
    
    
    '''def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.mono_PID[0] / float(self.scale), self.mono_PID[0] / float(self.scale)],
            [self.mono_PID[1] / float(self.scale), self.mono_PID[1] / float(self.scale)],
            [self.mono_PID[2] / float(self.scale), self.mono_PID[2] / float(self.scale)])'''

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x,y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'identify'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def process(self, rgb_img, action):
        # param action: [113 or 'q':退出]，[114 or 'r':重置]，[105 or 'i'：识别]，[32：开始追踪]
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
        if self.ros_ctrl.PWM_init == False: self.ros_ctrl.PWM_Reset()
        if action == 32: self.Track_state = 'tracking'
        elif action == ord('i') or action == 105: self.Track_state = "identify"
        elif action == ord('r') or action == 114: self.Reset()
        elif action == ord('q') or action == 113: self.cancel()
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
        if self.Track_state != 'init':
            if self.tracker_type == "color" and len(self.hsv_range) != 0:
                rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
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
            if self.circle[2] != 0: threading.Thread(target=self.execute, args=(self.circle[0], self.circle[1])).start()
            if self.point_pose[0] != 0 and self.point_pose[1] != 0: threading.Thread(target=self.execute, args=(self.point_pose[0], self.point_pose[1])).start()
        if self.tracker_type != "color": cv.putText(rgb_img, self.tracker_type + " Tracker", (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        return rgb_img, binary

if __name__ == '__main__':
    mono_Tracker = mono_Tracker()
    print(mono_Tracker.VideoSwitch)
    if mono_Tracker.VideoSwitch==False: rospy.spin()
    else:
        capture = cv.VideoCapture(0)
        cv_edition = cv.__version__
        if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
        while capture.isOpened():
            start = time.time()
            ret, frame = capture.read()
            action = cv.waitKey(10) & 0xFF
            frame, binary = mono_Tracker.process(frame, action)
            end = time.time()
            fps = 1 / (end - start)
            text = "FPS : " + str(int(fps))
            cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
            if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
            else:cv.imshow('frame', frame)
            if action == ord('q') or action == 113: break
        capture.release()
        cv.destroyAllWindows()
