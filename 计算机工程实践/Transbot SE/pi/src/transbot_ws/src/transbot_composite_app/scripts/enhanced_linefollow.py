#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2 as cv
import threading
import time
import json
import os
import sys
import platform
import getpass
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client

# 导入transbot_linefollow的功能
sys.path.append('/root/transbot_ws/src/transbot_linefollow/scripts')
try:
    from follow_common import *
    from transbot_linefollow.cfg import LineDetectPIDConfig
except ImportError as e:
    rospy.logerr("Failed to import transbot_linefollow modules: {}".format(e))
    rospy.logerr("Make sure transbot_linefollow package is properly installed")
    sys.exit(1)

class EnhancedLineDetect:
    """
    增强版巡线检测器
    基于transbot_linefollow，增加ROS话题控制功能
    """
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("EnhancedLineDetect", anonymous=False)
        
        # 从原始LineDetect复制的变量
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
        self.windows_name = 'Enhanced_LineFollow'
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        
        # 新增：ROS话题控制功能
        self.track_state_external = 'identify'  # 外部控制的状态
        self.ros_control_enabled = True  # 是否启用ROS控制
        
        # 原始状态控制
        self.Track_state = 'identify'  # init, identify, tracking, mouse
        
        # 初始化ROS控制器和颜色检测器
        self.ros_ctrl = ROSCtrl()
        self.color = color_follow()
        
        # 参数设置
        self.user_name = getpass.getuser()
        text_path = '/root/transbot_ws/src/transbot_linefollow/scripts'
        self.hsv_text = text_path + "/LineFollowHSV.text"
        self.python_version = int(platform.python_version()[0])
        self.VideoSwitch = rospy.get_param("~VideoSwitch", False)
        self.img_flip = rospy.get_param("~img_flip", False)
        
        # 设备检测
        try:
            self.CameraDevice = self.ros_ctrl.CamDevice_srv("GetDevice!!!")
        except:
            rospy.logwarn("Camera device service not available")
        
        # 动态参数配置
        Server(LineDetectPIDConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client("EnhancedLineDetect", timeout=60)
        self.FollowLinePID = (40, 0, 12)
        self.linear = 0.3
        self.PID_init()
        
        # ROS订阅器和发布器
        self.setup_ros_interface()
        
        # 图像订阅
        img_topic = "/usb_cam/image_raw/compressed"
        if self.VideoSwitch == False:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.sub_img = rospy.Subscriber(img_topic, CompressedImage, self.compressed_callback)
        
        rospy.loginfo("Enhanced LineDetect node initialized successfully")
    
    def setup_ros_interface(self):
        """设置ROS接口"""
        # 订阅器
        self.state_cmd_sub = rospy.Subscriber('/linefollow_state_cmd', String, self.state_cmd_callback)
        self.enable_sub = rospy.Subscriber('/linefollow_enable', Bool, self.enable_callback)
        self.debug_toggle_sub = rospy.Subscriber('/linefollow_debug_toggle', Bool, self.debug_toggle_callback)
        
        # 发布器
        self.status_pub = rospy.Publisher('/enhanced_linefollow_status', String, queue_size=1)
        
        # 状态发布定时器
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self.publish_status)
    
    def state_cmd_callback(self, msg):
        """处理状态命令"""
        try:
            cmd_data = json.loads(msg.data)
            cmd_type = cmd_data.get('cmd', '')
            
            rospy.loginfo("Received state command: {}".format(cmd_type))
            
            if cmd_type == "start_tracking":
                self.track_state_external = 'tracking'
                rospy.loginfo("External command: START tracking")
            elif cmd_type == "stop_tracking":
                self.track_state_external = 'identify'
                rospy.loginfo("External command: STOP tracking")
            elif cmd_type == "reset":
                self.Reset()
                rospy.loginfo("External command: RESET")
            elif cmd_type == "toggle_debug":
                self.toggle_debug_window()
                rospy.loginfo("External command: TOGGLE debug window")
            else:
                rospy.logwarn("Unknown state command: {}".format(cmd_type))
                
        except json.JSONDecodeError as e:
            rospy.logerr("JSON parsing failed: {}".format(e))
        except Exception as e:
            rospy.logerr("Error processing state command: {}".format(e))
    
    def enable_callback(self, msg):
        """巡线使能回调"""
        if msg.data:
            self.track_state_external = 'tracking'
            rospy.loginfo("Line following ENABLED via topic")
        else:
            self.track_state_external = 'identify'
            rospy.loginfo("Line following DISABLED via topic")
    
    def debug_toggle_callback(self, msg):
        """调试窗口切换回调"""
        self.VideoSwitch = not msg.data  # msg.data=True表示显示窗口，VideoSwitch=False
        rospy.loginfo("Debug window toggled: {}".format("ON" if msg.data else "OFF"))
    
    def toggle_debug_window(self):
        """切换调试窗口"""
        self.VideoSwitch = not self.VideoSwitch
        rospy.loginfo("Debug window toggled: {}".format("OFF" if self.VideoSwitch else "ON"))
    
    def publish_status(self, event):
        """发布状态信息"""
        status_data = {
            "track_state": self.Track_state,
            "external_state": self.track_state_external,
            "ros_control_enabled": self.ros_control_enabled,
            "video_switch": self.VideoSwitch,
            "linefollow_active": (self.Track_state == 'tracking'),
            "timestamp": rospy.Time.now().to_sec()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
    
    def cancel(self):
        """关闭节点"""
        self.Reset()
        self.ros_ctrl.cancel()
        rospy.loginfo("Shutting down Enhanced LineDetect node.")
        if self.VideoSwitch == False:
            self.sub_img.unregister()
            cv.destroyAllWindows()
    
    def compressed_callback(self, msg):
        """图像回调函数"""
        if self.VideoSwitch == True:
            if time.time() - self.now_time > 5:
                rospy.loginfo("Enhanced LineDetect running in headless mode")
                self.now_time = time.time()
            return
        
        rospy.logdebug("Enhanced LineDetect: processing image")
        if not isinstance(msg, CompressedImage):
            return
            
        start = time.time()
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # 处理键盘输入（保持原有功能）
        action = cv.waitKey(10) & 0xFF
        
        # 处理图像
        rgb_img, binary = self.process(frame, action)
        
        # 显示FPS
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS: {} | State: {} | External: {}".format(
            int(fps), self.Track_state, self.track_state_external)
        cv.putText(rgb_img, text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        
        # 显示图像
        try:
            if len(binary) != 0:
                cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))
            else:
                cv.imshow(self.windows_name, rgb_img)
        except Exception as e:
            rospy.logerr("OpenCV display error: {}".format(e))
    
    def process(self, rgb_img, action):
        """图像处理函数"""
        binary = []
        
        rgb_img = cv.resize(rgb_img, (640, 480))
        if self.PWM_init == False:
            self.ros_ctrl.PWM_Reset()
            self.ros_ctrl.Arm_Reset()
        if self.img_flip == True:
            rgb_img = cv.flip(rgb_img, 1)
        
        # 处理键盘输入（保持原有功能）
        if action == 32:  # 空格键
            self.Track_state = 'tracking'
        elif action == ord('i') or action == 105:  # i键
            self.Track_state = "identify"
        elif action == ord('r') or action == 114:  # r键
            self.Reset()
        elif action == ord('q') or action == 113:  # q键
            self.cancel()
        elif action == ord('d') or action == 100:  # d键 - 切换调试窗口
            self.toggle_debug_window()
        
        # 使用外部控制状态（如果启用ROS控制）
        if self.ros_control_enabled:
            self.Track_state = self.track_state_external
        
        # 原有的图像处理逻辑
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.dyn_update = True
                else:
                    self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text):
                self.hsv_range = read_HSV(self.hsv_text)
            else:
                self.Track_state = 'init'
        
        if self.Track_state != 'init' and len(self.hsv_range) != 0:
            rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
            if self.dyn_update == True:
                write_HSV(self.hsv_text, self.hsv_range)
                if self.VideoSwitch == False:
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_client.update_configuration(params)
                self.dyn_update = False
        
        # 运动控制
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
        """鼠标事件处理"""
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
    
    def execute(self, point_x, color_radius):
        """执行运动控制"""
        self.now_time = time.time()
        if self.ros_ctrl.Joy_active == True:
            if self.Start_state == True:
                self.PID_init()
                self.Start_state = False
            return
        
        self.Start_state = True
        self.twist = Twist()
        if color_radius == 0:
            self.ros_ctrl.pub_cmdVel.publish(Twist())
        else:
            self.twist = Twist()
            [z_Pid, _] = self.PID_controller.update([(point_x - 320) / 16, 0])
            if self.img_flip == True:
                self.twist.angular.z = -z_Pid
            else:
                self.twist.angular.z = +z_Pid
            self.twist.linear.x = self.linear
            
            if self.warning > 10:
                rospy.loginfo("Obstacles ahead !!!")
                self.ros_ctrl.pub_cmdVel.publish(Twist())
                self.ros_ctrl.Buzzer_srv(1)
                self.Buzzer_state = True
            else:
                if self.Buzzer_state == True:
                    for i in range(3):
                        self.ros_ctrl.Buzzer_srv(0)
                    self.Buzzer_state = False
                self.ros_ctrl.pub_cmdVel.publish(self.twist)
    
    def Reset(self):
        """重置函数"""
        self.PID_init()
        self.PWM_init = False
        self.Track_state = 'init'
        self.track_state_external = 'identify'
        self.hsv_range = ()
        self.ros_ctrl.Joy_active = False
        self.Mouse_XY = (0, 0)
        self.ros_ctrl.Buzzer_srv(0)
        self.ros_ctrl.pub_cmdVel.publish(Twist())
        rospy.loginfo("Enhanced LineDetect reset successful!")
    
    def PID_init(self):
        """PID初始化"""
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])
    
    def dynamic_reconfigure_callback(self, config, level):
        """动态参数配置回调"""
        rospy.loginfo("Enhanced LineDetect: dynamic_reconfigure_callback")
        self.scale = config['scale']
        self.linear = config['linear']
        self.FollowLinePID = (config['Kp'], config['Ki'], config['Kd'])
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                         (config['Hmax'], config['Smax'], config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        rospy.loginfo("HSV: {}".format(self.hsv_range))
        self.PID_init()
        return config

if __name__ == '__main__':
    try:
        line_detect = EnhancedLineDetect()
        if line_detect.VideoSwitch == False:
            rospy.spin()
        else:
            # 无头模式，使用摄像头直接捕获
            capture = cv.VideoCapture(0)
            cv_edition = cv.__version__
            if cv_edition[0] == '3':
                capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
            else:
                capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
            capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
            
            while capture.isOpened() and not rospy.is_shutdown():
                start = time.time()
                ret, frame = capture.read()
                if ret:
                    action = cv.waitKey(10) & 0xFF
                    frame, binary = line_detect.process(frame, action)
                    end = time.time()
                    fps = 1 / (end - start)
                    text = "FPS: {}".format(int(fps))
                    cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
                    if len(binary) != 0:
                        cv.imshow('Enhanced_LineFollow', ManyImgs(1, ([frame, binary])))
                    else:
                        cv.imshow('Enhanced_LineFollow', frame)
                    if action == ord('q') or action == 113:
                        line_detect.cancel()
                        break
                rospy.sleep(0.01)
            
            capture.release()
            cv.destroyAllWindows()
    except Exception as e:
        rospy.logerr("Enhanced LineDetect error: {}".format(e)) 