#!/usr/bin/env python
# encoding: utf-8
import os
import threading
import platform
import getpass
import json
from follow_common import *
from geometry_msgs.msg import Twist
from std_msgs.msg import String
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
        
        # 添加ROS控制订阅器 - 监听simple_linefollow_controller的命令
        self.linefollow_cmd_sub = rospy.Subscriber('/linefollow_cmd', String, self.linefollow_cmd_callback)
        rospy.loginfo("LineDetect: ROS control subscriber initialized for /linefollow_cmd")
        
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
        if self.VideoSwitch == True:
            if time.time() - self.now_time > 5:
                rospy.loginfo("LineDetect node is running in headless mode (VideoSwitch=True)")
                self.now_time = time.time()
            return

        rospy.logdebug("Enter compressed_callback")
        if not isinstance(msg, CompressedImage): return
        start = time.time()
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        action = cv.waitKey(10) & 0xFF
        rgb_img, binary = self.process(frame,action)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(rgb_img, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)

        try:
            rospy.logdebug("Attempting to show cv window...")
            if len(binary) != 0:
                cv.imshow(self.windows_name, ManyImgs(1, ([rgb_img, binary])))
            else:
                cv.imshow(self.windows_name, rgb_img)
            rospy.logdebug("cv.imshow call successful.")
        except Exception as e:
            rospy.logerr("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            rospy.logerr("!!! OpenCV cv.imshow FAILED! Error: {}".format(e))
            rospy.logerr("!!! This is likely a DISPLAY/X11/GUI environment issue inside Docker.")
            rospy.logerr("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    def process(self, rgb_img, action):
        binary = []
        
        rgb_img = cv.resize(rgb_img, (640, 480))
        if self.PWM_init == False: 
            self.ros_ctrl.PWM_Reset()
            self.ros_ctrl.Arm_Reset()
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
        
        # 保留键盘控制作为备用（可选）
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
            
            # 使用可配置的误差处理参数
            error_scale = getattr(self, 'error_scale', 16)  # 默认值16
            image_center_x = getattr(self, 'image_center_x', 320)  # 默认值320
            dead_zone = getattr(self, 'dead_zone', 0)  # 默认值0
            
            # 计算误差
            error = point_x - image_center_x
            
            # 应用死区
            if abs(error) <= dead_zone:
                error = 0
                rospy.logdebug("LineDetect: Error within dead zone, setting to 0")
            
            # 计算PID输入
            pid_input = error / error_scale
            
            [z_Pid, _] = self.PID_controller.update([pid_input, 0])
            
            # 应用角速度限制（如果配置了的话）
            max_angular_speed = getattr(self, 'max_angular_speed', None)
            if max_angular_speed is not None:
                z_Pid = max(-max_angular_speed, min(max_angular_speed, z_Pid))
                rospy.logdebug("LineDetect: Angular velocity limited to {}".format(z_Pid))
            
            if self.img_flip == True: self.twist.angular.z = -z_Pid
            else: self.twist.angular.z = +z_Pid
            self.twist.linear.x = self.linear
            
            # 记录调试信息
            rospy.logdebug("LineDetect: point_x={}, error={}, pid_input={:.3f}, angular_z={:.3f}".format(
                point_x, error, pid_input, self.twist.angular.z))
            
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
        print ("Received config: ", config)
        print ("Linear speed from config: ", config['linear'])
        
        self.scale = config['scale']
        self.linear = config['linear']        
        self.FollowLinePID = (config['Kp'], config['Ki'], config['Kd'])
        self.hsv_range=((config['Hmin'],config['Smin'],config['Vmin']),
                        (config['Hmax'],config['Smax'],config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        print ("HSV: ",self.hsv_range)
        print ("Final linear speed set to: ", self.linear)
        self.PID_init()
        return config

    def linefollow_cmd_callback(self, msg):
        """处理来自simple_linefollow_controller的巡线控制命令"""
        try:
            rospy.loginfo("LineDetect received linefollow command: {}".format(msg.data))
            
            if not msg.data or not msg.data.strip():
                return
                
            cmd_data = json.loads(msg.data)
            cmd_type = cmd_data.get('cmd', '')
            
            if cmd_type == "start":
                rospy.loginfo("LineDetect: Starting line following")
                
                # 处理PID参数
                if 'pid_params' in cmd_data:
                    pid_params = cmd_data['pid_params']
                    rospy.loginfo("LineDetect: Received PID parameters: {}".format(pid_params))
                    
                    # 更新PID参数
                    if 'Kp' in pid_params:
                        self.FollowLinePID = (
                            pid_params.get('Kp', self.FollowLinePID[0]),
                            pid_params.get('Ki', self.FollowLinePID[1]),
                            pid_params.get('Kd', self.FollowLinePID[2])
                        )
                        rospy.loginfo("LineDetect: Updated PID parameters to Kp={}, Ki={}, Kd={}".format(
                            self.FollowLinePID[0], self.FollowLinePID[1], self.FollowLinePID[2]))
                    
                    # 更新缩放因子
                    if 'scale' in pid_params:
                        self.scale = pid_params['scale']
                        rospy.loginfo("LineDetect: Updated scale to {}".format(self.scale))
                    
                    # 重新初始化PID控制器
                    self.PID_init()
                    rospy.loginfo("LineDetect: PID controller reinitialized with new parameters")
                
                # 处理误差处理参数
                if 'error_params' in cmd_data:
                    error_params = cmd_data['error_params']
                    rospy.loginfo("LineDetect: Received error processing parameters: {}".format(error_params))
                    
                    # 可以根据需要处理误差参数，比如调整误差缩放因子
                    # 这里先记录，后续可以在execute函数中使用
                    self.error_scale = error_params.get('error_scale', 16)
                    self.image_center_x = error_params.get('image_center_x', 320)
                    self.dead_zone = error_params.get('dead_zone', 0)
                    rospy.loginfo("LineDetect: Updated error processing - scale={}, center_x={}, dead_zone={}".format(
                        self.error_scale, self.image_center_x, self.dead_zone))
                
                # 处理速度参数
                if 'speed' in cmd_data:
                    new_speed = float(cmd_data['speed'])
                    self.linear = new_speed
                    rospy.loginfo("LineDetect: Speed set to {}".format(new_speed))
                
                # 直接加载保存的HSV参数
                if os.path.exists(self.hsv_text):
                    self.hsv_range = read_HSV(self.hsv_text)
                    rospy.loginfo("LineDetect: Loaded HSV parameters from file: {}".format(self.hsv_range))
                    # 直接进入tracking模式
                    self.Track_state = 'tracking'
                    rospy.loginfo("LineDetect: Track_state set to 'tracking' - LINE FOLLOWING ACTIVE")
                else:
                    rospy.logwarn("LineDetect: No saved HSV parameters found, switching to identify mode for setup")
                    self.Track_state = 'identify'
                    rospy.logwarn("LineDetect: Track_state set to 'identify' - NEED HSV SETUP")
                
                # 添加状态确认
                rospy.loginfo("LineDetect: Current status - Track_state={}, HSV_range_loaded={}, PID=({},{},{})".format(
                    self.Track_state, 
                    len(self.hsv_range) > 0 if hasattr(self, 'hsv_range') else False,
                    self.FollowLinePID[0], self.FollowLinePID[1], self.FollowLinePID[2]
                ))
                
            elif cmd_type == "stop":
                rospy.loginfo("LineDetect: Stopping line following")
                self.Track_state = 'identify'
                # 发送停止命令
                self.ros_ctrl.pub_cmdVel.publish(Twist())
                
            elif cmd_type == "reset":
                rospy.loginfo("LineDetect: Resetting")
                self.Reset()
                
            elif cmd_type == "identify":
                rospy.loginfo("LineDetect: Switching to identify mode")
                self.Track_state = 'identify'
                
            elif cmd_type == "set_speed":
                # 单独设置速度命令
                if 'speed' in cmd_data:
                    new_speed = float(cmd_data['speed'])
                    self.linear = new_speed
                    rospy.loginfo("LineDetect: Speed updated to {}".format(new_speed))
                    # 重新初始化PID控制器以应用新速度
                    self.PID_init()
                    
            elif cmd_type == "update_pid":
                # 单独更新PID参数命令
                if 'pid_params' in cmd_data:
                    pid_params = cmd_data['pid_params']
                    rospy.loginfo("LineDetect: Updating PID parameters: {}".format(pid_params))
                    
                    self.FollowLinePID = (
                        pid_params.get('Kp', self.FollowLinePID[0]),
                        pid_params.get('Ki', self.FollowLinePID[1]),
                        pid_params.get('Kd', self.FollowLinePID[2])
                    )
                    
                    if 'scale' in pid_params:
                        self.scale = pid_params['scale']
                    
                    # 重新初始化PID控制器
                    self.PID_init()
                    rospy.loginfo("LineDetect: PID parameters updated and controller reinitialized")
                
            else:
                rospy.logwarn("LineDetect: Unknown command: {}".format(cmd_type))
                
        except json.JSONDecodeError as e:
            rospy.logerr("LineDetect: JSON parsing failed: {}".format(e))
        except Exception as e:
            rospy.logerr("LineDetect: Error processing linefollow command: {}".format(e))

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
