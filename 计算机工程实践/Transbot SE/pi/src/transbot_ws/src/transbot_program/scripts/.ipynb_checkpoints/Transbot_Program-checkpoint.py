#!/usr/bin/env python
# encoding: utf-8
import threading
from Follow_Line import *
from Mono_Tracker import *
from Color_Common import *
from Program_Ctrl import *
from Simple_AR import *
from Device import *
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from transbot_program.cfg import TransbotProgramConfig


class transbot_program:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        # 创建实例  Create instance
        self.color = ColorFollow()
        self.program_ctrl = ProgramCtrl()
        self.follow_line = FollowLine()
        self.mono_tracker = MonoTracker()
        # 相机设备 Camera equipment
        self.CameraDevice = GetDevice()
        # self.CameraDevice = 'Astra'
        self.simple_ar = SimpleAR(self.CameraDevice)
        self.Track_state = ""
        # 初始化参数 Initialization parameters
        self.circle = ()
        self.hsv_range = ()
        self.hsv_text = self.color.hsv_follow
        self.readtext_state = False
        self.text_state = False
        # 单词控制开关  Word control switch
        self.switch = False
        # 玩法类型 Type of play
        self.Function = ""
        # 是否旋转图像 Whether to rotate the image
        self.img_flip = False
        # 控制流程 Control process
        self.action = -1
        # 巡线参数 Tracking parameters
        self.LaserAngle = 35 # [0,180]
        self.ResponseDist = 0.6
        # 雷达避障参数 Lidar avoid parameters
#         Server(TransbotProgramConfig, self.Transbot_Program_Reconfig)
        rospy.loginfo("process start!!!")

    def visual_Tracker_setup(self, Function, img_flip=False):
        '''
        玩法参数设置 Play parameter settings
        :param img_flip: 是否水平翻转图像
        :param img_flip: Whether to flip the image horizontally
        '''
        self.Function = Function
        if self.Function=="Follow_Line":self.hsv_text = self.color.hsv_follow
        else:
            # self.Function = self.CameraDevice
            self.hsv_text = self.color.hsv_tracker
        self.img_flip = img_flip
        state = False
        if os.path.exists(self.hsv_text):
            try:
                self.hsv_range = read_HSV(self.hsv_text)
                self.readtext_state = True
                state = True
            except:
                rospy.logwarn("READ HSV ERROR!!!")
        return state

    def laser_Avoid_setup(self, LaserAngle=35, ResponseDist=0.6, speed=0.2):
        self.program_ctrl.PatrolWarning_topic("laser_Avoid", speed, LaserAngle, ResponseDist)

    def laser_Warning_setup(self, LaserAngle=35, ResponseDist=0.6):
        self.program_ctrl.PatrolWarning_topic("laser_Warning", 0.2, LaserAngle, ResponseDist)

    def laser_Tracker_setup(self, LaserAngle=35, ResponseDist=1.0):
        self.program_ctrl.PatrolWarning_topic("laser_Tracker", 0.2, LaserAngle, ResponseDist)

    def img_setup(self, Function):
        # 图像美化玩法设置 Function: ["other"]  Image beautification gameplay setting Function: ["other"]
        self.Function = Function

    def AR_setup(self, Graphics, flip=False):
        '''
        AR玩法设置 AR setting
        :param Graphics: ['Triangle', 'Rectangle', 'Parallelogram', 'Parallelogram',
                         'WindMill', 'TableTennisTable', 'Ball', 'Arrow',
                         'Knife', 'Desk', 'Bench', 'Stickman', 'ParallelBars']
        :param img_flip: 是否水平翻转图像  Whether to flip the image horizontally
        '''
        self.simple_ar.flip = flip
        self.simple_ar.Graphics = Graphics
        self.Function = "AR"
        if len(self.simple_ar.distCoeffs) == 0 and len(self.simple_ar.cameraMatrix) == 0: print ("No internal camera reference！")

    def patrol_setup(self, Commond, LineScaling=0.9, RotationScaling=1.0):
        '''
        巡逻玩法设置  Patrol settings
        :param Commond: 指令设置[Triangle, Square, Parallelogram, Circle]
        :param Commond: Command setting [Triangle, Square, Parallelogram, Circle]
        :param LineScaling: [0.0,2.0]
        :param RotationScaling: [0.0,2.0]
        '''
        self.Function = "patrol"
        patrol_result = self.program_ctrl.Patrol_srv(Commond, LineScaling, RotationScaling)
        return patrol_result

    def img_process(self, rgb_img):
        if self.Function == "AR": rgb_img = self.simple_ar.process(rgb_img)
        elif self.Function == "other": self.program_ctrl.pub_imgMsg(rgb_img)
        return rgb_img

    def visual_process(self, action, rgb_img):
        # param action: [113 or 'q':退出]，[114 or 'r':重置]，[105 or 'i'：识别]，[32：开始追踪]
        # param action: [113 or'q': exit], [114 or'r': reset], [105 or'i': identification], [32: start tracking]
        binary = rgb_img
        self.action = action
        #    if self.readtext_state == False:
        #        if os.path.exists(self.hsv_text):
        #            try:
        #                self.hsv_range = read_HSV(self.hsv_text)
        #                self.readtext_state = True
        #            except: rospy.logwarn("READ HSV ERROR!!!")
        rgb_img = cv.resize(rgb_img, (640, 480))
        if self.program_ctrl.PWM_init == False:
            if self.Function == "Follow_Line": self.program_ctrl.PWM_Reset()
            else: self.mono_tracker.Reset()
            self.program_ctrl.PWM_init = True
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
        if self.action == 32: self.Track_state = 'tracking'
        elif self.action == ord('i') or self.action == 105: self.Track_state = "identify"
        elif self.action == ord('r') or self.action == 114: self.Reset()
        elif self.action == ord('q') or self.action == 113: self.stop()
        if self.Track_state == 'init':
            if self.Function == "Follow_Line": (x, y, w, h) = (320, 360, 20, 20)
            else: (x, y, w, h) = (320, 240, 20, 20)
            Roi = (x - w, y - h, x + w, y + h)
            cv.rectangle(rgb_img, (Roi[0], Roi[1]), (Roi[2], Roi[3]), (0, 255, 0), 2)
            rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, Roi)
            # print("init hsv range:", self.hsv_range)
            self.text_state = False
        if self.Track_state != 'init':
            rgb_img, binary, self.circle = self.color.get_position(rgb_img, self.hsv_range, self.Function)
            if self.text_state == False:
                # print("write hsv:", self.hsv_range)
                try: write_HSV(self.hsv_text, self.hsv_range)
                except:rospy.logwarn("WRITE HSV ERROR!!!")
                self.text_state = True
        if self.Track_state == 'tracking' and len(self.circle) != 0:
            center_x = self.circle[0]
            center_y = self.circle[1]
            center_r = self.circle[2]
            threading.Thread(target=self.execute, args=(center_x,center_y,center_r)).start()
        return rgb_img, binary

    def execute(self, center_x, center_y, center_r):
        if self.program_ctrl.Joy_active: return
        if self.Function == "Follow_Line": self.follow_line.execute(center_x, center_r, self.img_flip)
        elif self.Function == "tracker": self.mono_tracker.execute(center_x, center_y, self.img_flip)

    def Transbot_Program_Reconfig(self, config, level):
        lin_pid = [config['lin_Kp']/1000.0, config['lin_Ki']/1000.0, config['lin_Kd']/1000.0]
        ang_pid = [config['ang_Kp']/1000.0, config['ang_Ki']/1000.0, config['ang_Kd']/1000.0]
        self.LaserAngle = config['laserAngle']
        self.ResponseDist = config['ResponseDist']
        if self.Function == "Follow_Line": self.follow_line.set_PID(lin_pid, config['linear'])
        elif self.Function == "tracker": self.mono_tracker.Set_PID(lin_pid, ang_pid)
        return config

    def stop(self):
        # print("--------------------------function stop--------------------------------")
        self.program_ctrl.PatrolWarning_topic("stop", 0.2, 45, 0.6)
        self.Function = " "
        self.Track_state = ""
        self.readtext_state = False

    def ResetPID(self):
        if self.Function =="Follow_Line": self.follow_line.PID_init()
        elif self.Function == "tracker":self.mono_tracker.PID_init()

    def Reset(self):
        sleep(0.2)
        self.ResetPID()
        self.circle = ()
        self.hsv_range = ()
        self.program_ctrl.PWM_init = False
        self.Track_state = 'init'
        #self.follow_line.Reset()
        self.program_ctrl.pub_cmdVel.publish(Twist())
        rospy.loginfo("Reset succes!!!")

    def cancel(self):
        self.program_ctrl.pub_cmdVel.publish(Twist())
        self.program_ctrl.Buzzer_srv(0)
        self.program_ctrl.cancel()

if __name__ == '__main__':
    # AR玩法图像函数调取 Call AR gameplay image function 
    # Triangle, Square, Parallelogram, Circle
    rospy.init_node("TransbotProgram", anonymous=False)
    transbot_program = transbot_program()
    # transbot_program.laser_Avoid_setup()
    transbot_program.laser_Warning_setup()
    # transbot_program.laser_Tracker_setup()
    # while not rospy.is_shutdown():
    #     transbot_program.laser_process()
    # setup = transbot_program.patrol_setup('Circle')
    # capture = cv.VideoCapture(0)
    # cv_edition = cv.__version__
    # print("cv_edition: ",cv_edition)
    # if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    # else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    # capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # print ("capture get FPS : ",capture.get(cv.CAP_PROP_FPS))
    # sleep(0.2)
    # # transbot_program.img_setup("openpose")
    # while capture.isOpened():
    #     start = time.time()
    #     ret, frame = capture.read()
    #     action = cv.waitKey(10) & 0xFF
    #     # transbot_program.AR_setup('Stickman')  
    #     transbot_program.visual_Tracker_setup("Follow_Line")  
    #     frame, binary = transbot_program.visual_process(action,frame)  
    #     frame = transbot_program.img_process(frame)  
    #     end = time.time()
    #     fps = 1 / (end - start)
    #     text = "FPS : " + str(int(fps))
    #     cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
    #     thread_text = "thread : " + str(len(threading.enumerate()))
    #     cv.putText(frame, thread_text, (30, 70), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
    #     # if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
    #     # else:cv.imshow('frame', frame)
    #     cv.imshow('frame', frame)
    #     if action == ord('q') or action == 113:
    #         transbot_program.cancel()
    #         break
    # capture.release()
    # cv.destroyAllWindows()
    rospy.spin()
