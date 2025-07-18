#!/usr/bin/env python3
# coding=utf-8
# Version: V3.2.8
from flask import Flask, render_template, Response
import socket
import os
import time
import threading
import cv2 as cv
import sys
import struct
import inspect
import ctypes

from Transbot_Lib import Transbot
from camera_transbot import Transbot_Camera
from wifi_transbot import Transbot_WIFI
from seek_transbot import Transbot_Seek
from arm_transbot import Transbot_ARM
from file_transbot import Transbot_File
# from oled_transbot import Transbot_OLED

sys.path.append('/root/.local/lib/python2.7/site-packages')
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append("/opt/ros/melodic/bin")
sys.path.append("/root/transbot_ws/src/transbot_program/scripts")
sys.path.append("/root/transbot_ws/src/transbot_bringup/scripts")
import rospy
from gevent import pywsgi


g_debug = True#False
if len(sys.argv) > 1:
    if str(sys.argv[1]) == "debug":
        g_debug = True
print("debug=", g_debug)

g_arm = Transbot_ARM(debug=g_debug)
g_arm_offset = g_arm.get_arm_offset()
# 小车底层处理库
# g_bot = Transbot(com="/dev/ttyUSB0")
g_bot = Transbot(arm_offset=g_arm_offset)
# 启动线程接收串口数据
# g_bot.create_receive_threading()

# WIFI配网库
g_wifi = Transbot_WIFI(g_bot, debug=g_debug)

# 摄像头库
g_camera = Transbot_Camera(debug=g_debug)

# OLED库
# g_oled = Transbot_OLED(debug=g_debug)

# 搜索连接库
g_seek = Transbot_Seek(debug=g_debug)

# 文件数据管理
g_file = Transbot_File(debug=g_debug)


g_ip_addr = "x.x.x.x"
g_tcp_ip = g_ip_addr

g_wifi_state = False
g_init = False
g_mode = 'Standard'
g_yaw_state = False

from Device import *
g_camera_type = GetDevice()
# 摄像头型号, 深度相机：Astra, 普通相机：USBCam
# g_camera_type = "USBCam"
if g_debug:
    print("camera_type:", g_camera_type)

# wifi 配网的线程。
#def thread_wifi():
    #global g_camera, g_wifi_state, g_init, g_ip_addr
    #ip_init = '0'
    #while True:
        #if g_wifi.read_mode():
            #if not g_wifi_state:
                #if g_debug:
                    #print("-------------------Start Scanf WIFI--------------------------")
                #g_wifi_state = True
                #g_bot.set_pwm_servo(1, 90)
                #g_bot.set_pwm_servo(2, 90)

            #if g_wifi_state:
                #succeed, frame = g_camera.get_frame()
                #if succeed:
                    #ip_init, _ = g_wifi.connect(frame)
            #if ip_init != 'x.x.x.x':
                #g_wifi_state = False
                #g_init = False
                #if g_debug:
                    #print("-------------------Connect WIFI END--------------------------")
        #else:
            #g_ip_addr = g_wifi.led_show_state()
            #time.sleep(.5)

#task_wifi = threading.Thread(target=thread_wifi, name="task_wifi")
#task_wifi.setDaemon(True)
#task_wifi.start()


###################################################################################################
# roslaunch启动线程
def thread_roslaunch():
    cmd_ros = 'roslaunch transbot_program Transbot_Program.launch'
    # cmd_ros = 'roslaunch transbot_program Transbot_Program.launch open_depth:=False'
#     if g_camera_type == 'Astra':
#         cmd_ros = 'roslaunch transbot_program Transbot_Program.launch open_depth:=True'
    os.system(cmd_ros)
    if g_debug:
        print("---roslaunch end---")

if g_debug:
    print("start task_roslaunch")
task_roslaunch = threading.Thread(target=thread_roslaunch, name="task_roslaunch")
task_roslaunch.setDaemon(True)
task_roslaunch.start()
time.sleep(5)
time.sleep(5)
time.sleep(2)
if g_debug:
    print("task_roslaunch pass")
###################################################################################################

rospy.init_node('BigTransbot', anonymous=False, disable_signals=True)
if g_debug:
    print("------init_node BigTransbot------")

from Transbot_Program import transbot_program
g_program = transbot_program()

from Transbot_info import TransbotInfo
g_info = TransbotInfo()

from Color_Common import *
g_color_updateHSV = ColorFollow()
g_tracker_hsv = ((0, 0, 0), (0, 0, 0))
g_follow_line_hsv = ((0, 0, 0), (0, 0, 0))
g_color_calibration_binary = False
g_color_index = 1
g_color_hsv_temp = ((0, 0, 0), (0, 0, 0))

g_follow_line_pid = g_file.read_Follow_Line_PID()
print("read follow line pid:", g_follow_line_pid)

try: 
    g_follow_line_hsv = read_HSV(g_color_updateHSV.hsv_follow)
except Exception: 
    if g_debug:
        print("Read follow_line HSV_config Error!!!")
try: 
    g_tracker_hsv = read_HSV(g_color_updateHSV.hsv_tracker)
except Exception: 
    if g_debug:
        print("Read tracker HSV_config Error!!!")

if g_debug:
    print("tracker_hsv:", g_tracker_hsv)
    print("follow_line_hsv", g_follow_line_hsv)

# AR功能参数
# None, 长方形，平行四边形，三角形，球体，箭头，小刀，风车，乒乓球台，双杠，火柴人，桌子，椅子
g_AR_type = ["None", "Rectangle", "Parallelogram", "Triangle", "Ball", "Arrow", "Knife", \
             "WindMill", "TableTennisTable", "ParallelBars", "Stickman", "Desk", "Bench"]
g_AR_Graphics = g_AR_type[0]

# 巡逻参数:
g_patrol_type = ["finish", "Square", "Parallelogram", "Triangle", "Circle"]


# 功能玩法状态
g_img_state = False
g_laser_avoid_state = False
g_laser_tracker_state = False
g_laser_warning_state = False
g_ar_state = False
g_visual_state = False
g_patrol_state = False
g_follow_line_state = False


# [114 or 'r':重置]
# [105 or 'i'：识别]
# [32：开始追踪]
g_visual_action = 114 # 重新选择
g_follow_line_action = 114 # 重新选择
g_follow_line_binary = False
g_action_count = 0

app = Flask(__name__)

# 舵机云台相关变量
g_servo_x = 90
g_servo_y = 90
g_last_x = 0
g_last_y = 0
g_servo_state = 0

# 速度控制
g_speed_ctrl = 100
# TCP未接收命令超时计数
g_tcp_except_count = 0


# 返回单片机版本号, tcp=TCP服务对象
def return_bot_version(tcp):
    data = "$0104"
    # version = int(g_bot.get_version() * 10)
    version = int((g_info.edition + 0.05) * 10) % 256
    if version < 0:
        version = 0
    checknum = (5 + version) % 256
    data1 = "%02x%02x#" % (version, checknum)
    data = data + data1
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("Transbot Version:", version / 10.0)
        print("tcp send:", data)

# 返回电池电压
def return_battery_voltage(tcp):
    data = "$0204"
    # vol = int(g_bot.get_battery_voltage() * 10) % 256
    vol = int((g_info.voltage + 0.05) * 10) % 256
    if vol < 0:
        vol = 0
    checknum = (6 + vol) % 256
    data1 = "%02x%02x#" % (vol, checknum)
    data = data + data1
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("voltage:", vol / 10.0)
        print("tcp send:", data)
    return vol / 10.0

# 返回小车速度控制百分比
def return_car_speed(tcp, speed):
    data = "$1604"
    checknum = (26 + speed) % 256
    data1 = "%02x%02x#" % (speed, checknum)
    data = data + data1
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("speed:", speed)
        print("tcp send:", data)
    return speed


# 返回机械臂角度
def return_arm_angle(tcp):
    # angle = g_bot.get_uart_servo_angle_array()
    angle = g_info.RobotArmSrv("")
    if len(angle) != 3:
        for i in range(3):
            for j in range(5):
                # g_bot.set_uart_servo_angle(9, 90)
                g_bot.set_uart_servo_angle_array(225, 30, 90)
                time.sleep(.1)
            time.sleep(.3)
            angle = g_info.RobotArmSrv("")
            if len(angle) == 3:
                if g_debug:
                    print("Arm read angle OK:", i + 1)
                break
            if i == 2:
                if g_debug:
                    print("Arm read angle ERROR:", i + 1)
                angle = [-1, -1, -1]
    else:
        if g_debug:
            print("Arm read angle OK")
    angle_s1 = bytearray(struct.pack('h', int(angle[0])))
    angle_s2 = bytearray(struct.pack('h', int(angle[1])))
    angle_s3 = bytearray(struct.pack('h', int(angle[2])))
    data = "$140e"
    checknum = (34 + angle_s1[0] + angle_s1[1] + angle_s2[0] + angle_s2[1] + angle_s3[0] + angle_s3[1]) % 256
    data1 = "%02x%02x%02x%02x%02x%02x%02x#" % \
            (angle_s1[0], angle_s1[1], angle_s2[0], angle_s2[1], angle_s3[0], angle_s3[1], checknum)
    data = data + data1
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("return angle_s1-s3:", angle)
        print("tcp send:", data)

# 返回颜色校准HSV值，index=1自动驾驶，index=2智能追踪
def return_hsv(tcp, index):
    h_min = s_min = v_min = h_max = s_max = v_max = 0
    if index == 1:
        min, max = g_follow_line_hsv
        h_min = int(min[0])
        s_min = int(min[1])
        v_min = int(min[2])
        h_max = int(max[0])
        s_max = int(max[1])
        v_max = int(max[2])
    elif index == 2:
        min, max = g_tracker_hsv
        h_min = int(min[0])
        s_min = int(min[1])
        v_min = int(min[2])
        h_max = int(max[0])
        s_max = int(max[1])
        v_max = int(max[2])
    else:
        if g_debug:
            print("-----color request index error!-----")
        return
    data = "$9210"
    checknum = (162 + index + h_min + s_min + v_min + h_max + s_max + v_max) % 256
    data1 = "%02x%02x%02x%02x%02x%02x%02x%02x#" % \
            (index, h_min, s_min, v_min, h_max, s_max, v_max, checknum)
    data = data + data1
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("return hsv:", index)
        print("tcp send:", data)

# 返回自动驾驶的PID参数给APP
def return_follow_line_pid(tcp):
    kp = int(g_follow_line_pid[0]) % 256
    ki = int(g_follow_line_pid[1]) % 256
    kd = int(g_follow_line_pid[2]) % 256
    data = "$8408"
    checknum = (140 + kp + ki + kd) % 256
    data1 = "%02x%02x%02x%02x#" % (kp, ki, kd, checknum)
    data = data + data1
    tcp.send(data.encode(encoding="utf-8"))
    if g_debug:
        print("return pid:", kp, ki, kd)
        print("tcp send:", data)

# 数值变换
def my_map(x, in_min, in_max, out_min, out_max):
    return int((out_max - out_min) * (x - in_min) / (in_max - in_min) + out_min)


# 创建主动停止线程的方法
def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


# 结束子线程
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


def stop_mode():
    g_program.stop()
    time.sleep(.05)
    g_bot.set_car_motion(0, 0)
    g_bot.set_beep(0)
    if g_debug:
        print("---stop_mode---")

# state:=0舵机云台归中，=1自动驾驶位置，=2智能追踪位置。
def set_pwm_servo_state(state):
    global g_last_x, g_last_y, g_servo_x, g_servo_y
    if state == 0:
        g_servo_x = g_last_x = 90
        g_servo_y = g_last_y = 90
        g_bot.set_pwm_servo(1, g_servo_x)
        g_bot.set_pwm_servo(2, g_servo_y)
    elif state == 1:
        g_servo_x = g_last_x = 90
        g_servo_y = g_last_y = 70
        g_bot.set_pwm_servo(1, g_servo_x)
        g_bot.set_pwm_servo(2, g_servo_y)
    elif state == 2:
        g_servo_x = g_last_x = 90
        g_servo_y = g_last_y = 90
        g_bot.set_pwm_servo(1, g_servo_x)
        g_bot.set_pwm_servo(2, g_servo_y)

# 控制舵机云台的线程
def thread_pwm_servo():
    global g_servo_state, g_last_x, g_last_y, g_servo_x, g_servo_y
    limit_x_max = 180
    limit_x_min = 0
    if g_camera_type == "Astra":
        limit_x_max = 130
        limit_x_min = 50
    while True:
        if g_servo_state != 0:
            if g_servo_state == 11:  # 向右
                g_servo_x = g_servo_x - 1
                if g_servo_x < limit_x_min:
                    g_servo_x = limit_x_min
            elif g_servo_state == 12:  # 向左
                g_servo_x = g_servo_x + 1
                if g_servo_x > limit_x_max:
                    g_servo_x = limit_x_max
            elif g_servo_state == 21:  # 向上
                g_servo_y = g_servo_y + 1
                if g_servo_y > 150:
                    g_servo_y = 150
            elif g_servo_state == 22:  # 向下
                g_servo_y = g_servo_y - 1
                if g_servo_y < 55:
                    g_servo_y = 55

        if g_servo_x != g_last_x:
            g_bot.set_pwm_servo(1, g_servo_x)
            g_last_x = g_servo_x
            if g_debug:
                print("servo_x:", g_servo_x)
        if g_servo_y != g_last_y:
            g_bot.set_pwm_servo(2, g_servo_y)
            g_last_y = g_servo_y
            if g_debug:
                print("g_servo_y:", g_servo_y)
        time.sleep(.05)

# 巡逻线程
def thread_patrol(patrol_type, num_line_s, num_angular_s):
    global g_patrol_state
    if g_debug:
        print("patrol start state=", g_patrol_state)
    g_program.patrol_setup(patrol_type, num_line_s, num_angular_s)
    g_patrol_state = False
    if g_debug:
        print("patrol end state=", g_patrol_state)


# 协议解析部分
def parse_data(sk_client, data):
    # print(data)
    global g_mode, g_bot, g_camera
    global g_servo_state
    global g_AR_type, g_AR_Graphics
    global g_img_state, g_ar_state
    global g_laser_avoid_state, g_laser_tracker_state, g_laser_warning_state
    global g_patrol_state
    global g_visual_state, g_visual_action
    global g_follow_line_state, g_follow_line_action, g_follow_line_binary, g_follow_line_pid
    global g_speed_ctrl, g_yaw_state
    global g_color_calibration_binary, g_tracker_hsv, g_follow_line_hsv, g_color_hsv_temp, g_color_index

    cmd = data[1:3]
    length = data[3:5]
    num_cmd = int(cmd, 16)
    num_len = int(length, 16)
    num_checknum = int(data[num_len + 3:num_len + 5], 16)
    if cmd == "0F":  # 回到主界面
        func = int(data[5:7])
        if g_debug:
            print("cmd func=", func)
        if func == 0:
            g_mode = 'Home'
            if g_debug:
                print("Go Home")
            g_follow_line_binary = False
            g_color_calibration_binary = False
            if g_visual_action == 32:
                g_visual_action = 105
            if g_follow_line_action == 32:
                g_follow_line_action = 105
            stop_mode()
            return_battery_voltage(g_socket)
            
        else:
            if func == 8:
                g_mode = 'follow_line_mode'
                if g_program.visual_Tracker_setup("Follow_Line", False):
                    g_follow_line_action = 105 # 确认颜色，如果从text文件读出来数据
                set_pwm_servo_state(1)
                return_follow_line_pid(g_socket)
                if g_debug:
                    print("go to follow_line_mode")
            elif func == 3:
                g_mode = "img_mode"
                g_program.img_setup("other")
                if g_debug:
                    print("go to img_mode")
            elif func == 1:
                return_car_speed(sk_client, g_speed_ctrl)
            elif func == 9:
                try: 
                    g_tracker_hsv = read_HSV(g_color_updateHSV.hsv_tracker)
                    g_follow_line_hsv = read_HSV(g_color_updateHSV.hsv_follow)
                    g_color_hsv_temp = g_follow_line_hsv
                    g_color_index = 1
                except Exception: 
                    if g_debug:
                        print("Read HSV_config Error!!!")
                if g_debug:
                    print("tracker_hsv:", g_tracker_hsv)
                    print("follow_line_hsv", g_follow_line_hsv)
                g_mode = "color_mode"
            elif func == 6:
                set_pwm_servo_state(2)
                g_mode = 'visual_mode'
                tracker_state = g_program.visual_Tracker_setup("tracker", False)
                if tracker_state:
                    g_visual_action = 105 # 确认颜色，如果从text文件读出数据


    elif cmd == "01":  # 获取硬件版本号
        if num_cmd + num_len == num_checknum:
            return_bot_version(g_bot, sk_client)

    elif cmd == "02":  # 获取电池电压
        if num_cmd + num_len == num_checknum:
            return_battery_voltage(sk_client)

    elif cmd == "10":  # 控制小车
        num_line_speed = int(data[5:7], 16)
        num_angular_speed = int(data[7:9], 16)
        checknum = (num_cmd + num_len + num_line_speed + num_angular_speed) % 256
        if checknum == num_checknum:
            line = my_map(num_line_speed, 0, 200, -450, 450) / 1000.0
            angular = my_map(num_angular_speed, 0, 200, -200, 200) / 100.0
            if -0.1 < line < 0.1:
                line = 0
            if -0.5 < angular < 0.5:
                angular = 0
            if line == 0:
                if g_yaw_state:
                    g_yaw_state = False
                    g_bot.set_imu_adjust(False)
            else:
                if not g_yaw_state:
                    g_yaw_state = True
                    g_bot.set_imu_adjust(True)
            g_bot.set_car_motion(line, angular)
            if g_debug:
                print("line_speed:%.2f, angular_speed:%.2f" % (line, angular))
        else:
            print("num_checknum error!", checknum, num_checknum)

    # 控制舵机云台
    elif cmd == "11":
        num_id = int(data[5:7], 16)
        num_angle = int(data[7:9], 16)
        checknum = (num_cmd + num_len + num_id + num_angle) % 256
        if checknum == num_checknum:
            if num_angle == 0:
                g_servo_state = 0
            else:
                g_servo_state = num_id * 10 + num_angle
            if num_id == 3 and num_angle == 3:
                set_pwm_servo_state(0)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("pwm servo id:%d, angle:%d" % (num_id, num_angle))

    # 控制机械臂
    elif cmd == "12":
        num_id = int(data[5:7], 16)
        num_angle_l = int(data[7:9], 16)
        num_angle_h = int(data[9:11], 16)
        checknum = (num_cmd + num_len + num_id + num_angle_l + num_angle_h) % 256
        if checknum == num_checknum:
            uart_servo_angle = num_angle_h * 256 + num_angle_l
            g_bot.set_uart_servo_angle(num_id + 6, uart_servo_angle)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("uart servo id:%d, angle:%d" % (num_id, uart_servo_angle))

    # 设置蜂鸣器
    elif cmd == "13":
        num_state = int(data[5:7], 16)
        num_delay = int(data[7:9], 16)
        checknum = (num_cmd + num_len + num_state + num_delay) % 256
        if checknum == num_checknum:
            delay_ms = 0
            if num_state > 0:
                if num_delay == 255:
                    delay_ms = 1
                else:
                    delay_ms = num_delay * 10
            g_bot.set_beep(delay_ms)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("beep:%d, delay:%d" % (num_state, num_delay))

    # 读取机械臂舵机角度。
    elif cmd == "14":
        num_id = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_id) % 256
        if checknum == num_checknum:
            if num_id == 3:
                return_arm_angle(sk_client)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("read angle:%d" % num_id)
    
    # 按键控制
    elif cmd == "15":
        num_dir = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_dir) % 256
        if checknum == num_checknum:
            speed_l = 0.0
            speed_a = 0.0
            if num_dir == 0:
                g_bot.set_car_motion(0, 0)
                # time.sleep(.003)
                # g_bot.set_imu_adjust(False, False)
            elif num_dir == 1:
                speed_l = 0.45 * g_speed_ctrl / 100.0
                g_bot.set_car_run(speed_l)
                # g_bot.set_car_motion(speed_l, 0)
                # time.sleep(.003)
                # g_bot.set_imu_adjust(True, False)
            elif num_dir == 2:
                speed_l = -0.45 * g_speed_ctrl / 100.0
                g_bot.set_car_run(speed_l)
                # g_bot.set_car_motion(speed_l, 0)
                # time.sleep(.003)
                # g_bot.set_imu_adjust(True, False)
            elif num_dir == 3:
                speed_l = 0.2
                speed_a = 1.5 * g_speed_ctrl / 100.0
                g_bot.set_car_motion(speed_l, speed_a)
            elif num_dir == 4:
                speed_l = 0.2
                speed_a = -1.5 * g_speed_ctrl / 100.0
                g_bot.set_car_motion(speed_l, speed_a)
            elif num_dir == 5:
                speed_a = 2 * g_speed_ctrl / 100.0
                g_bot.set_car_motion(0, speed_a)
            elif num_dir == 6:
                speed_a = -2 * g_speed_ctrl / 100.0
                g_bot.set_car_motion(0, speed_a)
            if g_debug:
                print("car speed:%.2f, %.2f" % (speed_l, speed_a))
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("btn ctl:%d" % num_dir)
    
    # 控制速度
    elif cmd == '16':
        num_speed = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_speed) % 256
        if checknum == num_checknum:
            g_speed_ctrl = num_speed
            if g_speed_ctrl > 100:
                g_speed_ctrl = 100
            if g_speed_ctrl < 0:
                g_speed_ctrl = 0
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("speed ctl:%d" % num_speed)

    # 控制大灯
    elif cmd == '17':
        num_light = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_light) % 256
        if checknum == num_checknum:
            if num_light > 100:
                num_light = 100
            g_bot.set_floodlight(num_light)
            # if num_light == 0:
            #     g_bot.set_imu_adjust(False, False)
            # else:
            #     g_bot.set_imu_adjust(True, False)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("big led:%d" % num_light)
    
    # 设置彩色灯带的颜色
    elif cmd == "20":
        num_id = int(data[5:7], 16)
        num_r = int(data[7:9], 16)
        num_g = int(data[9:11], 16)
        num_b = int(data[11:13], 16)
        checknum = (num_cmd + num_len + num_id + num_r + num_g + num_b) % 256
        if checknum == num_checknum:
            g_bot.set_colorful_lamps(num_id, num_r, num_g, num_b)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("lamp:%d, r:%d, g:%d, b:%d" % (num_id, num_r, num_g, num_b))

    # 设置彩色灯带的特效
    elif cmd == "21":
        num_effect = int(data[5:7], 16)
        num_speed = int(data[7:9], 16)
        checknum = (num_cmd + num_len + num_effect + num_speed) % 256
        if checknum == num_checknum:
            g_bot.set_colorful_effect(num_effect, num_speed, 255)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("effect:%d, speed:%d" % (num_effect, num_speed))

    # 设置彩色灯带的单色呼吸灯效果的颜色
    elif cmd == "22":
        num_color = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_color) % 256
        if checknum == num_checknum:
            if num_color == 0:
                g_bot.set_colorful_effect(0, 255, 255)
            else:
                g_bot.set_colorful_effect(3, 255, num_color - 1)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("breath color:%d" % num_color)

    # 保存图片
    elif cmd == "30":
        num_save = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_save) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("save photo:%d" % num_save)

    # 查看图片
    elif cmd == "31":
        num_id = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_id) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            if 1 < num_id < 3:
                g_program.img_setup("other")
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("show photo:%d" % num_id)

    # 激光雷达, 玩法模式
    #elif cmd == "40":
        # mode:1：雷达避障，2：雷达跟踪，3：雷达警卫
        #num_mode = int(data[5:7], 16)
        #num_distance = int(data[7:9], 16) # 需要除以100.0得到真实数据
        #num_angle = int(data[9:11], 16)   # 真实数据
        #num_speed = int(data[11:13], 16)  # 需要除以100.0得到真实数据
        #checknum = (num_cmd + num_len + num_mode + num_distance + num_angle + num_speed) % 256

        #if checknum == num_checknum:
            #g_bot.set_beep(20)
            #if num_mode == 0: # 玩法关闭
                #g_mode = 'Home'
                #stop_mode()
                #stop_mode()
            #elif num_mode == 1:
                #g_program.laser_Avoid_setup(num_angle, num_distance/100.0, num_speed/100.0)
                #g_mode = 'laser_mode'
            #elif num_mode == 2:
                #g_program.laser_Tracker_setup(num_angle, num_distance/100.0)
                #g_mode = 'laser_mode'
            #elif num_mode == 3:
                #g_program.laser_Warning_setup(num_angle, num_distance/100.0)
                #g_mode = 'laser_mode'
        #else:
            #if g_debug:
                #print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        #if g_debug:
            #print("radar mode:%d" % num_mode)

    # # 激光雷达, 检测距离
    # elif cmd == "41":
    #     num_distance = int(data[5:7], 16)
    #     checknum = (num_cmd + num_len + num_distance) % 256
    #     if checknum == num_checknum:
    #         g_bot.set_beep(20)
    #     else:
    #         if g_debug:
    #             print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
    #     if g_debug:
    #         print("radar mode:%d" % num_distance)

    # # 激光雷达, 扫描角度
    # elif cmd == "42":
    #     num_angle = int(data[5:7], 16)
    #     checknum = (num_cmd + num_len + num_angle) % 256
    #     if checknum == num_checknum:
    #         g_bot.set_beep(20)
    #     else:
    #         if g_debug:
    #             print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
    #     if g_debug:
    #         print("radar mode:%d" % num_angle)

    # # 激光雷达, 移动速度
    # elif cmd == "43":
    #     num_speed = int(data[5:7], 16)
    #     checknum = (num_cmd + num_len + num_speed) % 256
    #     if checknum == num_checknum:
    #         g_bot.set_beep(20)
    #     else:
    #         if g_debug:
    #             print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
    #     if g_debug:
    #         print("radar mode:%d" % num_speed)

    # AR世界，切换图形
    elif cmd == "50":
        num_index = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_index) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            if 0 <= num_index < 13:
                g_AR_Graphics = g_AR_type[num_index]
                g_program.AR_setup(g_AR_Graphics, flip=False)
                if num_index == 0:
                    g_mode = 'Home'
                else:
                    g_mode = 'ar_mode'
                if g_debug:
                    print("g_AR_Graphics=", g_AR_Graphics)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("AR mode:%d" % num_index)

    # 智能追踪，取色追踪
    elif cmd == "60":
        num_switch = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_switch) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            if num_switch == 1:
                g_mode = 'visual_mode'
                tracker_state = g_program.visual_Tracker_setup("tracker", False)
                if tracker_state:
                    g_visual_action = 105 # 确认颜色，如果从text文件读出数据
                if g_debug:
                    print("tracker_state:", tracker_state)
            else:
                g_mode = 'Home'
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("trace color:%d" % num_switch)

    # 智能追踪，重选/确认颜色
    elif cmd == "61":
        num_verify = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_verify) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            if num_verify == 1: 
                g_visual_action = 105 # 确认颜色
            else:
                g_visual_action = 114 # 重新选择
            g_mode = 'visual_mode'
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("trace color verify:%d" % num_verify)

    # 智能追踪，小车运动开关
    elif cmd == "62":
        num_start = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_start) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            if num_start == 1:
                if g_visual_action == 105 and g_mode == 'visual_mode':
                    g_visual_action = 32 # 开始追踪
                    if g_debug:
                        print("color trace start")
            else:
                g_visual_action = 105 # 关闭追踪，确认颜色
                time.sleep(.001)
                g_bot.set_car_motion(0, 0)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("trace color start:%d" % num_start)

    # 智能巡逻，设置形状
    elif cmd == "70":
        num_index = int(data[5:7], 16)
        num_line_s = int(data[7:9], 16)
        num_angular_s = int(data[9:11], 16)
        checknum = (num_cmd + num_len + num_index + num_line_s + num_angular_s) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            g_mode = 'Home'
            # if g_debug:
            #     print("g_patrol_state:" , g_patrol_state)
            if 0 < num_index < 5:
                if not g_patrol_state:
                    patrol_type = g_patrol_type[num_index]
                    if g_debug:
                        print("start patrol type:", patrol_type)
                    task_patrol = threading.Thread(target=thread_patrol, name="task_patrol", args=(patrol_type, num_line_s/10.0, num_angular_s/10.0))
                    task_patrol.setDaemon(True)
                    task_patrol.start()
                    g_patrol_state = True
            else:
                stop_mode()
                stop_mode()
                if g_debug:
                    print("---stop_patrol---")
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("patrol shape:%d, %2f, %.2f" % (num_index, num_line_s/10.0, num_angular_s/10.0))

    # 自动驾驶，小车运动开关
    elif cmd == "80":
        num_start = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_start) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            g_mode = 'follow_line_mode'
            if num_start == 1:
                g_follow_line_action = 32 # 小车开启
            else:
                g_follow_line_action = 105 # 小车停止，确认颜色
                g_bot.set_beep(0)
                time.sleep(.1)
                g_bot.set_car_motion(0, 0)
                g_program.ResetPID()
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("auto drive:%d" % num_start)

    # 自动驾驶，画面切换
    elif cmd == "81":
        num_binary = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_binary) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            if num_binary == 1:
                g_follow_line_binary = True
            else:
                g_follow_line_binary = False
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("drive frame:%d" % num_binary)

    # 自动驾驶，重选/确认颜色
    elif cmd == "82":
        num_verify = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_verify) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            g_mode = 'follow_line_mode'
            if num_verify == 1: # 确认颜色
                g_follow_line_action = 105 # 确认颜色
            else:
                g_follow_line_action = 114 # 重新选择
                set_pwm_servo_state(1)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("drive color verify:%d" % num_verify)
    
    # 自动驾驶，接收PID参数
    elif cmd == "83":
        num_index = int(data[5:7], 16)
        num_pid_value = int(data[7:9], 16)
        checknum = (num_cmd + num_len + num_index + num_pid_value) % 256
        if g_debug:
            print("drive pid:%d, %d" % (num_index, num_pid_value))
        if checknum == num_checknum:
            g_bot.set_beep(20)
            if num_index >= 1 and num_index <= 3:
                index = int(num_index - 1)
                g_follow_line_pid[index] = int(num_pid_value)
                speed = 0.25
                pid_value = (g_follow_line_pid[0] / 1000.0, g_follow_line_pid[1] / 1000.0, g_follow_line_pid[2] / 1000.0)
                g_program.follow_line.set_PID(pid_value, speed)
                g_file.write_Follow_Line_PID(g_follow_line_pid)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))


    # 自动驾驶，重置PID参数
    elif cmd == "85":
        num_verify = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_verify) % 256
        if checknum == num_checknum:
            g_bot.set_beep(20)
            pid_default = g_file.get_follow_pid_default_value()
            for i in range(3):
                g_follow_line_pid[i] = pid_default[i]
            g_file.write_Follow_Line_PID(g_follow_line_pid)
            return_follow_line_pid(g_socket)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("drive reset pid:%d" % num_verify)

    # 使用指引，颜色校准，临时修改HSV值
    elif cmd == "90":
        num_h_min = int(data[5:7], 16)
        num_s_min = int(data[7:9], 16)
        num_v_min = int(data[9:11], 16)
        num_h_max = int(data[11:13], 16)
        num_s_max = int(data[13:15], 16)
        num_v_max = int(data[15:17], 16)
        checknum = (num_cmd + num_len + num_h_min + num_s_min + num_v_min + num_h_max + num_s_max + num_v_max) % 256
        if checknum == num_checknum:
            g_color_hsv_temp = ((num_h_min, num_s_min, num_v_min), (num_h_max, num_s_max, num_v_max))
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("color temp:", num_h_min, num_s_min, num_v_min, num_h_max, num_s_max, num_v_max)

    # 使用指引，颜色校准，确认校准
    elif cmd == "91":
        num_index = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_index) % 256
        if checknum == num_checknum:
            if num_index == 1: # 自动驾驶
                g_follow_line_hsv = g_color_hsv_temp
                write_HSV(g_color_updateHSV.hsv_follow, g_follow_line_hsv)
                if g_debug:
                    print("follow line calibration:", g_color_hsv_temp)
            elif num_index == 2: # 智能追踪
                g_tracker_hsv = g_color_hsv_temp
                write_HSV(g_color_updateHSV.hsv_tracker, g_tracker_hsv)
                if g_debug:
                    print("tracker calibration:", g_color_hsv_temp)
            g_bot.set_beep(20)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        

    # 使用指引，颜色校准，请求数据
    elif cmd == "92":
        num_index = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_index) % 256
        if checknum == num_checknum:
            if num_index == 1: # 自动驾驶
                return_hsv(g_socket, num_index)
                g_color_hsv_temp = g_follow_line_hsv
                g_color_index = 1
            elif num_index == 2: # 智能追踪
                return_hsv(g_socket, num_index)
                g_color_hsv_temp = g_tracker_hsv
                g_color_index = 2
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("color calibration request:%d" % num_index)

    # 使用指引，颜色校准，显示切换
    elif cmd == "93":
        num_index = int(data[5:7], 16)
        checknum = (num_cmd + num_len + num_index) % 256
        if checknum == num_checknum:
            if num_index == 0: # 原图像
                g_color_calibration_binary = False
            else: # 二进制图
                g_color_calibration_binary = True
            g_bot.set_beep(20)
        else:
            if g_debug:
                print("checksum error! cmd:%d, calnum:%d, recvnum:%d" % (int(cmd), checknum, num_checknum))
        if g_debug:
            print("color calibration binary:%d" % num_index)



# socket TCP通信建立
def start_tcp_server(ip, port):
    global g_socket, g_init, g_tcp_except_count
    g_init = True
    if g_debug:
        print('start_tcp_server')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(None)
    sock.bind((ip, port))
    sock.listen(1)

    while True:
        print("Waiting for the client to connect!")
        # times = 0
        tcp_state = 0
        g_tcp_except_count = 0
        g_socket, address = sock.accept()
        print("Connected, Client IP:", address)
        g_ip_addr = g_wifi.get_ip_address()
        print(g_ip_addr)
        while True:
            try:
                # if times == 0:
                #     # 首次连接报版本号和电池电量
                #     return_bot_version(g_socket)
                #     return_battery_voltage(g_socket)
                #     times = 1
                tcp_state = 1
                if g_tcp_ip != g_ip_addr:
                    g_init = False
                    g_socket.close()
                    if g_debug:
                        print("ip addr is changed, Please reconnect tcp server.")
                    return
                tcp_state = 2
                cmd = g_socket.recv(1024).decode(encoding="utf-8")
                if not cmd:
                    break
                tcp_state = 3
                if g_debug:
                    print("   [-]cmd:{0}, len:{1}".format(cmd, len(cmd)))
                tcp_state = 4
                index1 = cmd.rfind("$")
                index2 = cmd.rfind("#")
                if index1 < 0 or index2 <= index1:
                    continue
                tcp_state = 5
                parse_data(g_socket, cmd[index1:index2 + 1])
                g_tcp_except_count = 0
            except:
                if tcp_state == 2:
                    g_tcp_except_count += 1
                    if g_tcp_except_count >= 10:
                        g_tcp_except_count = 0
                        break
                else:
                    if g_debug:
                        print("!!!----TCP Except:%d-----!!!" % tcp_state)
                continue
            # parse_data(g_socket, cmd[index1:index2 + 1])
        print("socket disconnected!")
        g_socket.close()


# 初始化TCP Socket
def init_tcp_socket():
    global g_ip_addr, g_tcp_ip
    if g_init:
        return
    while True:
        ip = g_wifi.get_ip_address()
        if ip == "x.x.x.x":
            g_tcp_ip = ip
            if g_debug:
                print("get ip address fail!")
            time.sleep(.5)
            continue
        if ip != "x.x.x.x":
            g_tcp_ip = ip
            if g_debug:
                print("TCP Service IP=", ip)
            break
    task_tcp = threading.Thread(target=start_tcp_server, name="task_tcp", args=(ip, 6000))
    task_tcp.setDaemon(True)
    task_tcp.start()
    if g_debug:
        print('-------------------Init TCP Socket!-------------------------')


# 根据状态机来运行程序包含视频流返回
def mode_handle():
    global g_mode, g_camera
    global g_follow_line_action, g_visual_action
    global g_action_count
    if g_debug:
        print("----------------------------mode_handle--------------------------")
    m_fps = 0
    t_start = time.time()
    while True:
        success, frame = g_camera.get_frame()
        m_fps = m_fps + 1
        fps = m_fps / (time.time() - t_start)

        # start = time.time()
        # success, frame = g_camera.get_frame()
        # end = time.time()
        # fps = 1 / (end - start)

        text = "FPS:" + str(int(fps))
        if not success:
            g_camera.clear()
            m_fps = 0
            t_start = time.time()
            if g_debug:
                print("-----The camera is reconnecting...")
            g_camera.reconnect()
            time.sleep(.5)
            continue
        # 分析图像，处理数据
        if g_mode == 'Home':
            pass
        # 雷达
        #elif g_mode == 'laser_mode':
            #text = text + '-4'
            # text = "laser"
            # g_program.laser_process()
        # 智能追踪
        elif g_mode == 'visual_mode':
            text = text + '-6'
            # text = "visual"
            frame, binary = g_program.visual_process(g_visual_action, frame)
            if g_visual_action == 114: # 重置, 三次后自动切换为无状态
                g_action_count = g_action_count + 1
                if g_action_count >= 3:
                    g_action_count = 0
                    g_visual_action = 0
        # 自动驾驶
        elif g_mode == 'follow_line_mode':
            text = text + '-8'
            # text = "drive"
            frame, binary = g_program.visual_process(g_follow_line_action, frame)
            if g_follow_line_binary:
                frame = binary
            if g_follow_line_action == 114: # 重置状态只运行一次
                g_action_count = g_action_count + 1
                if g_action_count >= 3:
                    g_action_count = 0
                    g_follow_line_action = 0 # 只显示方框

        # 图像美化
        elif g_mode == 'img_mode':
            text = text + '-3'
            # text = "beautify"
            frame = g_program.img_process(frame)
        # AR
        elif g_mode == 'ar_mode':
            text = text + '-5'
            # text = "AR"
            frame = g_program.img_process(frame)

        # 颜色校准
        elif g_mode == 'color_mode':
            text = text + '-9'
            # text = "color"
            if g_color_index == 1:
                frame, binary, _ = g_color_updateHSV.get_position(frame, g_color_hsv_temp, "Follow_Line")
            else:
                frame, binary, _ = g_color_updateHSV.get_position(frame, g_color_hsv_temp, "None")
            if g_color_calibration_binary:
                frame = binary
            # print("g_color_hsv_temp", g_color_hsv_temp)

        cv.putText(frame, text, (10, 25), cv.FONT_HERSHEY_TRIPLEX, 0.8, (0, 200, 0), 1)
        ret, img_encode = cv.imencode('.jpg', frame)
        if ret:
            img_encode = img_encode.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + img_encode + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    global g_camera
    if g_debug:
        print("----------------------------video_feed--------------------------")
    return Response(mode_handle(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/init')
def init():
    init_tcp_socket()
    return render_template('init.html')


# OLED的线程
# def thread_oled():
#     while True:
#         g_oled.main_program()
#         time.sleep(2)


# 搜索连接的线程
def thread_seek():
    while True:
        g_seek.handle_message()
        time.sleep(2)


if __name__ == '__main__':
    task_pwm_servo = threading.Thread(target=thread_pwm_servo, name="task_pwm_servo")
    task_pwm_servo.setDaemon(True)
    task_pwm_servo.start()

    task_seek = threading.Thread(target=thread_seek, name="task_seek")
    task_seek.setDaemon(True)
    task_seek.start()

    # task_oled = threading.Thread(target=thread_oled, name="task_oled")
    # task_oled.setDaemon(True)
    # task_oled.start()
    
    # init_tcp_socket()

    for i in range(3):
        g_bot.set_beep(60)
        time.sleep(.2)

    g_bot.set_uart_servo_angle(9, 90)
    print("Firmware version:", int(g_info.edition * 10)/10.0)
    print("Waiting for connect to the APP!")

    try:
        # app.run(host='0.0.0.0', port=6500, debug=g_debug)
        server = pywsgi.WSGIServer(('0.0.0.0', 6500), app)
        server.serve_forever()
    except KeyboardInterrupt:
        g_bot.set_car_motion(0, 0)
        g_bot.set_beep(0)
        if g_debug:
            print("-----del g_bot-----")
        del g_bot
