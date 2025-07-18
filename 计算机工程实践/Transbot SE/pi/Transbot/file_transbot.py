#!/usr/bin/env python3
# coding=utf-8
import os

# V3.2.1
class Transbot_File:
    def __init__(self, debug=False):
        self.__follow_pid_default = [20, 0, 10]
        self.__debug = debug
        self.__path = '/home/yahboom/Transbot/transbot/'

    # 获取默认的PID值，可用于重置PID值。
    # Gets the default PID value, which can be used to reset the PID value.
    def get_follow_pid_default_value(self):
        return self.__follow_pid_default

    # 写入自动驾驶的PID值
    # Write the PID value for autopilot
    def write_Follow_Line_PID(self, value):
        try:
            wf_path = self.__path + 'TransbotFollowPID.text'
            with open(wf_path, "w") as wf:
                wf_str = str(int(value[0])) + ', ' + str(int(value[1])) + ', ' + str(int(value[2]))
                wf.write(wf_str)
                wf.flush()
            if self.__debug:
                print("follow line pid data is writed：", value)
        except:
            print("!!!---follow line pid file write error---!!!")

    # 读取自动驾驶的PID值
    # Read the PID value of autopilot
    def read_Follow_Line_PID(self):
        try:
            rf_path = self.__path + 'TransbotFollowPID.text'
            rf = open(rf_path, "r")
            line = rf.readline()
            if len(line) == 0:
                print("!!!---follow line pid file len=0, return default value---!!!")
                return self.__follow_pid_default
            list = line.split(',')
            if len(list) != 3:
                print("!!!---follow line pid file list!=3, return default value---!!!")
                return self.__follow_pid_default
            pid = [int(list[0]), int(list[1]), int(list[2])]
            rf.flush()
        except FileNotFoundError:
            print("!!!---follow line pid file not found, return default value---!!!")
            pid = self.__follow_pid_default
        return pid

if __name__ == '__main__':
    bot_file = Transbot_File(debug=True)
    try:
        input_pid = [20, 0, 10]
        bot_file.write_Follow_Line_PID(input_pid)
        read_pid = bot_file.read_Follow_Line_PID()
        print("read pid:", read_pid)
    except KeyboardInterrupt:
        pass
    del bot_file
    print('file_transbot.py killed')

