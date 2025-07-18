#!/usr/bin/env python3
# coding=utf-8
import socket
import time

# V3.2.1
class Transbot_Seek:
    def __init__(self, debug):
        self.__host = '0.0.0.0'
        self.__port = 8000
        self.__addr = (self.__host, self.__port)
        self.__debug = debug

    
    def __del__(self):
        if self.__debug:
            print("---Seek-Del---")


    # 消息处理函数，需要在循环中运行。
    # Message handler function that needs to run in a loop
    def handle_message(self):
        try:
            udpServer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udpServer.settimeout(None)
            udpServer.bind(self.__addr)
            if self.__debug:
                print("---start seek handle---")
            while True:
                data, addr = udpServer.recvfrom(1024)
                msg = str(data, encoding = 'utf-8')
                if self.__debug:
                    print("---UDP Recv:", msg)
                if msg == "YAHBOOM_FIND":
                    udpServer.sendto(bytes("Transbot_Jetson_4G_V2", encoding='utf-8'), addr)
                    if self.__debug:
                        print("---UDP Send {0} OK!---".format(addr))
                time.sleep(1)
        except:
            if self.__debug:
                print("!!!---Seek msg error ---!!!")
            pass


if __name__ == "__main__":
    try:
        seek = Transbot_Seek(True)
        while True:
            seek.handle_message()
            time.sleep(2)
    except KeyboardInterrupt:
        print(" Program closed! ")
        pass
