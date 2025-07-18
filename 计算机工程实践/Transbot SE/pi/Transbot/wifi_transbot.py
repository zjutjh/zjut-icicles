#!/usr/bin/env python3
# coding=utf-8
import cv2
import pyzbar.pyzbar as pyzbar
import threading
import RPi.GPIO as GPIO
import os
import time
from Transbot_Lib import Transbot


# V3.2.1
class Transbot_WIFI:
    def __init__(self, bot, debug=False):
        self.KEY1_PIN = 17
        self.LED_PIN = 18

        self.key1_pressed = False
        self.config_Mode = False
        self.count = 0
        self.SSID = ''
        self.PASSWD = ''
        self.ip = 'x.x.x.x'
        self.__debug = debug

        self.key_scan_state = 0
        self.bot = bot

        self.led_count = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.KEY1_PIN, GPIO.IN)
        GPIO.setup(self.LED_PIN, GPIO.OUT, initial=GPIO.LOW)

        task_key = threading.Thread(target=self.key_scan, name="task_key")
        task_key.setDaemon(True)
        task_key.start()

    # 获取本机IP
    # Read the local IP address
    def get_ip_address(self):
        ip = os.popen(
            "/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0: ip.find('\n')]
        if(ip == ''):
            ip = os.popen(
                "/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
            ip = ip[0: ip.find('\n')]
            if(ip == ''):
                ip = 'x.x.x.x'
        if len(ip) > 15:
            ip = 'x.x.x.x'
        return ip

    # 按键扫描
    # Scan button
    def key_scan(self):
        if self.__debug:
            print("-----create key threading-----")
        while True:
            if GPIO.input(self.KEY1_PIN) == GPIO.LOW:
                time.sleep(0.05)
                if GPIO.input(self.KEY1_PIN) == GPIO.LOW:
                    if self.key1_pressed == False:
                        self.key1_pressed = True
                        self.count = 0
                        if self.__debug:
                            print("key1_pressed start")
                    else:
                        self.count += 1
                        # 长按K1键进入配网模式 
                        # Long press K1 to enter network mode
                        if self.count == 40: 
                            self.bot.set_beep(50)
                            self.config_Mode = not self.config_Mode
                            if self.__debug:
                                print("key1_pressed long event")
                else:
                    self.count = 0
                    self.key1_pressed = False
            else:
                self.count = 0
                self.key1_pressed = False
                time.sleep(0.1)

    # 读取当前模式，进入摄像头识别二维码时返回True，否则返回False。
    # Read the current mode, enter the camera to recognize the TWO-DIMENSIONAL code return True, otherwise return False
    def read_mode(self):
        return self.config_Mode

    # LED显示WiFi连接状态，已连接灯亮，未连接灯灭。
    # LED display WiFi connection status, connected to the light, not connected to the light off
    def led_show_state(self):
        self.ip = self.get_ip_address()
        if self.ip == 'x.x.x.x':
            self.wifi_LED_OFF()
        else:
            self.wifi_LED_ON()
        return self.ip

    # 打开WiFi指示灯
    # Turn on WiFi indicator
    def wifi_LED_ON(self):
        GPIO.output(self.LED_PIN, GPIO.HIGH)

    # 关闭WiFi指示灯
    # Turn off WiFi indicator
    def wifi_LED_OFF(self):
        GPIO.output(self.LED_PIN, GPIO.LOW)

    # 定义解析二维码接口
    # Define and parse the TWO-DIMENSIONAL code interface
    def decodeDisplay(self, image):
        barcodes = pyzbar.decode(image)
        SSID = ''
        PASSWD = ''
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(image, (x, y), (x + w, y + h), (225, 225, 225), 2)
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(image, text, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            a = barcodeData.find('SSID')
            b = barcodeData.find('|')
            SSID = barcodeData[6:b-1]
            PASSWD = barcodeData[b+2:-1]
            if self.__debug:
                print("Find SSID:", SSID)
                print("PASSWD:", PASSWD)
                print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return image, SSID, PASSWD

    # 摄像头连接wifi, 返回IP地址和图像
    # The camera connects to wifi and returns the IP address and image
    def connect(self, frame):
        try:
            self.led_count = self.led_count + 1
            if self.led_count == 5:
                self.bot.set_beep(30)
                self.wifi_LED_ON()
            elif self.led_count >= 10:
                self.wifi_LED_OFF()
                self.led_count = 0
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img, self.SSID, self.PASSWD = self.decodeDisplay(gray)
            if(self.SSID != '' and self.PASSWD != ''):
                if self.__debug:
                    print("Connecting WIFI! Please Wait a minute...")
                self.bot.set_beep(1000)
                passwd = "yahboom"
                cmd_scan = "sudo iw dev wlan0 scan | grep " + self.SSID
                os.system('echo %s | sudo -S %s' % (passwd, cmd_scan))
                cmd = "sudo nmcli dev wifi connect \'" + \
                    self.SSID + "\' password \'" + self.PASSWD + "\'"
                os.system('echo %s | sudo -S %s' % (passwd, cmd))
                
                self.SSID = ''
                self.PASSWD = ''
                time.sleep(.2)
                self.config_Mode = False
                for ip in range(5):
                    self.ip = self.get_ip_address()
                    if self.ip != "x.x.x.x":
                        for i in range(3):
                            self.bot.set_beep(100)
                            time.sleep(.2)
                        if self.__debug:
                            print("WIFI Connect OK! IP=", self.ip)
                        return self.ip, img
                    time.sleep(1)
                if self.__debug:
                    print("Connect WIFI Error!")
        except:
            img = frame
        return "x.x.x.x", img

    # Cleanup GPIO 
    def destroy(self):
        GPIO.cleanup()


if __name__ == '__main__':
    transbot = Transbot()
    wifi = Transbot_WIFI(transbot, debug=True)
    try:
        average = True
        m_fps = 0
        t_start = time.time()
        my_camera = cv2.VideoCapture(0)
        cv_edition = cv2.__version__
        if cv_edition[0]=='3':
            my_camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'XVID'))
        else:
            my_camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        
        while my_camera.isOpened():
            if average:
                success, frame = my_camera.read()
                m_fps = m_fps + 1
                fps = m_fps / (time.time() - t_start)
            else:
                start = time.time()
                success, frame = my_camera.read()
                end = time.time()
                fps = 1 / (end - start)
            text="FPS:" + str(int(fps))
            cv2.putText(frame, text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 1)

            cv2.imshow("img", frame)
            if wifi.read_mode():
                ip, img = wifi.connect(frame)
                if ip != 'x.x.x.x':
                    print("IP:" + ip)
                    break

            action = cv2.waitKey(10) & 0xff
            if action == ord('q') or action == 27:
                break
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    del my_camera
    print('wifi_transbot.py killed')

