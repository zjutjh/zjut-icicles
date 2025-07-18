#!/usr/bin/env python3
# coding=utf-8
import cv2 as cv
import time

# V3.2.1
class Transbot_Camera(object):

    def __init__(self, video_id=0, width=640, height=480, debug=False):
        self.__debug = debug
        self.__video_id = video_id
        self.__state = False
        self.__width = width
        self.__height = height

        self.__video = cv.VideoCapture(self.__video_id)
        # success, _ = self.__video.read()
        success = self.__video.isOpened()
        if not success:
            self.__video_id = (self.__video_id + 1) % 2
            self.__video = cv.VideoCapture(self.__video_id)
            # success, _ = self.__video.read()
            success = self.__video.isOpened()
            if not success:
                if self.__debug:
                    print("---------Camera Init Error!------------")
                return
        self.__state = True

        self.__config_camera()

        if self.__debug:
            print("---------Video%d Init OK!------------" % self.__video_id)

    def __del__(self):
        if self.__debug:
            print("---------Del Camera!------------")
        self.__video.release()
        self.__state = False

    def __config_camera(self):
        cv_edition = cv.__version__
        if cv_edition[0]=='3':
            self.__video.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else:
            self.__video.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        
        # self.__video.set(cv.CAP_PROP_BRIGHTNESS, 30)  # 设置亮度 -64 - 64  0.0
        # self.__video.set(cv.CAP_PROP_CONTRAST, 50)  # 设置对比度 -64 - 64  2.0
        # self.__video.set(cv.CAP_PROP_EXPOSURE, 156)  # 设置曝光值 1.0 - 5000  156.0
        self.__video.set(cv.CAP_PROP_FRAME_WIDTH, self.__width)  # 640
        self.__video.set(cv.CAP_PROP_FRAME_HEIGHT, self.__height)  # 480

    # 摄像头是否打开成功
    # Check whether the camera is enabled successfully
    def isOpened(self):
        return self.__video.isOpened()

    # 释放摄像头 Release the camera
    def clear(self):
        self.__video.release()

    # 重新连接摄像头 
    # Reconnect the camera
    def reconnect(self):
        self.__video = cv.VideoCapture(self.__video_id)
        success, _ = self.__video.read()
        if not success:
            self.__video_id = (self.__video_id + 1) % 2
            self.__video = cv.VideoCapture(self.__video_id)
            success, _ = self.__video.read()
            if not success:
                if self.__debug:
                    self.__state = False
                    print("---------Camera Reconnect Error!------------")
                return False
        if not self.__state:
            if self.__debug:
                print("---------Video%d Reconnect OK!------------" % self.__video_id)
            self.__state = True
            self.__config_camera()
        return True

    # 获取摄像头的一帧图片 
    # Gets a frame of the camera
    def get_frame(self):
        success, image = self.__video.read()
        if not success:
            return success, bytes({1})
        return success, image

    # 获取摄像头的jpg图片 
    # Gets the JPG image of the camera
    def get_frame_jpg(self, text="", color=(0, 255, 0)):
        success, image = self.__video.read()
        if not success:
            return success, bytes({1})
        if text != "":
            # 各参数依次是：图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
            # The parameters are: image, added text, top left coordinate, font, font size, color, font size  
            cv.putText(image, str(text), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        success, jpeg = cv.imencode('.jpg', image)
        return success, jpeg.tobytes()


if __name__ == '__main__':
    camera = Transbot_Camera(debug=True)
    average = False
    m_fps = 0
    t_start = time.time()
    while camera.isOpened():
        if average:
            ret, frame = camera.get_frame()
            m_fps = m_fps + 1
            fps = m_fps / (time.time() - t_start)
            
        else:
            start = time.time()
            ret, frame = camera.get_frame()
            end = time.time()
            fps = 1 / (end - start)
        text="FPS:" + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 1)
        cv.imshow('frame', frame)

        k = cv.waitKey(1) & 0xFF
        if k == 27 or k == ord('q'):
            break
    del camera
    cv.destroyAllWindows()
