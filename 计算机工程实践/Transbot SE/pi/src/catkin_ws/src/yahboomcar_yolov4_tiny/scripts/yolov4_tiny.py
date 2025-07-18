#!/usr/bin/env python3.7
# encoding: utf-8
import sys
import time
import rospy
import rospkg
import base64
import cv2 as cv
import numpy as np
import tensorflow as tf
from utils.yolo import YOLO
from yahboomcar_msgs.msg import *

gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)

class YoloDetect:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("YoloDetect", anonymous=False)
        self.pTime = self.cTime = 0
        rospkg_path = rospkg.RosPack().get_path("yahboomcar_yolov4_tiny") + ''
        model_path = rospkg_path + '/param/yolov4_tiny_weights_coco.h5'
        classes_path = rospkg_path + '/param/coco.txt'
        self.yolov4_tiny = YOLO(rospkg_path, model_path, classes_path)
        self.pub_image = rospy.Publisher('Detect/image_msg', Image_Msg, queue_size=10)
        self.pub_msg = rospy.Publisher('DetectMsg', TargetArray, queue_size=10)

    def cancel(self):
        self.pub_image.unregister()
        self.pub_msg.unregister()

    def pub_imgMsg(self, frame):
        pic_base64 = base64.b64encode(frame)
        image = Image_Msg()
        size = frame.shape
        image.height = size[0]
        image.width = size[1]
        image.channels = size[2]
        image.data = pic_base64
        self.pub_image.publish(image)

    def detect(self, frame):
        target_array = TargetArray()
        target = Target()
        # 格式转变，BGRtoRGB
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        frame, out_boxes, out_scores, out_classes = self.yolov4_tiny.detect_image(frame)
        for i, c in list(enumerate(out_classes)):
            predicted_class = self.yolov4_tiny.class_names[c]
            box = out_boxes[i]
            score = out_scores[i]
            self.yolov4_tiny.draw_img(frame, c, box, score, predicted_class)
            target.frame_id = predicted_class
            target.stamp = rospy.Time.now()
            target.scores = score
            # x1, y1, x2, y2  top, left, bottom, right
            target.ptx = box[0]
            target.pty = box[1]
            target.distw = box[2] - box[0]
            target.disth = box[3] - box[1]
            target.centerx = (box[2] - box[0]) / 2
            target.centery = (box[3] - box[1]) / 2
            target_array.data.append(target)
        self.cTime = time.time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        frame = np.array(frame)
        # RGBtoBGR满足opencv显示格式
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.pub_msg.publish(target_array)
        self.pub_imgMsg(frame)
        return frame

if __name__ == "__main__":
    print("Python version: ", sys.version)
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    detect = YoloDetect()
    while capture.isOpened():
        ret, frame = capture.read()
        action = cv.waitKey(1) & 0xFF
        frame = detect.detect(frame)
        if action == ord('q'): break
        if len(sys.argv) != 1:
            if sys.argv[1]=="true" or sys.argv[1]=="True": cv.imshow('frame', frame)
        else: cv.imshow('frame', frame)
    detect.cancel()
    capture.release()
    cv.destroyAllWindows()
