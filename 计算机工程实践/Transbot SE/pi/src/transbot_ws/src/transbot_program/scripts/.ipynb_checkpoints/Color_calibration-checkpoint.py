#!/usr/bin/env python
# encoding: utf-8
import time
from Color_Common import *

color_updateHSV = ColorFollow()
color_hsv = ((0, 85, 126), (9, 253, 255))
Hmin, Smin, Vmin = 0, 85, 126
Hmax, Smax, Vmax = 255, 9, 253

if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    cv_edition = cv.__version__
    if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # Enter color calibration
    color = 1
    if color == 1: hsv_text = color_updateHSV.hsv_follow
    else: hsv_text = color_updateHSV.hsv_tracker
    try: color_hsv = read_HSV(hsv_text)
    except Exception: print("Read HSV_config Error!!!")
    while capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        action = cv.waitKey(10) & 0xFF
        color_hsv = ((Hmin, Smin, Vmin), (Hmax, Smax, Vmax))
        frame, binary, _ = color_updateHSV.get_position(frame, color_hsv, "None")
        if action == 32:
            cv.bitwise_not(frame, frame)
            write_HSV(hsv_text, color_hsv)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))
        else:cv.imshow('frame', frame)
        # cv.imshow('frame', frame)
        if action == ord('q') or action == 113: break
    capture.release()
    cv.destroyAllWindows()
