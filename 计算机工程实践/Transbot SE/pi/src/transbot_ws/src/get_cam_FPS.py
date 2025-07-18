#!/usr/bin/env python
# encoding: utf-8
import cv2 as cv
import time

capture = cv.VideoCapture(0)
capture.set(6,cv.VideoWriter.fourcc('M','J','P','G'))
capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
print ("capture get FPS : ",capture.get(cv.CAP_PROP_FPS))
while capture.isOpened():
    start = time.time()
    ret, frame = capture.read()
    if cv.waitKey(1) & 0xFF == ord('q'): break
    end = time.time()
    fps = 1 / (end - start)
    text="FPS : "+str(int(fps))
    cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
    cv.imshow('frame', frame)
capture.release()
cv.destroyAllWindows()
