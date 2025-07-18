import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	ret,thresh1=cv2.threshold(gray,180,255,cv2.THRESH_BINARY_INV) 
	while True :
		cv2.imshow("frame",img)
		cv2.imshow('gray', gray)
		cv2.imshow("binary",thresh1)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
