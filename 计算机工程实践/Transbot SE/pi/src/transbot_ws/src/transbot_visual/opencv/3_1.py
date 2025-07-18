import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	while True :
		cv2.imshow("frame",img)
		cv2.imshow('gray', gray)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
