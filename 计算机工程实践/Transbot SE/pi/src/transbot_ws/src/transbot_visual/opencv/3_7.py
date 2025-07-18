import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	ellipse = cv2.ellipse(img, (80,80), (20,50),0,0, 360,(255,0,255), 5)
	while True :
		cv2.imshow("ellipse",ellipse)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
