import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	circle = cv2.circle(img, (80,80), 50, (255,0,255), 10)
	while True :
		cv2.imshow("circle",circle)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
