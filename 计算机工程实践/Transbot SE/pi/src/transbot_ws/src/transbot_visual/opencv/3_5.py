import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	rect = cv2.rectangle(img, (50,20), (100,100), (255,0,255), 10)
	while True :
		cv2.imshow("line",rect)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
