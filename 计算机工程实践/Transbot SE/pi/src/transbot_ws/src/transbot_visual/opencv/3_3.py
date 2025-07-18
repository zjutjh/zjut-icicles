import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	imgG = cv2.GaussianBlur(gray,(3,3),0)
	dst = cv2.Canny(imgG,50,50)
	while True :
		cv2.imshow("frame",img)
		cv2.imshow('gray', gray)
		cv2.imshow("canny",dst)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
