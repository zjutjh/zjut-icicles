import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	imgInfo = img.shape
	height = imgInfo[0]
	width = imgInfo[1]
	matShift = np.float32([[1,0,10],[0,1,10]])# 2*3
	dst = cv2.warpAffine(img, matShift, (width,height))
	while True :
		cv2.imshow("frame",img)
		cv2.imshow('dst', dst)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
