import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	imgInfo = img.shape
	height = imgInfo[0]
	width = imgInfo[1]
	dst =   np.zeros((height,width,3),np.uint8)
	for i in range(0,height):
		for j in range(0,width):
			(b,g,r) = img[i,j]
			bb = int(b*1.4) + 5
			gg = int(g*1.4) + 5
			if bb > 255:
				bb = 255
			if gg > 255:
				gg = 255
			dst[i,j] = (bb,gg,r)
	while True :
		cv2.imshow("origin",img)
		cv2.imshow("dst",dst)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
