import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	for i in range(50,100):
		img[i,50] =   (0,0,0)
		img[i,50+1] =   (0,0,0)
		img[i,50-1] =   (0,0,0)
	for i in range(100,150):
		img[150,i] =   (0,0,0)
		img[150,i+1] =   (0,0,0)
		img[150-1,i] =   (0,0,0)
	cv2.imwrite("damaged.jpg",img)
	dam_img = cv2.imread('damaged.jpg')
	while True :
		cv2.imshow("dam_img",dam_img)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
