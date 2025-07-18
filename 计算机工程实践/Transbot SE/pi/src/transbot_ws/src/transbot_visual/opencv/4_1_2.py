import cv2
import numpy as np
if __name__ == '__main__':
	dam_img = cv2.imread('damaged.jpg')
	imgInfo = dam_img.shape
	height = imgInfo[0]
	width = imgInfo[1]
	paint =   np.zeros((height,width,1),np.uint8)
	for i in range(50,100):
		paint[i,50] = 255
		paint[i,50+1] = 255
		paint[i,50-1] = 255
	for i in range(100,150):
		paint[150,i] = 255
		paint[150+1,i] = 255
		paint[150-1,i] = 255
	dst_img =  cv2.inpaint(dam_img,paint,3,cv2.INPAINT_TELEA)
	while True :
		cv2.imshow("dam_img",dam_img)
		cv2.imshow("paint",paint)
		cv2.imshow("dst",dst_img)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
