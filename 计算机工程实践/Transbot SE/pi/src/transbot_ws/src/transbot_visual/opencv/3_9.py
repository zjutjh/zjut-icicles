import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	cv2.putText(img,'This is Yahboom!',(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,200,0),2)
	while True :
		cv2.imshow("img",img)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
