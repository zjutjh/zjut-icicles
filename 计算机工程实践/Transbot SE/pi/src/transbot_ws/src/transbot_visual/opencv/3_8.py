import cv2
import numpy as np
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	points = np.array([[120,50], [40,140], [120,70], [110,110], [50,50]], np.int32)
	polylines = cv2.polylines(img, [points],True,(255,0,255), 5)
	while True :
		cv2.imshow("polylines",polylines)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
