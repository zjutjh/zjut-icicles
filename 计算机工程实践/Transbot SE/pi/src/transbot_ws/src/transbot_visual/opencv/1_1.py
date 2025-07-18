import cv2 as cv

if __name__ == '__main__':
	img = cv.imread('yahboom.jpg')
	while True :
		cv.imshow("frame",img)
		action = cv.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv.destroyAllWindows()
