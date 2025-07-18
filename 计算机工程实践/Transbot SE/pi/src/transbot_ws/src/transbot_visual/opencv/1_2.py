import cv2 as cv

if __name__ == '__main__':
	img = cv.imread('yahboom.jpg')
	cv.imwrite("yahboom_new.jpg",img)
	new_img = cv.imread('yahboom_new.jpg')
	while True :
		cv.imshow("frame",img)
		cv.imshow("new_frame",new_img)
		action = cv.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv.destroyAllWindows()
