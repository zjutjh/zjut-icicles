import cv2
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	print(img.shape)
	x, y = img.shape[0:2]
	img_test1 = cv2.resize(img, (int(y / 2),   int(x / 2)))
	while True :
		cv2.imshow("frame",img)
		cv2.imshow('resize0', img_test1)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
