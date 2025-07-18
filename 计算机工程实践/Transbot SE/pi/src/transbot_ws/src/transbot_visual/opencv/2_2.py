import cv2
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	dst = img[0:100,100:200]
	while True :
		cv2.imshow("frame",img)
		cv2.imshow('dst', dst)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
