import cv2
if __name__ == '__main__':
	img = cv2.imread('yahboom.jpg')
	(b,g,r) = img[100,100]
	print(b,g,r)
	i=j=0
	for j in range(1,255):
		img[i,j] = (0,0,0)
		for j in range(1,255):
			img[i,j] = (0,0,0)
	while True :
		cv2.imshow("frame",img)
		action = cv2.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv2.destroyAllWindows()
