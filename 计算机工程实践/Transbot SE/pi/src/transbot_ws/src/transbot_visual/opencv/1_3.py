import cv2 as cv
if __name__ == '__main__':
	frame = cv.VideoCapture(0)
	while frame.isOpened():
		ret,img = frame.read()
		cv.imshow('frame',img)
		action = cv.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
frame.release()
cv.destroyAllWindows()
