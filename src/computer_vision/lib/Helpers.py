import cv2
def showImage(img):
	cv2.imshow('myimage', img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
