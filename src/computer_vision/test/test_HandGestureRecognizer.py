import cv2
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))
from HandGestureRecognizer import *
from Helpers import *

def test_subtractBackground():
	cap = cv2.VideoCapture(0)
	handGestureRecognizer = HandGestureRecognizer(cap)
	while(1):
	    cv2.imshow('frame',handGestureRecognizer.subtractBackground())
	    k = cv2.waitKey(30) & 0xff
	    if k == 27:
	        break

def test_calibrate():
	cap = cv2.VideoCapture(0)
	handGestureRecognizer = HandGestureRecognizer(cap)
	# handGestureRecognizer.calibrate()

def test_track():
	colors = [[220, 160,  92], [215, 152,  59], [199, 148,  79], [184, 143,  85], [188, 136,  56], [208, 158,  93], [204, 152,  51], [153, 126,  66], [188, 135,  51], [183, 137,  67], [169, 127,  65], [166, 128,  74], [185, 136,  68], [177, 141,  95], [192, 146,  82]]
	cap = cv2.VideoCapture(0)
	handGestureRecognizer = HandGestureRecognizer(cap)
	handGestureRecognizer._trackHand(colors)

def test_drawContour():
	im = cv2.imread('test_image.jpg')
	imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(imgray,127,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(im,contours,-1,(0,255,0),3)
	showImage(im)

if __name__ == "__main__":
	# test_drawContour()
	# test_track()
	# test_calibrate()