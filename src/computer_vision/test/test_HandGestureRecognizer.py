import cv2
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))
from HandGestureRecognizer import *

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
	handGestureRecognizer.calibrate()


if __name__ == "__main__":
	test_calibrate()