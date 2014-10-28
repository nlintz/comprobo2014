import cv2
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))
from HandGestureRecognizer import *

def test_subtractBackground():
	cap = cv2.VideoCapture(0)
	handGestureRecognizer = HandGestureRecognizer()
