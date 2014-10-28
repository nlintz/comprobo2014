import numpy as np
import cv2

class HandGestureRecognizer(object):
	""" Class for converting video footage to a hand gesture 
	"""
	def __init__(self, videoFeed):
		""" 
		videoCapture: capture object returned from cv2.VideoCapture
		"""
		self._videoFeed = videoFeed

	def subtractBackground(self):
		ret, img = self._videoFeed.read()
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(gray,(5,5),0)
		ret,thresh = cv2.threshold(blur,70,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

		return thresh